/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gps_checks.cpp
 * 执行飞行前和飞行中 GPS 质量检查
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"

#include <ecl.h>
#include <geo_lookup/geo_mag_declination.h>
#include <mathlib/mathlib.h>

// GPS 飞行前检查位位置
#define MASK_GPS_NSATS  (1<<0)
#define MASK_GPS_PDOP   (1<<1)
#define MASK_GPS_HACC   (1<<2)
#define MASK_GPS_VACC   (1<<3)
#define MASK_GPS_SACC   (1<<4)
#define MASK_GPS_HDRIFT (1<<5)
#define MASK_GPS_VDRIFT (1<<6)
#define MASK_GPS_HSPD   (1<<7)
#define MASK_GPS_VSPD   (1<<8)

bool Ekf::collect_gps(const gps_message &gps)
{
	// 始终运行 GPS 检查
	_gps_checks_passed = gps_is_good(gps);

	if (_filter_initialised && !_NED_origin_initialised && _gps_checks_passed) {
		// 如果 GPS 数据良好，则将原点的 WGS-84 位置设为最后一次 GPS 定位结果
		const double lat = gps.lat * 1.0e-7;
		const double lon = gps.lon * 1.0e-7;

		if (!map_projection_initialized(&_pos_ref)) {
			map_projection_init_timestamped(&_pos_ref, lat, lon, _time_last_imu);

			// 如果已经在执行辅助导航，需修正自 EKF 开始导航以来的位置变化
			if (isHorizontalAidingActive()) {
				double est_lat;
				double est_lon;
				map_projection_reproject(&_pos_ref, -_state.pos(0), -_state.pos(1), &est_lat, &est_lon);
				map_projection_init_timestamped(&_pos_ref, est_lat, est_lon, _time_last_imu);
			}
		}

		// 取当前 GPS 高度并减去原点上方滤波器高度，以估算原点的 GPS 高度
		_gps_alt_ref = 1e-3f * (float)gps.alt + _state.pos(2);
		_NED_origin_initialised = true;
		_earth_rate_NED = calcEarthRateNED((float)_pos_ref.lat_rad);
		_last_gps_origin_time_us = _time_last_imu;

		const bool declination_was_valid = ISFINITE(_mag_declination_gps);

		// 使用当前 GPS 位置设置地理库返回的磁场数据
		_mag_declination_gps = get_mag_declination_radians(lat, lon);
		_mag_inclination_gps = get_mag_inclination_radians(lat, lon);
		_mag_strength_gps = get_mag_strength_gauss(lat, lon);

		// 使用新偏角请求偏航重置
		if (_params.mag_fusion_type == MAG_FUSE_TYPE_NONE) {
			// 尝试使用 EKF-GSF 偏航估计器重置偏航
			_do_ekfgsf_yaw_reset = true;
			_ekfgsf_yaw_reset_time = 0;

		} else {
			if (!declination_was_valid) {
				_mag_yaw_reset_req = true;
			}
		}

		// 保存原点的水平和垂直位置不确定度
		_gps_origin_eph = gps.eph;
		_gps_origin_epv = gps.epv;

		// 如果用户将 GPS 作为主高度源，则切换到使用 GPS 高度
		if (_params.vdist_sensor_type == VDIST_SENSOR_GPS) {
			startGpsHgtFusion();
		}

		_information_events.flags.gps_checks_passed = true;
		ECL_INFO("GPS checks passed");

	} else if (!_NED_origin_initialised) {
		// 粗略 2D 定位仍足以查找偏角
		if ((gps.fix_type >= 2) && (gps.eph < 1000)) {

			const bool declination_was_valid = ISFINITE(_mag_declination_gps);

			// 如果 GPS 数据质量良好，则将原点的 WGS-84 位置设为最近一次 GPS 定位值
			const double lat = gps.lat * 1.0e-7;
			const double lon = gps.lon * 1.0e-7;

			// set the magnetic field data returned by the geo library using the current GPS position
			_mag_declination_gps = get_mag_declination_radians(lat, lon);
			_mag_inclination_gps = get_mag_inclination_radians(lat, lon);
			_mag_strength_gps = get_mag_strength_gauss(lat, lon);

			// request mag yaw reset if there's a mag declination for the first time
			if (_params.mag_fusion_type != MAG_FUSE_TYPE_NONE) {
				if (!declination_was_valid && ISFINITE(_mag_declination_gps)) {
					_mag_yaw_reset_req = true;
				}
			}

			_earth_rate_NED = calcEarthRateNED((float)math::radians(lat));
		}
	}

	// 如果存在 3D 定位且已设置 NED 原点，则开始收集 GPS
	return _NED_origin_initialised && (gps.fix_type >= 3);
}

/*
* 返回 true，当 GPS 解质量足以设定 EKF 原点并开始 GPS 辅助。
* 所有已启用的检查必须连续通过 10 秒。
* 检查通过 EKF2_GPS_CHECK 位掩码参数启用
* 通过 EKF2_REQ_* 参数调整检查阈值
*/
bool Ekf::gps_is_good(const gps_message &gps)
{
	// 检查定位类型
	_gps_check_fail_status.flags.fix = (gps.fix_type < 3);

	// 检查卫星数量
	_gps_check_fail_status.flags.nsats = (gps.nsats < _params.req_nsats);

	// 检查位置精度稀释因子
	_gps_check_fail_status.flags.pdop = (gps.pdop > _params.req_pdop);

	// 检查报告的水平和垂直位置精度
	_gps_check_fail_status.flags.hacc = (gps.eph > _params.req_hacc);
	_gps_check_fail_status.flags.vacc = (gps.epv > _params.req_vacc);

	// 检查报告的速度精度
	_gps_check_fail_status.flags.sacc = (gps.sacc > _params.req_sacc);

	// 检查 GPS 质量是否退化
	_gps_error_norm = fmaxf((gps.eph / _params.req_hacc), (gps.epv / _params.req_vacc));
	_gps_error_norm = fmaxf(_gps_error_norm, (gps.sacc / _params.req_sacc));

	// 计算自上次更新以来经过的时间，限制以避免数值误差，并计算低通滤波系数
	constexpr float filt_time_const = 10.0f;
	const float dt = math::constrain(float(int64_t(_time_last_imu) - int64_t(_gps_pos_prev.timestamp)) * 1e-6f, 0.001f, filt_time_const);
	const float filter_coef = dt / filt_time_const;

	// 下列检查仅在飞行器静止时有效
	const double lat = gps.lat * 1.0e-7;
	const double lon = gps.lon * 1.0e-7;

	if (!_control_status.flags.in_air && _control_status.flags.vehicle_at_rest) {
		// 计算自上次测量以来的位置变化
		float delta_pos_n = 0.0f;
		float delta_pos_e = 0.0f;

		// 计算自上次 GPS 定位以来的位置变化
		if (_gps_pos_prev.timestamp > 0) {
			map_projection_project(&_gps_pos_prev, lat, lon, &delta_pos_n, &delta_pos_e);

		} else {
			// no previous position has been set
			map_projection_init_timestamped(&_gps_pos_prev, lat, lon, _time_last_imu);
			_gps_alt_prev = 1e-3f * (float)gps.alt;
		}

		// Calculate the horizontal and vertical drift velocity components and limit to 10x the threshold
		const Vector3f vel_limit(_params.req_hdrift, _params.req_hdrift, _params.req_vdrift);
		Vector3f pos_derived(delta_pos_n, delta_pos_e, (_gps_alt_prev - 1e-3f * (float)gps.alt));
		pos_derived = matrix::constrain(pos_derived / dt, -10.0f * vel_limit, 10.0f * vel_limit);

		// Apply a low pass filter
		_gps_pos_deriv_filt = pos_derived * filter_coef + _gps_pos_deriv_filt * (1.0f - filter_coef);

		// Calculate the horizontal drift speed and fail if too high
		_gps_drift_metrics[0] = Vector2f(_gps_pos_deriv_filt.xy()).norm();
		_gps_check_fail_status.flags.hdrift = (_gps_drift_metrics[0] > _params.req_hdrift);

		// Fail if the vertical drift speed is too high
		_gps_drift_metrics[1] = fabsf(_gps_pos_deriv_filt(2));
		_gps_check_fail_status.flags.vdrift = (_gps_drift_metrics[1] > _params.req_vdrift);

		// Check the magnitude of the filtered horizontal GPS velocity
		const Vector2f gps_velNE = matrix::constrain(Vector2f(gps.vel_ned.xy()),
							     -10.0f * _params.req_hdrift,
							      10.0f * _params.req_hdrift);
		_gps_velNE_filt = gps_velNE * filter_coef + _gps_velNE_filt * (1.0f - filter_coef);
		_gps_drift_metrics[2] = _gps_velNE_filt.norm();
		_gps_check_fail_status.flags.hspeed = (_gps_drift_metrics[2] > _params.req_hdrift);

		_gps_drift_updated = true;

	} else if (_control_status.flags.in_air) {
		// These checks are always declared as passed when flying
		// If on ground and moving, the last result before movement commenced is kept
		_gps_check_fail_status.flags.hdrift = false;
		_gps_check_fail_status.flags.vdrift = false;
		_gps_check_fail_status.flags.hspeed = false;
		_gps_drift_updated = false;

		resetGpsDriftCheckFilters();

	} else {
		// This is the case where the vehicle is on ground and IMU movement is blocking the drift calculation
		_gps_drift_updated = true;

		resetGpsDriftCheckFilters();
	}

	// save GPS fix for next time
	map_projection_init_timestamped(&_gps_pos_prev, lat, lon, _time_last_imu);
	_gps_alt_prev = 1e-3f * (float)gps.alt;

	// Check  the filtered difference between GPS and EKF vertical velocity
	const float vz_diff_limit = 10.0f * _params.req_vdrift;
	const float vertVel = math::constrain(gps.vel_ned(2) - _state.vel(2), -vz_diff_limit, vz_diff_limit);
	_gps_velD_diff_filt = vertVel * filter_coef + _gps_velD_diff_filt * (1.0f - filter_coef);
	_gps_check_fail_status.flags.vspeed = (fabsf(_gps_velD_diff_filt) > _params.req_vdrift);

	// assume failed first time through
	if (_last_gps_fail_us == 0) {
		_last_gps_fail_us = _time_last_imu;
	}

	// if any user selected checks have failed, record the fail time
	if (
		_gps_check_fail_status.flags.fix ||
		(_gps_check_fail_status.flags.nsats   && (_params.gps_check_mask & MASK_GPS_NSATS)) ||
		(_gps_check_fail_status.flags.pdop    && (_params.gps_check_mask & MASK_GPS_PDOP)) ||
		(_gps_check_fail_status.flags.hacc    && (_params.gps_check_mask & MASK_GPS_HACC)) ||
		(_gps_check_fail_status.flags.vacc    && (_params.gps_check_mask & MASK_GPS_VACC)) ||
		(_gps_check_fail_status.flags.sacc    && (_params.gps_check_mask & MASK_GPS_SACC)) ||
		(_gps_check_fail_status.flags.hdrift  && (_params.gps_check_mask & MASK_GPS_HDRIFT)) ||
		(_gps_check_fail_status.flags.vdrift  && (_params.gps_check_mask & MASK_GPS_VDRIFT)) ||
		(_gps_check_fail_status.flags.hspeed  && (_params.gps_check_mask & MASK_GPS_HSPD)) ||
		(_gps_check_fail_status.flags.vspeed  && (_params.gps_check_mask & MASK_GPS_VSPD))
	) {
		_last_gps_fail_us = _time_last_imu;

	} else {
		_last_gps_pass_us = _time_last_imu;
	}

	// continuous period without fail of x seconds required to return a healthy status
	return isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us);
}
