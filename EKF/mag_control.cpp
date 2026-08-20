/****************************************************************************
 *
 *   Copyright (c) 2019 Estimation and Control Library (ECL). All rights reserved.
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
 * @file mag_control.cpp
 * EKF 磁场融合的控制函数
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlMagFusion()
{
	checkMagFieldStrength();

	// 如果在地面上，则重置飞行对齐标志，以便在下次达到飞行高度时重新初始化磁场
	if (!_control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = false;
		_num_bad_flight_yaw_events = 0;
	}

	// 当没有磁力计且没有其他偏航辅助源处于活动状态时，
	// 会选择性地运行偏航融合，以在静止在地面时学习偏航陀螺仪偏置，
	// 并防止失控的偏航方差增长
	// 同时在整平精确对齐步骤中融合零航向创新，以保持偏航方差较低
	if (_params.mag_fusion_type >= MAG_FUSE_TYPE_NONE
	    || _control_status.flags.mag_fault
	    || !_control_status.flags.tilt_align) {

		stopMagFusion();

		if (noOtherYawAidingThanMag())
		{
			// TODO: 设置 _is_yaw_fusion_inhibited 为 true 是为了告诉
			// fuseHeading 执行“零创新偏航融合”
			// 我们应该重构它，避免在这里使用这个标志
			_is_yaw_fusion_inhibited = true;
			fuseHeading();
			_is_yaw_fusion_inhibited = false;
		}
		return;
	}

	_mag_yaw_reset_req |= otherHeadingSourcesHaveStopped();
	_mag_yaw_reset_req |= !_control_status.flags.yaw_align;
	_mag_yaw_reset_req |= _mag_inhibit_yaw_reset_req;

	if (noOtherYawAidingThanMag() && _mag_data_ready) {
		// 判断应该使用简单磁航向融合（在存在大外部干扰时更有效），
		// 还是更准确的三轴融合
		switch (_params.mag_fusion_type) {
		default:
		/* fallthrough */
		case MAG_FUSE_TYPE_AUTO:
			selectMagAuto();
			break;

		case MAG_FUSE_TYPE_INDOOR:
		/* fallthrough */
		case MAG_FUSE_TYPE_HEADING:
			startMagHdgFusion();
			break;

		case MAG_FUSE_TYPE_3D:
			startMag3DFusion();
			break;
		}

		if (_control_status.flags.in_air) {
			checkHaglYawResetReq();
			runInAirYawReset();
			runVelPosReset(); // TODO: 检查这一点；速度/位置复位只能由 COG 复位（针对固定翼）请求

		} else {
			runOnGroundYawReset();
		}

		if (!_control_status.flags.yaw_align) {
			// 偏航已对齐是继续运行的必要条件
			return;
		}

		checkMagDeclRequired();
		checkMagInhibition();

		runMagAndMagDeclFusions();
	}
}

bool Ekf::noOtherYawAidingThanMag() const
{
	// 如果使用外部视觉数据或 GPS 航向来提供航向，则不使用磁力计融合
	return !_control_status.flags.ev_yaw && !_control_status.flags.gps_yaw;
}

void Ekf::checkHaglYawResetReq()
{
	// 在离开地面上升后，需要重置偏航角，以便恢复因地面磁场干扰造成的偏航问题。
	if (!_control_status.flags.mag_aligned_in_flight) {
		// 检查高度是否已足够高以远离地面磁异常
		// 若还未请求偏航重置，则发起请求。
		static constexpr float mag_anomalies_max_hagl = 1.5f;
		const bool above_mag_anomalies = (getTerrainVPos() - _state.pos(2)) > mag_anomalies_max_hagl;
		_mag_yaw_reset_req = _mag_yaw_reset_req || above_mag_anomalies;
	}
}

void Ekf::runOnGroundYawReset()
{
	if (_mag_yaw_reset_req && isYawResetAuthorized()) {
		const bool has_realigned_yaw = canResetMagHeading()
					       ? resetMagHeading(_mag_lpf.getState())
					       : false;

		if (has_realigned_yaw) {
			_mag_yaw_reset_req = false;
			_control_status.flags.yaw_align = true;

			// 处理特殊情况：当室内运行且下视光流传感器导致假定磁场无效时，
			// 我们无法继续约束偏航漂移或学习偏航偏置。
			if (_mag_inhibit_yaw_reset_req) {
				_mag_inhibit_yaw_reset_req = false;
				// 清零偏航偏置协方差，并将方差设为初始对准不确定度
				P.uncorrelateCovarianceSetVariance<1>(12, sq(_params.switch_on_gyro_bias * FILTER_UPDATE_PERIOD_S));
			}
		}
	}
}

bool Ekf::canResetMagHeading() const
{
	return !isStrongMagneticDisturbance() && (_params.mag_fusion_type != MAG_FUSE_TYPE_NONE);
}

void Ekf::runInAirYawReset()
{
	if (_mag_yaw_reset_req && isYawResetAuthorized()) {
		bool has_realigned_yaw = false;

		if (canRealignYawUsingGps()) { has_realigned_yaw = realignYawGPS(); }
		else if (canResetMagHeading()) { has_realigned_yaw = resetMagHeading(_mag_lpf.getState()); }

		if (has_realigned_yaw) {
			_mag_yaw_reset_req = false;
			_control_status.flags.yaw_align = true;
			_control_status.flags.mag_aligned_in_flight = true;

			// 处理特殊情况：当室内运行且下视光流传感器导致假定磁场无效时，
			// 我们无法继续约束偏航漂移或学习偏航偏置。
			if (_mag_inhibit_yaw_reset_req) {
				_mag_inhibit_yaw_reset_req = false;
				// 清零偏航偏置协方差，并将方差设为初始对准不确定度
				P.uncorrelateCovarianceSetVariance<1>(12, sq(_params.switch_on_gyro_bias * FILTER_UPDATE_PERIOD_S));
			}
		}

	}
}

void Ekf::runVelPosReset()
{
	if (_velpos_reset_request) {
		resetVelocity();
		resetHorizontalPosition();
		_velpos_reset_request = false;
	}
}

void Ekf::selectMagAuto()
{
	check3DMagFusionSuitability();
	canUse3DMagFusion() ? startMag3DFusion() : startMagHdgFusion();
}

void Ekf::check3DMagFusionSuitability()
{
	checkYawAngleObservability();
	checkMagBiasObservability();

	if (isMagBiasObservable() || isYawAngleObservable()) {
		_time_last_mov_3d_mag_suitable = _imu_sample_delayed.time_us;
	}
}

void Ekf::checkYawAngleObservability()
{
	// 检查水平速度是否已发生足够变化，使偏航角可观测
	// 使用迟滞判断，以避免快速切换
	_yaw_angle_observable = _yaw_angle_observable
				? _accel_lpf_NE.norm() > _params.mag_acc_gate
				: _accel_lpf_NE.norm() > 2.0f * _params.mag_acc_gate;

	_yaw_angle_observable = _yaw_angle_observable
				&& (_control_status.flags.gps || _control_status.flags.ev_pos); // 这里是否还需要加上 ev_vel？
}

void Ekf::checkMagBiasObservability()
{
	// 检查是否存在足够的偏航旋转，以使磁偏置状态可观测
	if (!_mag_bias_observable && (fabsf(_yaw_rate_lpf_ef) > _params.mag_yaw_rate_gate)) {
		// 检测到初始偏航运动
		_mag_bias_observable = true;

	} else if (_mag_bias_observable) {
		// 需要持续的偏航运动达到初始偏航速率阈值的 50%
		const float yaw_dt = 1e-6f * (float)(_imu_sample_delayed.time_us - _time_yaw_started);
		const float min_yaw_change_req =  0.5f * _params.mag_yaw_rate_gate * yaw_dt;
		_mag_bias_observable = fabsf(_yaw_delta_ef) > min_yaw_change_req;
	}

	_yaw_delta_ef = 0.0f;
	_time_yaw_started = _imu_sample_delayed.time_us;
}

bool Ekf::canUse3DMagFusion() const
{
	// 使用 3D 融合要求在空中完成航向对齐，但当航向和磁偏置在超过 2 秒内不能观测时，
	// 不应使用该方法
	return _control_status.flags.mag_aligned_in_flight
	       && ((_imu_sample_delayed.time_us - _time_last_mov_3d_mag_suitable) < (uint64_t)2e6);
}

void Ekf::checkMagDeclRequired()
{
	// 如果我们使用三轴磁力计融合，但没有外部 NE 辅助，
	// 那么必须将偏角作为观测量融合，以防止长期航向漂移
	// 当 GPS 辅助可用时，融合偏角是可选的，但建议这样做，
	// 以防车辆长时间静止时出现问题
	const bool user_selected = (_params.mag_declination_source & MASK_FUSE_DECL);
	const bool not_using_ne_aiding = !_control_status.flags.gps;
	_control_status.flags.mag_dec = (_control_status.flags.mag_3D && (not_using_ne_aiding || user_selected));
}

void Ekf::checkMagInhibition()
{
	_is_yaw_fusion_inhibited = shouldInhibitMag();
	if (!_is_yaw_fusion_inhibited) {
		_mag_use_not_inhibit_us = _imu_sample_delayed.time_us;
	}

	// 如果磁力计使用一直被抑制，则需要进行偏航重置以获得有效航向
	if (uint32_t(_imu_sample_delayed.time_us - _mag_use_not_inhibit_us) > (uint32_t)5e6) {
		_mag_inhibit_yaw_reset_req = true;
	}
}

bool Ekf::shouldInhibitMag() const
{
	// 如果用户选择了针对室内磁场误差的自动保护，则仅在航向相对于真北方向对于导航确实需要时，
	// 才使用磁力计。如果没有 GPS 或其他地球坐标系辅助信息，
	// 则假定处于室内环境，磁力计不应使用。
	// 当检测到强磁场干扰，或用户已明确停止使用磁力计时，也会抑制磁力计融合。
	const bool user_selected = (_params.mag_fusion_type == MAG_FUSE_TYPE_INDOOR);

	const bool heading_not_required_for_navigation = !_control_status.flags.gps
							 && !_control_status.flags.ev_pos
							 && !_control_status.flags.ev_vel;

	return (user_selected && heading_not_required_for_navigation)
	       || isStrongMagneticDisturbance();
}

void Ekf::checkMagFieldStrength()
{
	if (_params.check_mag_strength) {
		_control_status.flags.mag_field_disturbed = _NED_origin_initialised
							    ? !isMeasuredMatchingGpsMagStrength()
							    : !isMeasuredMatchingAverageMagStrength();

	} else {
		_control_status.flags.mag_field_disturbed = false;
	}
}

bool Ekf::isMeasuredMatchingGpsMagStrength() const
{
	constexpr float wmm_gate_size = 0.2f; // +/- 高斯
	return isMeasuredMatchingExpected(_mag_sample_delayed.mag.length(), _mag_strength_gps, wmm_gate_size);
}

bool Ekf::isMeasuredMatchingAverageMagStrength() const
{
	constexpr float average_earth_mag_field_strength = 0.45f; // 高斯
	constexpr float average_earth_mag_gate_size = 0.40f; // +/- 高斯
	return isMeasuredMatchingExpected(_mag_sample_delayed.mag.length(),
					  average_earth_mag_field_strength,
					  average_earth_mag_gate_size);
}

bool Ekf::isMeasuredMatchingExpected(const float measured, const float expected, const float gate)
{
	return (measured >= expected - gate)
		&& (measured <= expected + gate);
}

void Ekf::runMagAndMagDeclFusions()
{
	if (_control_status.flags.mag_3D) {
		run3DMagAndDeclFusions();
	} else if (_control_status.flags.mag_hdg) {
		fuseHeading();
	}
}

void Ekf::run3DMagAndDeclFusions()
{
	if (!_mag_decl_cov_reset) {
		// 在任意磁场协方差重置事件之后，需先修正地球磁场状态的协方差，
		// 以纳入偏角信息，然后再融合磁力计数据，避免前几次观测时地球磁场状态发生快速旋转。
		fuseDeclination(0.02f);
		_mag_decl_cov_reset = true;
		fuseMag();

	} else {
		// 常规顺序是先融合磁力计数据，再以较高不确定度融合偏角，
		// 允许随着时间逐步学习偏角。
		fuseMag();
		if (_control_status.flags.mag_dec) {
			fuseDeclination(0.5f);
		}
	}
}

bool Ekf::otherHeadingSourcesHaveStopped()
{
    // 检测 noOtherYawAidingThanMag() 的上升沿
    bool result = noOtherYawAidingThanMag() && _non_mag_yaw_aiding_running_prev;

    _non_mag_yaw_aiding_running_prev = !noOtherYawAidingThanMag();

    return  result;
}
