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
 * @file estimator_interface.cpp
 * 姿态估计器基类的定义
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth B Purohit <siddharthbharatpurohit@gmail.com>
 */

#include "estimator_interface.h"

#include <ecl.h>
#include <mathlib/mathlib.h>

// 累积 IMU 数据，并按目标速率存入缓冲区
void EstimatorInterface::setIMUData(const imuSample &imu_sample)
{
	// TODO：解决职责放错的问题
	if (!_initialised) {
		_initialised = init(imu_sample.time_us);
	}

	const float dt = math::constrain((imu_sample.time_us - _time_last_imu) / 1e6f, 1.0e-4f, 0.02f);

	_time_last_imu = imu_sample.time_us;

	if (_time_last_imu > 0) {
		_dt_imu_avg = 0.8f * _dt_imu_avg + 0.2f * dt;
	}

	_newest_high_rate_imu_sample = imu_sample;

	// 不要更改 computeVibrationMetric 和 checkIfVehicleAtRest 的顺序
	computeVibrationMetric(imu_sample);
	_control_status.flags.vehicle_at_rest = checkIfVehicleAtRest(dt, imu_sample);

	// IMU 降采样
	_imu_updated = _imu_down_sampler.update(imu_sample);

	// 当新的下采样 IMU 数据可用时，累积并下采样 IMU 数据，然后压入缓冲区
	if (_imu_updated) {

		_imu_buffer.push(_imu_down_sampler.getDownSampledImuAndTriggerReset());

		// 获取缓冲区中最旧的数据
		_imu_sample_delayed = _imu_buffer.get_oldest();

		// 计算保证不会丢失数据所需的最小观测间隔
		// 该情况出现于数据在时间戳落入融合时间窗之前被覆盖时
		_min_obs_interval_us = (imu_sample.time_us - _imu_sample_delayed.time_us) / (_obs_buffer_length - 1);

		// 计算阻力融合所需的平均 IMU 数据
		setDragData(imu_sample);
	}
}

void EstimatorInterface::computeVibrationMetric(const imuSample &imu)
{
	// 计算表征锥形振动幅度的指标
	Vector3f temp = imu.delta_ang % _delta_ang_prev;
	_vibe_metrics(0) = 0.99f * _vibe_metrics(0) + 0.01f * temp.norm();

	// 计算表征高频陀螺仪振动量的指标
	temp = imu.delta_ang - _delta_ang_prev;
	_delta_ang_prev = imu.delta_ang;
	_vibe_metrics(1) = 0.99f * _vibe_metrics(1) + 0.01f * temp.norm();

	// 计算表征高频加速度计振动量的指标
	temp = imu.delta_vel - _delta_vel_prev;
	_delta_vel_prev = imu.delta_vel;
	_vibe_metrics(2) = 0.99f * _vibe_metrics(2) + 0.01f * temp.norm();
}

bool EstimatorInterface::checkIfVehicleAtRest(float dt, const imuSample &imu)
{
	// 检测车辆在地面时是否未运动
	if (!_control_status.flags.in_air) {
		if ((_vibe_metrics(1) * 4.0E4f > _params.is_moving_scaler)
		    || (_vibe_metrics(2) * 2.1E2f > _params.is_moving_scaler)
		    || ((imu.delta_ang.norm() / dt) > 0.05f * _params.is_moving_scaler)) {

			_time_last_move_detect_us = imu.time_us;
		}

		return ((imu.time_us - _time_last_move_detect_us) > (uint64_t)1E6);

	} else {
		_time_last_move_detect_us = imu.time_us;
		return false;
	}
}


void EstimatorInterface::setMagData(const magSample &mag_sample)
{
	if (!_initialised || _mag_buffer_fail) {
		return;
	}

	// 如果之前尚未分配所需缓冲区，则在此处进行分配
	// 若此前分配失败，则不再重试
	if (_mag_buffer.get_length() < _obs_buffer_length) {
		_mag_buffer_fail = !_mag_buffer.allocate(_obs_buffer_length);

		if (_mag_buffer_fail) {
			printBufferAllocationFailed("mag");
			return;
		}
	}

	// 以尽可能高的速率进行下采样
	// 方式是对输入样本取平均
	_mag_sample_count++;
	_mag_data_sum += mag_sample.mag;
	_mag_timestamp_sum += mag_sample.time_us / 1000; // 除以 1000 以避免溢出

	// 限制数据速率，防止数据丢失
	if ((mag_sample.time_us - _time_last_mag) > _min_obs_interval_us) {
		_time_last_mag = mag_sample.time_us;

		magSample mag_sample_new;

		// 使用下采样时间区间中点对应的时间作为样本时间
		mag_sample_new.time_us = 1000 * (_mag_timestamp_sum / _mag_sample_count);
		mag_sample_new.time_us -= _params.mag_delay_ms * 1000;
		mag_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		mag_sample_new.mag = _mag_data_sum / _mag_sample_count;

		_mag_buffer.push(mag_sample_new);

		_mag_sample_count = 0;
		_mag_data_sum.setZero();
		_mag_timestamp_sum = 0;
	}
}

void EstimatorInterface::setGpsData(const gps_message &gps)
{
	if (!_initialised || _gps_buffer_fail) {
		return;
	}

	// 如果之前尚未分配所需缓冲区，则在此处进行分配
	// 若此前分配失败，则不再重试
	if (_gps_buffer.get_length() < _obs_buffer_length) {
		_gps_buffer_fail = !_gps_buffer.allocate(_obs_buffer_length);

		if (_gps_buffer_fail) {
			printBufferAllocationFailed("GPS");
			return;
		}
	}

	// 限制数据速率，防止数据丢失
	const bool need_gps = (_params.fusion_mode & MASK_USE_GPS) || (_params.vdist_sensor_type == VDIST_SENSOR_GPS);

	// TODO：移除与时间无关的检查
	if (((gps.time_usec - _time_last_gps) > _min_obs_interval_us) && need_gps && gps.fix_type > 2) {
		_time_last_gps = gps.time_usec;

		gpsSample gps_sample_new;

		gps_sample_new.time_us = gps.time_usec - _params.gps_delay_ms * 1000;
		gps_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		gps_sample_new.vel = gps.vel_ned;

		_gps_speed_valid = gps.vel_ned_valid;
		gps_sample_new.sacc = gps.sacc;
		gps_sample_new.hacc = gps.eph;
		gps_sample_new.vacc = gps.epv;

		gps_sample_new.hgt = (float)gps.alt * 1e-3f;

		gps_sample_new.yaw = gps.yaw;

		if (ISFINITE(gps.yaw_offset)) {
			_gps_yaw_offset = gps.yaw_offset;

		} else {
			_gps_yaw_offset = 0.0f;
		}

		// 仅当已设置原点的 WGS-84 位置时，才计算相对位置
		if (collect_gps(gps)) {
			float lpos_x = 0.0f;
			float lpos_y = 0.0f;
			map_projection_project(&_pos_ref, (gps.lat / 1.0e7), (gps.lon / 1.0e7), &lpos_x, &lpos_y);
			gps_sample_new.pos(0) = lpos_x;
			gps_sample_new.pos(1) = lpos_y;

		} else {
			gps_sample_new.pos(0) = 0.0f;
			gps_sample_new.pos(1) = 0.0f;
		}

		_gps_buffer.push(gps_sample_new);
	}
}

void EstimatorInterface::setBaroData(const baroSample &baro_sample)
{
	if (!_initialised || _baro_buffer_fail) {
		return;
	}

	// 如果之前尚未分配所需缓冲区，则在此处进行分配
	// 若此前分配失败，则不再重试
	if (_baro_buffer.get_length() < _obs_buffer_length) {
		_baro_buffer_fail = !_baro_buffer.allocate(_obs_buffer_length);

		if (_baro_buffer_fail) {
			printBufferAllocationFailed("baro");
			return;
		}
	}

	// 以尽可能高的速率进行下采样
	// 通过对输入样本取平均来处理气压计数据
	_baro_sample_count++;
	_baro_alt_sum += baro_sample.hgt;
	_baro_timestamp_sum += baro_sample.time_us / 1000; // 除以 1000 以避免溢出

	// 限制数据速率，防止数据丢失
	if ((baro_sample.time_us - _time_last_baro) > _min_obs_interval_us) {
		_time_last_baro = baro_sample.time_us;

		const float baro_alt_avg = _baro_alt_sum / (float)_baro_sample_count;

		baroSample baro_sample_new;
		baro_sample_new.hgt = compensateBaroForDynamicPressure(baro_alt_avg);

		// 使用下采样时间区间中点对应的时间作为样本时间
		baro_sample_new.time_us = 1000 * (_baro_timestamp_sum / _baro_sample_count);
		baro_sample_new.time_us -= _params.baro_delay_ms * 1000;
		baro_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_baro_buffer.push(baro_sample_new);

		_baro_sample_count = 0;
		_baro_alt_sum = 0.0f;
		_baro_timestamp_sum = 0;
	}
}

void EstimatorInterface::setAirspeedData(const airspeedSample &airspeed_sample)
{
	if (!_initialised || _airspeed_buffer_fail) {
		return;
	}

	// 如果之前尚未分配所需缓冲区，则在此处进行分配
	// 若此前分配失败，则不再重试
	if (_airspeed_buffer.get_length() < _obs_buffer_length) {
		_airspeed_buffer_fail = !_airspeed_buffer.allocate(_obs_buffer_length);

		if (_airspeed_buffer_fail) {
			printBufferAllocationFailed("airspeed");
			return;
		}
	}

	// limit data rate to prevent data being lost
	if ((airspeed_sample.time_us - _time_last_airspeed) > _min_obs_interval_us) {
		_time_last_airspeed = airspeed_sample.time_us;

		airspeedSample airspeed_sample_new = airspeed_sample;

		airspeed_sample_new.time_us -= _params.airspeed_delay_ms * 1000;
		airspeed_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_airspeed_buffer.push(airspeed_sample_new);
	}
}

void EstimatorInterface::setRangeData(const rangeSample &range_sample)
{
	if (!_initialised || _range_buffer_fail) {
		return;
	}

	// 如果之前尚未分配所需缓冲区，则在此处进行分配
	// 若此前分配失败，则不再重试
	if (_range_buffer.get_length() < _obs_buffer_length) {
		_range_buffer_fail = !_range_buffer.allocate(_obs_buffer_length);

		if (_range_buffer_fail) {
			printBufferAllocationFailed("range");
			return;
		}
	}

	// limit data rate to prevent data being lost
	if ((range_sample.time_us - _time_last_range) > _min_obs_interval_us) {
		_time_last_range = range_sample.time_us;

		rangeSample range_sample_new = range_sample;
		range_sample_new.time_us -= _params.range_delay_ms * 1000;
		range_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_range_buffer.push(range_sample_new);
	}
}

void EstimatorInterface::setOpticalFlowData(const flowSample &flow)
{
	if (!_initialised || _flow_buffer_fail) {
		return;
	}

	// 如果之前尚未分配所需缓冲区，则在此处进行分配
	// 若此前分配失败，则不再重试
	if (_flow_buffer.get_length() < _imu_buffer_length) {
		_flow_buffer_fail = !_flow_buffer.allocate(_imu_buffer_length);

		if (_flow_buffer_fail) {
			printBufferAllocationFailed("flow");
			return;
		}
	}

	// limit data rate to prevent data being lost
	if ((flow.time_us - _time_last_optflow) > _min_obs_interval_us) {
		_time_last_optflow = flow.time_us;

		flowSample optflow_sample_new = flow;

		optflow_sample_new.time_us -= _params.flow_delay_ms * 1000;
		optflow_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_flow_buffer.push(optflow_sample_new);
	}
}

// set attitude and position data derived from an external vision system
void EstimatorInterface::setExtVisionData(const extVisionSample &evdata)
{
	if (!_initialised || _ev_buffer_fail) {
		return;
	}

	// 如果之前尚未分配所需缓冲区，则在此处进行分配
	// 若此前分配失败，则不再重试
	if (_ext_vision_buffer.get_length() < _obs_buffer_length) {
		_ev_buffer_fail = !_ext_vision_buffer.allocate(_obs_buffer_length);

		if (_ev_buffer_fail) {
			printBufferAllocationFailed("vision");
			return;
		}
	}

	// limit data rate to prevent data being lost
	if ((evdata.time_us - _time_last_ext_vision) > _min_obs_interval_us) {
		_time_last_ext_vision = evdata.time_us;

		extVisionSample ev_sample_new = evdata;
		// calculate the system time-stamp for the mid point of the integration period
		ev_sample_new.time_us -= _params.ev_delay_ms * 1000;
		ev_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_ext_vision_buffer.push(ev_sample_new);
	}
}

void EstimatorInterface::setAuxVelData(const auxVelSample &auxvel_sample)
{
	if (!_initialised || _auxvel_buffer_fail) {
		return;
	}

	// 如果之前尚未分配所需缓冲区，则在此处进行分配
	// 若此前分配失败，则不再重试
	if (_auxvel_buffer.get_length() < _obs_buffer_length) {
		_auxvel_buffer_fail = !_auxvel_buffer.allocate(_obs_buffer_length);

		if (_auxvel_buffer_fail) {
			printBufferAllocationFailed("aux vel");
			return;
		}
	}

	// limit data rate to prevent data being lost
	if ((auxvel_sample.time_us - _time_last_auxvel) > _min_obs_interval_us) {
		_time_last_auxvel = auxvel_sample.time_us;

		auxVelSample auxvel_sample_new = auxvel_sample;

		auxvel_sample_new.time_us -= _params.auxvel_delay_ms * 1000;
		auxvel_sample_new.time_us -= FILTER_UPDATE_PERIOD_MS * 1000 / 2;

		_auxvel_buffer.push(auxvel_sample_new);
	}
}


/* 
在多旋翼无人机上，当无人机向前飞行时，机体会向前倾斜以抵抗空气阻力。
在这个倾斜状态下，机体的 XY 平面（水平面）会产生一个特定的加速度分量。
利用这个物理特性，即使没有空速管（Pitot Tube），EKF 也可以通过加速度计的 XY 轴读数“反推”出当前的风速和空速。 
这就是所谓的“阻力融合（Drag Fusion）”。
*/
void EstimatorInterface::setDragData(const imuSample &imu)
{
	// 当累积了足够样本后，通过累加和求均值来对阻力比力数据进行下采样
	if ((_params.fusion_mode & MASK_USE_DRAG) && !_drag_buffer_fail) {

		// 如果之前尚未分配所需缓冲区，则在此处进行分配
		// 若此前分配失败，则不再重试
		if (_drag_buffer.get_length() < _obs_buffer_length) {
			_drag_buffer_fail = !_drag_buffer.allocate(_obs_buffer_length);

			if (_drag_buffer_fail) {
				printBufferAllocationFailed("drag");
				return;
			}
		}

		_drag_sample_count ++;
		// 注意：加速度是以增量速度的形式累积的
		_drag_down_sampled.accelXY(0) += imu.delta_vel(0);
		_drag_down_sampled.accelXY(1) += imu.delta_vel(1);
		_drag_down_sampled.time_us += imu.time_us;
		_drag_sample_time_dt += imu.delta_vel_dt;

		// 计算拖曳比力数据的下采样比率
		uint8_t min_sample_ratio = (uint8_t) ceilf((float)_imu_buffer_length / _obs_buffer_length);

		if (min_sample_ratio < 5) {
			min_sample_ratio = 5;
		}

		// 计算并存储累计值的均值
		if (_drag_sample_count >= min_sample_ratio) {
			// 注意：将累计的增量速度转换为加速度
			_drag_down_sampled.accelXY(0) /= _drag_sample_time_dt;
			_drag_down_sampled.accelXY(1) /= _drag_sample_time_dt;
			_drag_down_sampled.time_us /= _drag_sample_count;

			// 写入缓冲区
			_drag_buffer.push(_drag_down_sampled);

			// 重置累加器
			_drag_sample_count = 0;
			_drag_down_sampled.accelXY.zero();
			_drag_down_sampled.time_us = 0;
			_drag_sample_time_dt = 0.0f;
		}
	}
}

bool EstimatorInterface::initialise_interface(uint64_t timestamp)
{
	// 确定缓冲区需要处理的最大时间延迟
	// 合理地假设气压计始终被使用，并且其延迟较低
	// 同样合理地假设辅助速度设备的延迟也较低。TODO：只有在使用辅助设备时才检查延迟
	float max_time_delay_ms = math::max(_params.baro_delay_ms, _params.auxvel_delay_ms);

	// 使用空速
	if (_params.arsp_thr > FLT_EPSILON) {
		max_time_delay_ms = math::max(_params.airspeed_delay_ms, max_time_delay_ms);
	}

	// 磁力计模式
	if (_params.mag_fusion_type != MAG_FUSE_TYPE_NONE) {
		max_time_delay_ms = math::max(_params.mag_delay_ms, max_time_delay_ms);
	}

	// 距离辅助或距离高度
	if (_params.range_aid || (_params.vdist_sensor_type == VDIST_SENSOR_RANGE)) {
		max_time_delay_ms = math::max(_params.range_delay_ms, max_time_delay_ms);
	}

	if (_params.fusion_mode & MASK_USE_GPS) {
		max_time_delay_ms = math::max(_params.gps_delay_ms, max_time_delay_ms);
	}

	if (_params.fusion_mode & MASK_USE_OF) {
		max_time_delay_ms = math::max(_params.flow_delay_ms, max_time_delay_ms);
	}

	if (_params.fusion_mode & (MASK_USE_EVPOS | MASK_USE_EVYAW | MASK_USE_EVVEL)) {
		max_time_delay_ms = math::max(_params.ev_delay_ms, max_time_delay_ms);
	}

	// 计算 IMU 缓冲区长度，以容纳最大延迟并预留一定抖动余量
	_imu_buffer_length = ceilf(max_time_delay_ms / FILTER_UPDATE_PERIOD_MS) + 1;

	// 设置观测缓冲区长度，以同时处理观测时间间隔最小值
	// 以及当前时间到 EKF 融合时间的最坏延迟
	// 允许因时序抖动造成 EKF 融合时间窗延迟最多扩大 50%
	const float ekf_delay_ms = max_time_delay_ms * 1.5f;
	_obs_buffer_length = ceilf(ekf_delay_ms / _params.sensor_interval_min_ms);

	// limit to be no longer than the IMU buffer (we can't process data faster than the EKF prediction rate)
	_obs_buffer_length = math::min(_obs_buffer_length, _imu_buffer_length);

	if (!_imu_buffer.allocate(_imu_buffer_length) || !_output_buffer.allocate(_imu_buffer_length) || !_output_vert_buffer.allocate(_imu_buffer_length)) {
		printBufferAllocationFailed("IMU and output");
		return false;
	}

	_imu_sample_delayed.time_us = timestamp;
	_imu_sample_delayed.delta_vel_clipping[0] = false;
	_imu_sample_delayed.delta_vel_clipping[1] = false;
	_imu_sample_delayed.delta_vel_clipping[2] = false;

	_fault_status.value = 0;

	return true;
}

bool EstimatorInterface::isOnlyActiveSourceOfHorizontalAiding(const bool aiding_flag) const
{
	return aiding_flag && !isOtherSourceOfHorizontalAidingThan(aiding_flag);
}

bool EstimatorInterface::isOtherSourceOfHorizontalAidingThan(const bool aiding_flag) const
{
	const int nb_sources = getNumberOfActiveHorizontalAidingSources();
	return aiding_flag ? nb_sources > 1 : nb_sources > 0;
}

int EstimatorInterface::getNumberOfActiveHorizontalAidingSources() const
{
	return int(_control_status.flags.gps)
	       + int(_control_status.flags.opt_flow)
	       + int(_control_status.flags.ev_pos)
	       + int(_control_status.flags.ev_vel)
	       // 组合使用空速和侧滑融合可实现持续的相对风死算，
	       // 因此被视为单一辅助源。
	       + int(_control_status.flags.fuse_aspd && _control_status.flags.fuse_beta);
}

bool EstimatorInterface::isHorizontalAidingActive() const
{
	return getNumberOfActiveHorizontalAidingSources() > 0;
}

void EstimatorInterface::printBufferAllocationFailed(const char *buffer_name)
{
	if (buffer_name) {
		ECL_ERR("%s buffer allocation failed", buffer_name);
	}
}

void EstimatorInterface::print_status()
{
	ECL_INFO("imu buffer: %d (%d Bytes)", _imu_buffer.get_length(), _imu_buffer.get_total_size());
	ECL_INFO("gps buffer: %d (%d Bytes)", _gps_buffer.get_length(), _gps_buffer.get_total_size());
	ECL_INFO("mag buffer: %d (%d Bytes)", _mag_buffer.get_length(), _mag_buffer.get_total_size());
	ECL_INFO("baro buffer: %d (%d Bytes)", _baro_buffer.get_length(), _baro_buffer.get_total_size());
	ECL_INFO("range buffer: %d (%d Bytes)", _range_buffer.get_length(), _range_buffer.get_total_size());
	ECL_INFO("airspeed buffer: %d (%d Bytes)", _airspeed_buffer.get_length(), _airspeed_buffer.get_total_size());
	ECL_INFO("flow buffer: %d (%d Bytes)", _flow_buffer.get_length(), _flow_buffer.get_total_size());
	ECL_INFO("vision buffer: %d (%d Bytes)", _ext_vision_buffer.get_length(), _ext_vision_buffer.get_total_size());
	ECL_INFO("output buffer: %d (%d Bytes)", _output_buffer.get_length(), _output_buffer.get_total_size());
	ECL_INFO("output vert buffer: %d (%d Bytes)", _output_vert_buffer.get_length(), _output_vert_buffer.get_total_size());
	ECL_INFO("drag buffer: %d (%d Bytes)", _drag_buffer.get_length(), _drag_buffer.get_total_size());
}
