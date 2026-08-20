/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * @file ekf_helper.cpp
 * EKF 辅助函数定义。
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#include "ekf.h"

#include <ecl.h>
#include <mathlib/mathlib.h>
#include <cstdlib>


void Ekf::resetVelocity()
{
	if (_control_status.flags.gps && isTimedOut(_last_gps_fail_us, (uint64_t)_min_gps_health_time_us)) {
		// 仅当融合时间窗中有新的 GPS 数据时才会调用此重置
		resetVelocityToGps();

	} else if (_control_status.flags.opt_flow) {
		resetHorizontalVelocityToOpticalFlow();

	} else if (_control_status.flags.ev_vel) {
		resetVelocityToVision();

	} else {
		resetHorizontalVelocityToZero();
	}
}

void Ekf::resetVelocityToGps()
{
	_information_events.flags.reset_vel_to_gps = true;
	ECL_INFO("reset velocity to GPS");
	resetVelocityTo(_gps_sample_delayed.vel);
	P.uncorrelateCovarianceSetVariance<3>(4, sq(_gps_sample_delayed.sacc));
}

void Ekf::resetHorizontalVelocityToOpticalFlow()
{
	_information_events.flags.reset_vel_to_flow = true;
	ECL_INFO("reset velocity to flow");
	// 将离地高度约束为至少高于最小可能值
	const float heightAboveGndEst = fmaxf((_terrain_vpos - _state.pos(2)), _params.rng_gnd_clearance);

	// 假设地球为平面，计算从焦点到图像中心的绝对距离
	const float range = heightAboveGndEst / _range_sensor.getCosTilt();

	if ((range - _params.rng_gnd_clearance) > 0.3f) {
		// 说明我们拥有可靠的光流测量，
		// 因此根据光流测量计算机体坐标系下的 X/Y 速度
		Vector3f vel_optflow_body;
		vel_optflow_body(0) = - range * _flow_compensated_XY_rad(1) / _flow_sample_delayed.dt;
		vel_optflow_body(1) =   range * _flow_compensated_XY_rad(0) / _flow_sample_delayed.dt;
		vel_optflow_body(2) = 0.0f;

		// 从机体坐标系旋转到地球坐标系
		const Vector3f vel_optflow_earth = _R_to_earth * vel_optflow_body;

		resetHorizontalVelocityTo(Vector2f(vel_optflow_earth));

	} else {
		resetHorizontalVelocityTo(Vector2f{0.f, 0.f});
	}

	// 使用光流噪声方差重置水平速度方差
	P.uncorrelateCovarianceSetVariance<2>(4, sq(range) * calcOptFlowMeasVar());
}

void Ekf::resetVelocityToVision()
{
	_information_events.flags.reset_vel_to_vision = true;
	ECL_INFO("reset to vision velocity");
	resetVelocityTo(getVisionVelocityInEkfFrame());
	P.uncorrelateCovarianceSetVariance<3>(4, getVisionVelocityVarianceInEkfFrame());
}

void Ekf::resetHorizontalVelocityToZero()
{
	_information_events.flags.reset_vel_to_zero = true;
	ECL_INFO("reset velocity to zero");
	// 当回退到无辅助操作模式时使用
	resetHorizontalVelocityTo(Vector2f{0.f, 0.f});
	P.uncorrelateCovarianceSetVariance<2>(4, 25.0f);
}

void Ekf::resetVelocityTo(const Vector3f &new_vel)
{
	resetHorizontalVelocityTo(Vector2f(new_vel));
	resetVerticalVelocityTo(new_vel(2));
}

void Ekf::resetHorizontalVelocityTo(const Vector2f &new_horz_vel)
{
	const Vector2f delta_horz_vel = new_horz_vel - Vector2f(_state.vel);
	_state.vel.xy() = new_horz_vel;

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel.xy() += delta_horz_vel;
	}

	_output_new.vel.xy() += delta_horz_vel;

	_state_reset_status.velNE_change = delta_horz_vel;
	_state_reset_status.velNE_counter++;
}

void Ekf::resetVerticalVelocityTo(float new_vert_vel)
{
	const float delta_vert_vel = new_vert_vel - _state.vel(2);
	_state.vel(2) = new_vert_vel;

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].vel(2) += delta_vert_vel;
		_output_vert_buffer[index].vert_vel += delta_vert_vel;
	}

	_output_new.vel(2) += delta_vert_vel;
	_output_vert_new.vert_vel += delta_vert_vel;

	_state_reset_status.velD_change = delta_vert_vel;
	_state_reset_status.velD_counter++;
}

void Ekf::resetHorizontalPosition()
{
	// 让下一次里程计更新知道不能使用先前的状态值来计算位置变化
	_hpos_prev_available = false;

	if (_control_status.flags.gps) {
		// 仅当在融合时间窗内有新的 GPS 数据时才调用此重置
		resetHorizontalPositionToGps();

	} else if (_control_status.flags.ev_pos) {
		// 仅当在融合时间窗内有新的外部视觉 (EV) 数据时才调用此重置
		resetHorizontalPositionToVision();

	} else if (_control_status.flags.opt_flow) {
		_information_events.flags.reset_pos_to_last_known = true;
		ECL_INFO("reset position to last known position");

		if (!_control_status.flags.in_air) {
			// 我们很可能是首次启动光流 (OF)，因此重置水平位置
			resetHorizontalPositionTo(Vector2f(0.f, 0.f));

		} else {
			resetHorizontalPositionTo(_last_known_posNE);
		}

		// 在此模式下，估计值是相对于初始位置的，因此我们以零误差开始。
		P.uncorrelateCovarianceSetVariance<2>(7, 0.0f);

	} else {
		_information_events.flags.reset_pos_to_last_known = true;
		ECL_INFO("reset position to last known position");
		// 当回退到无辅助操作模式时使用
		resetHorizontalPositionTo(_last_known_posNE);
		P.uncorrelateCovarianceSetVariance<2>(7, sq(_params.pos_noaid_noise));
	}
}

void Ekf::resetHorizontalPositionToGps()
{
	_information_events.flags.reset_pos_to_gps = true;
	ECL_INFO("reset position to GPS");
	resetHorizontalPositionTo(_gps_sample_delayed.pos);
	P.uncorrelateCovarianceSetVariance<2>(7, sq(_gps_sample_delayed.hacc));
}

void Ekf::resetHorizontalPositionToVision()
{
	_information_events.flags.reset_pos_to_vision = true;
	ECL_INFO("reset position to ev position");
	Vector3f _ev_pos = _ev_sample_delayed.pos;

	if (_params.fusion_mode & MASK_ROTATE_EV) {
		_ev_pos = _R_ev_to_ekf * _ev_sample_delayed.pos;
	}

	resetHorizontalPositionTo(Vector2f(_ev_pos));
	P.uncorrelateCovarianceSetVariance<2>(7, _ev_sample_delayed.posVar.slice<2, 1>(0, 0));
}

void Ekf::resetHorizontalPositionTo(const Vector2f &new_horz_pos)
{
	const Vector2f delta_horz_pos{new_horz_pos - Vector2f{_state.pos}};
	_state.pos.xy() = new_horz_pos;

	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		_output_buffer[index].pos.xy() += delta_horz_pos;
	}

	_output_new.pos.xy() += delta_horz_pos;

	_state_reset_status.posNE_change = delta_horz_pos;
	_state_reset_status.posNE_counter++;
}

void Ekf::resetVerticalPositionTo(const float &new_vert_pos)
{
	const float old_vert_pos = _state.pos(2);
	_state.pos(2) = new_vert_pos;

	// 存储要发布的重置量和时间
	_state_reset_status.posD_change = new_vert_pos - old_vert_pos;
	_state_reset_status.posD_counter++;

	// 将高度/高度率的变化应用到我们最新的、已经从输出缓冲区中取出的高度/高度率估计值中
	_output_new.pos(2) += _state_reset_status.posD_change;

	// 将重置量添加到输出观测器的缓冲数据中
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].pos(2) += _state_reset_status.posD_change;
		_output_vert_buffer[i].vert_vel_integ += _state_reset_status.posD_change;
	}

	// 将重置量添加到输出观测器的垂直位置状态中
	_output_vert_new.vert_vel_integ = _state.pos(2);
}

// 使用最近的高度测量值重置高度状态
void Ekf::resetHeight()
{
	// 获取最新的 GPS 数据
	const gpsSample &gps_newest = _gps_buffer.get_newest();

	// 重置垂直位置
	if (_control_status.flags.rng_hgt) {

		// 发生了从任何其他高度源到测距仪的回退
		if(!_control_status_prev.flags.rng_hgt) {

			if (_control_status.flags.in_air && isTerrainEstimateValid()) {
			    _hgt_sensor_offset = _terrain_vpos;
			} else if (_control_status.flags.in_air) {
				 _hgt_sensor_offset = _range_sensor.getDistBottom() + _state.pos(2);
			} else {
				_hgt_sensor_offset = _params.rng_gnd_clearance;
			}

		}

		// 更新状态及相关的方差
		resetVerticalPositionTo(_hgt_sensor_offset - _range_sensor.getDistBottom());

		// 状态方差与观测方差相同
		P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.range_noise));

		// 重置气压计偏移量，如果我们需要将气压计作为备份使用，该偏移量将从气压计读数中减去
		const baroSample &baro_newest = _baro_buffer.get_newest();
		_baro_hgt_offset = baro_newest.hgt + _state.pos(2);

	} else if (_control_status.flags.baro_hgt) {
		// 使用最新的气压计测量值初始化垂直位置
		const baroSample &baro_newest = _baro_buffer.get_newest();

		if (!_baro_hgt_faulty) {
			resetVerticalPositionTo(-baro_newest.hgt + _baro_hgt_offset);

			// 状态方差与观测方差相同
			P.uncorrelateCovarianceSetVariance<1>(9, sq(_params.baro_noise));

		} else {
			// TODO: 重置为最近一次已知基于气压计的估计值
		}

	} else if (_control_status.flags.gps_hgt) {
		// 使用最新的 GPS 测量值初始化垂直位置和速度
		if (!_gps_hgt_intermittent) {
			resetVerticalPositionTo(_hgt_sensor_offset - gps_newest.hgt + _gps_alt_ref);

			// 状态方差与观测方差相同
			P.uncorrelateCovarianceSetVariance<1>(9, sq(gps_newest.vacc));

			// 重置气压计偏移量，如果我们需要将气压计作为备份使用，该偏移量将从气压计读数中减去
			const baroSample &baro_newest = _baro_buffer.get_newest();
			_baro_hgt_offset = baro_newest.hgt + _state.pos(2);

		} else {
			// TODO: 重置为最近一次已知基于 GPS 的估计值
		}

	} else if (_control_status.flags.ev_hgt) {
		// 使用最新的测量值初始化垂直位置
		const extVisionSample &ev_newest = _ext_vision_buffer.get_newest();

		// 如果最新数据与融合时间窗的时间偏移较小，则使用该数据
		if (ev_newest.time_us >= _ev_sample_delayed.time_us) {
			resetVerticalPositionTo(ev_newest.pos(2));

		} else {
			resetVerticalPositionTo(_ev_sample_delayed.pos(2));
		}
	}

	// 重置垂直速度状态
	if (_control_status.flags.gps && !_gps_hgt_intermittent) {
		// 如果我们正在使用 GPS，则使用它来重置垂直速度
		resetVerticalVelocityTo(gps_newest.vel(2));

		// 状态方差与观测方差相同
		P.uncorrelateCovarianceSetVariance<1>(6, sq(1.5f * gps_newest.sacc));

	} else {
		// 我们不知道垂直速度是多少，因此将其设置为零
		resetVerticalVelocityTo(0.0f);

		// 将方差设置得足够大以允许状态快速收敛，但又不会破坏滤波器的稳定性
		P.uncorrelateCovarianceSetVariance<1>(6, 10.0f);
	}
}

// 对齐输出滤波器状态，使其与融合时间窗处的 EKF 状态相匹配
void Ekf::alignOutputFilter()
{
	const outputSample &output_delayed = _output_buffer.get_oldest();

	// 计算在 EKF 融合时间窗处，从 EKF 到输出观测器状态的四元数旋转增量
	Quatf q_delta{_state.quat_nominal * output_delayed.quat_nominal.inversed()};
	q_delta.normalize();

	// 计算在 EKF 融合时间窗处输出和 EKF 之间的速度和位置增量
	const Vector3f vel_delta = _state.vel - output_delayed.vel;
	const Vector3f pos_delta = _state.pos - output_delayed.pos;

	// 遍历输出滤波器状态历史记录并添加增量
	for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
		_output_buffer[i].quat_nominal = q_delta * _output_buffer[i].quat_nominal;
		_output_buffer[i].quat_nominal.normalize();
		_output_buffer[i].vel += vel_delta;
		_output_buffer[i].pos += pos_delta;
	}

	_output_new = _output_buffer.get_newest();
}

// 强制重新对齐偏航角，使其与 GPS 的水平速度向量对齐。
// 仅用于固定翼飞行器在发射或起飞后对齐偏航角。
bool Ekf::realignYawGPS()
{
	const float gpsSpeed = sqrtf(sq(_gps_sample_delayed.vel(0)) + sq(_gps_sample_delayed.vel(1)));

	// 需要至少 5 m/s 的 GPS 水平速度，并且速度误差与速度的比值 < 0.15 才能进行可靠的对齐
	const bool gps_yaw_alignment_possible = (gpsSpeed > 5.0f) && (_gps_sample_delayed.sacc < (0.15f * gpsSpeed));

	if (!gps_yaw_alignment_possible) {
		// 尝试使用磁力计进行正常对齐
		return resetMagHeading(_mag_lpf.getState());
	}

	// 检查水平 GPS 速度新息是否过大
	const bool badVelInnov = (_gps_vel_test_ratio(0) > 1.0f) && _control_status.flags.gps;

	// 计算 GPS 对地航向角 (COG)
	const float gpsCOG = atan2f(_gps_sample_delayed.vel(1), _gps_sample_delayed.vel(0));

	// 计算航向偏航角
	const float ekfCOG = atan2f(_state.vel(1), _state.vel(0));

	// 检查 EKF 和 GPS 对地航向的一致性
	const float courseYawError = wrap_pi(gpsCOG - ekfCOG);

	// 如果角度不一致且水平 GPS 速度新息较大，或之前未进行偏航对齐，我们宣布磁偏航无效
	const bool badYawErr = fabsf(courseYawError) > 0.5f;
	const bool badMagYaw = (badYawErr && badVelInnov);

	if (badMagYaw) {
		_num_bad_flight_yaw_events ++;
	}

	// 如果指南针偏航无效或之前偏航未对齐，则使用 GPS 地面航向校正偏航角
	if (badMagYaw || !_control_status.flags.yaw_align) {
		_warning_events.flags.bad_yaw_using_gps_course = true;
		ECL_WARN("bad yaw, using GPS course");

		// 如果多次发生不良偏航，则宣布磁力计发生故障
		if (_control_status.flags.mag_aligned_in_flight && (_num_bad_flight_yaw_events >= 2)
		    && !_control_status.flags.mag_fault) {
			_warning_events.flags.stopping_mag_use = true;
			ECL_WARN("stopping mag use");
			_control_status.flags.mag_fault = true;
		}

		// 计算新的偏航估计值
		float yaw_new;

		if (!_control_status.flags.mag_aligned_in_flight) {
			// 这是我们的首次飞行对齐，因此我们可以假设最近的速度变化是由于向前方向的起飞或发射引起的，因此惯性和 GPS 地面航向之间的差异是由于偏航误差造成的
			const float current_yaw = getEuler321Yaw(_state.quat_nominal);
			yaw_new = current_yaw + courseYawError;
			_control_status.flags.mag_aligned_in_flight = true;

		} else if (_control_status.flags.wind) {
			// 我们之前已经在飞行中对齐了偏航角并且有风速估计值，因此设置偏航角使得机头与相对风的 GPS 速度向量对齐
			yaw_new = atan2f((_gps_sample_delayed.vel(1) - _state.wind_vel(1)),
					 (_gps_sample_delayed.vel(0) - _state.wind_vel(0)));

		} else {
			// 我们没有风速估计值，因此将偏航角与 GPS 速度向量对齐
			yaw_new = atan2f(_gps_sample_delayed.vel(1), _gps_sample_delayed.vel(0));

		}

		// 使用组合的 EKF 和 GPS 速度方差来粗略估计对齐后的偏航误差
		const float SpdErrorVariance = sq(_gps_sample_delayed.sacc) + P(4, 4) + P(5, 5);
		const float sineYawError = math::constrain(sqrtf(SpdErrorVariance) / gpsSpeed, 0.0f, 1.0f);
		const float yaw_variance_new = sq(asinf(sineYawError));

		// 将更新的偏航角和偏航方差应用于状态和协方差
		resetQuatStateYaw(yaw_new, yaw_variance_new, true);

		// 使用最后的磁力计测量值重置磁场状态
		_state.mag_B.zero();
		_R_to_earth = Dcmf(_state.quat_nominal);
		_state.mag_I = _R_to_earth * _mag_sample_delayed.mag;

		resetMagCov();

		// 记录磁场对齐的开始时间
		_flt_mag_align_start_time = _imu_sample_delayed.time_us;

		// 如果航向错误，我们还需要重置速度和位置状态
		_velpos_reset_request = badMagYaw;

		return true;

	} else {
		// 仅对齐磁状态

		// 计算初始地球磁场状态
		_state.mag_I = _R_to_earth * _mag_sample_delayed.mag;

		resetMagCov();

		// 记录磁场对齐的开始时间
		_flt_mag_align_start_time = _imu_sample_delayed.time_us;

		return true;
	}
}

// 重置航向和磁场状态
bool Ekf::resetMagHeading(const Vector3f &mag_init, bool increase_yaw_var, bool update_buffer)
{
	// 防止在同一帧上执行多于一次的重置
	if (_imu_sample_delayed.time_us == _flt_mag_align_start_time) {
		return true;
	}

	// 计算观测到的偏航角和偏航方差
	float yaw_new;
	float yaw_new_variance = 0.0f;

	if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
		// 使用零偏航角将磁力计测量值旋转到地球坐标系
		const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

		// 投影到水平面上的角度给出了偏航角
		const Vector3f mag_earth_pred = R_to_earth * mag_init;
		yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

		if (increase_yaw_var) {
			yaw_new_variance = sq(fmaxf(_params.mag_heading_noise, 1.0e-2f));
		}

	} else if (_params.mag_fusion_type == MAG_FUSE_TYPE_INDOOR) {
		// 我们在不知道地球坐标系偏航角的情况下临时运行
		return true;

	} else {
		// 没有磁偏航观测值
		return false;
	}

	// 更新四元数状态和相应的协方差
	resetQuatStateYaw(yaw_new, yaw_new_variance, update_buffer);

	// 使用更新的旋转设置地球磁场状态
	_state.mag_I = _R_to_earth * mag_init;

	resetMagCov();

	// 记录磁场对齐事件的时间
	_flt_mag_align_start_time = _imu_sample_delayed.time_us;

	return true;
}

bool Ekf::resetYawToEv()
{
	const float yaw_new = getEuler312Yaw(_ev_sample_delayed.quat);
	const float yaw_new_variance = fmaxf(_ev_sample_delayed.angVar, sq(1.0e-2f));

	resetQuatStateYaw(yaw_new, yaw_new_variance, true);
	_R_ev_to_ekf.setIdentity();

	return true;
}

// 返回以弧度为单位的磁偏角，用于对齐和融合处理
float Ekf::getMagDeclination()
{
	// 设置内部使用的磁偏角来源
	if (_control_status.flags.mag_aligned_in_flight) {
		// 使用与地球磁场状态一致的值
		return atan2f(_state.mag_I(1), _state.mag_I(0));

	} else if (_params.mag_declination_source & MASK_USE_GEO_DECL) {
		// 在 GPS 可用之前使用参数值，然后使用 geo 库返回的值
		if (_NED_origin_initialised || ISFINITE(_mag_declination_gps)) {
			return _mag_declination_gps;

		} else {
			return math::radians(_params.mag_declination_deg);
		}

	} else {
		// 始终使用参数值
		return math::radians(_params.mag_declination_deg);
	}
}

void Ekf::constrainStates()
{
	_state.quat_nominal = matrix::constrain(_state.quat_nominal, -1.0f, 1.0f);
	_state.vel = matrix::constrain(_state.vel, -1000.0f, 1000.0f);
	_state.pos = matrix::constrain(_state.pos, -1.e6f, 1.e6f);

	const float delta_ang_bias_limit = math::radians(20.f) * _dt_ekf_avg;
	_state.delta_ang_bias = matrix::constrain(_state.delta_ang_bias, -delta_ang_bias_limit, delta_ang_bias_limit);

	const float delta_vel_bias_limit = _params.acc_bias_lim * _dt_ekf_avg;
	_state.delta_vel_bias = matrix::constrain(_state.delta_vel_bias, -delta_vel_bias_limit, delta_vel_bias_limit);

	_state.mag_I = matrix::constrain(_state.mag_I, -1.0f, 1.0f);
	_state.mag_B = matrix::constrain(_state.mag_B, -0.5f, 0.5f);
	_state.wind_vel = matrix::constrain(_state.wind_vel, -100.0f, 100.0f);
}

float Ekf::compensateBaroForDynamicPressure(const float baro_alt_uncompensated) const
{
	// 计算静压误差 = 测量压力 - 真实压力
	// 将位置误差灵敏度建模为固定在机体上的椭圆，在正负 X 和 Y 方向上具有不同的比例。用于纠正气压计数据的位置误差
	const matrix::Dcmf R_to_body(_output_new.quat_nominal.inversed());

	// 计算机体坐标系下的空速
	const Vector3f velocity_earth = _output_new.vel - _vel_imu_rel_body_ned;

	const Vector3f wind_velocity_earth(_state.wind_vel(0), _state.wind_vel(1), 0.0f);

	const Vector3f airspeed_earth = velocity_earth - wind_velocity_earth;

	const Vector3f airspeed_body = R_to_body * airspeed_earth;

	const Vector3f K_pstatic_coef(airspeed_body(0) >= 0.0f ? _params.static_pressure_coef_xp :
				      _params.static_pressure_coef_xn,
				      airspeed_body(1) >= 0.0f ? _params.static_pressure_coef_yp : _params.static_pressure_coef_yn,
				      _params.static_pressure_coef_z);

	const Vector3f airspeed_squared = matrix::min(airspeed_body.emult(airspeed_body), sq(_params.max_correction_airspeed));

	const float pstatic_err = 0.5f * _air_density * (airspeed_squared.dot(K_pstatic_coef));

	// 使用压力误差估计值并假设海平面重力来校正气压计测量值
	return baro_alt_uncompensated + pstatic_err / (_air_density * CONSTANTS_ONE_G);
}

// 计算地球旋转向量
Vector3f Ekf::calcEarthRateNED(float lat_rad) const
{
	return Vector3f(CONSTANTS_EARTH_SPIN_RATE * cosf(lat_rad),
			0.0f,
			-CONSTANTS_EARTH_SPIN_RATE * sinf(lat_rad));
}

void Ekf::getGpsVelPosInnov(float hvel[2], float &vvel, float hpos[2],  float &vpos) const
{
	hvel[0] = _gps_vel_innov(0);
	hvel[1] = _gps_vel_innov(1);
	vvel    = _gps_vel_innov(2);
	hpos[0] = _gps_pos_innov(0);
	hpos[1] = _gps_pos_innov(1);
	vpos    = _gps_pos_innov(2);
}

void Ekf::getGpsVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos)  const
{
	hvel[0] = _gps_vel_innov_var(0);
	hvel[1] = _gps_vel_innov_var(1);
	vvel    = _gps_vel_innov_var(2);
	hpos[0] = _gps_pos_innov_var(0);
	hpos[1] = _gps_pos_innov_var(1);
	vpos    = _gps_pos_innov_var(2);
}

void Ekf::getGpsVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const
{
	hvel = _gps_vel_test_ratio(0);
	vvel = _gps_vel_test_ratio(1);
	hpos = _gps_pos_test_ratio(0);
	vpos = _gps_pos_test_ratio(1);
}

void Ekf::getEvVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) const
{
	hvel[0] = _ev_vel_innov(0);
	hvel[1] = _ev_vel_innov(1);
	vvel    = _ev_vel_innov(2);
	hpos[0] = _ev_pos_innov(0);
	hpos[1] = _ev_pos_innov(1);
	vpos    = _ev_pos_innov(2);
}

void Ekf::getEvVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) const
{
	hvel[0] = _ev_vel_innov_var(0);
	hvel[1] = _ev_vel_innov_var(1);
	vvel    = _ev_vel_innov_var(2);
	hpos[0] = _ev_pos_innov_var(0);
	hpos[1] = _ev_pos_innov_var(1);
	vpos    = _ev_pos_innov_var(2);
}

void Ekf::getEvVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const
{
	hvel = _ev_vel_test_ratio(0);
	vvel = _ev_vel_test_ratio(1);
	hpos = _ev_pos_test_ratio(0);
	vpos = _ev_pos_test_ratio(1);
}

void Ekf::getAuxVelInnov(float aux_vel_innov[2]) const
{
	aux_vel_innov[0] = _aux_vel_innov(0);
	aux_vel_innov[1] = _aux_vel_innov(1);
}

void Ekf::getAuxVelInnovVar(float aux_vel_innov_var[2]) const
{
	aux_vel_innov_var[0] = _aux_vel_innov_var(0);
	aux_vel_innov_var[1] = _aux_vel_innov_var(1);
}

// 获取在延迟时间窗处的状态向量
matrix::Vector<float, 24> Ekf::getStateAtFusionHorizonAsVector() const
{
	matrix::Vector<float, 24> state;
	state.slice<4, 1>(0, 0) = _state.quat_nominal;
	state.slice<3, 1>(4, 0) = _state.vel;
	state.slice<3, 1>(7, 0) = _state.pos;
	state.slice<3, 1>(10, 0) = _state.delta_ang_bias;
	state.slice<3, 1>(13, 0) = _state.delta_vel_bias;
	state.slice<3, 1>(16, 0) = _state.mag_I;
	state.slice<3, 1>(19, 0) = _state.mag_B;
	state.slice<2, 1>(22, 0) = _state.wind_vel;
	return state;
}

bool Ekf::getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const
{
	origin_time = _last_gps_origin_time_us;
	latitude    = math::degrees(_pos_ref.lat_rad);
	longitude   = math::degrees(_pos_ref.lon_rad);
	origin_alt  = _gps_alt_ref;
	return _NED_origin_initialised;
}

bool Ekf::setEkfGlobalOrigin(const double latitude, const double longitude, const float altitude)
{
	bool current_pos_available = false;
	double current_lat = static_cast<double>(NAN);
	double current_lon = static_cast<double>(NAN);
	float current_alt  = 0.f;

	// 如果我们已经在进行辅助，则对 EKF 开始导航以来的位置变化进行校正
	if (map_projection_initialized(&_pos_ref) && isHorizontalAidingActive()) {
		map_projection_reproject(&_pos_ref, _state.pos(0), _state.pos(1), &current_lat, &current_lon);
		current_alt = -_state.pos(2) + _gps_alt_ref;
		current_pos_available = true;
	}

	// 将地图投影重新初始化为纬度、经度、高度，并重置位置
	if (map_projection_init_timestamped(&_pos_ref, latitude, longitude, _time_last_imu) == 0) {
		if (current_pos_available) {
			// 重置水平位置
			Vector2f position;
			map_projection_project(&_pos_ref, current_lat, current_lon, &position(0), &position(1));
			resetHorizontalPositionTo(position);

			// 重置高度
			_gps_alt_ref = altitude;
			resetVerticalPositionTo(_gps_alt_ref - current_alt);
		} else {
			// 重置高度
			_gps_alt_ref = altitude;
		}

		return true;
	}

	return false;
}

/*
	第一个参数在以下数组位置返回 GPS 漂移指标：
	0 : 水平位置漂移率 (m/s)
	1 : 垂直位置漂移率 (m/s)
	2 : 滤波后的水平速度 (m/s)
	当 IMU 运动阻碍漂移计算时，第二个参数返回 true
	如果指标已更新且以前未由此函数返回，则该函数返回 true
*/
bool Ekf::get_gps_drift_metrics(float drift[3], bool *blocked)
{
	memcpy(drift, _gps_drift_metrics, 3 * sizeof(float));
	*blocked = !_control_status.flags.vehicle_at_rest;

	if (_gps_drift_updated) {
		_gps_drift_updated = false;
		return true;
	}

	return false;
}

// 获取 EKF WGS-84 位置的 1-sigma 水平和垂直位置不确定度
void Ekf::get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// 报告考虑了原点位置不确定性的绝对精度
	// 如果没有辅助，由于没有可用的估计值，水平位置估计返回 0
	// TODO - 允许垂直位置误差中存在气压计漂移
	float hpos_err = sqrtf(P(7, 7) + P(8, 8) + sq(_gps_origin_eph));

	// 如果我们正在进行航位推算，则使用新息作为水平位置误差的保守替代度量
	// 原因是完全拒绝测量值通常是由航向未对齐或惯性传感误差引起的，在这些情况下使用状态方差进行精度报告过于乐观
	if (_is_dead_reckoning && (_control_status.flags.gps)) {
		hpos_err = math::max(hpos_err, sqrtf(sq(_gps_pos_innov(0)) + sq(_gps_pos_innov(1))));

	} else if (_is_dead_reckoning && (_control_status.flags.ev_pos)) {
		hpos_err = math::max(hpos_err, sqrtf(sq(_ev_pos_innov(0)) + sq(_ev_pos_innov(1))));
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(9, 9) + sq(_gps_origin_epv));
}

// 获取 EKF 局部位置的 1-sigma 水平和垂直位置不确定度
void Ekf::get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const
{
	// TODO - 允许垂直位置误差中存在气压计漂移
	float hpos_err = sqrtf(P(7, 7) + P(8, 8));

	// 如果我们航位推算的时间过长，则使用新息作为水平位置误差的保守替代度量
	// 原因是完全拒绝测量值通常是由航向未对齐或惯性传感误差引起的，在这些情况下使用状态方差进行精度报告过于乐观
	if (_deadreckon_time_exceeded && _control_status.flags.gps) {
		hpos_err = math::max(hpos_err, sqrtf(sq(_gps_pos_innov(0)) + sq(_gps_pos_innov(1))));
	}

	*ekf_eph = hpos_err;
	*ekf_epv = sqrtf(P(9, 9));
}

// 获取 1-sigma 水平和垂直速度不确定度
void Ekf::get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const
{
	float hvel_err = sqrtf(P(4, 4) + P(5, 5));

	// 如果我们航位推算的时间过长，则使用新息作为水平速度误差的保守替代度量
	// 原因是完全拒绝测量值通常是由航向未对齐或惯性传感误差引起的，在这些情况下使用状态方差进行精度报告过于乐观
	if (_deadreckon_time_exceeded) {
		float vel_err_conservative = 0.0f;

		if (_control_status.flags.opt_flow) {
			float gndclearance = math::max(_params.rng_gnd_clearance, 0.1f);
			vel_err_conservative = math::max((_terrain_vpos - _state.pos(2)), gndclearance) * _flow_innov.norm();
		}

		if (_control_status.flags.gps) {
			vel_err_conservative = math::max(vel_err_conservative, sqrtf(sq(_gps_pos_innov(0)) + sq(_gps_pos_innov(1))));

		} else if (_control_status.flags.ev_pos) {
			vel_err_conservative = math::max(vel_err_conservative, sqrtf(sq(_ev_pos_innov(0)) + sq(_ev_pos_innov(1))));
		}

		if (_control_status.flags.ev_vel) {
			vel_err_conservative = math::max(vel_err_conservative, sqrtf(sq(_ev_vel_innov(0)) + sq(_ev_vel_innov(1))));
		}

		hvel_err = math::max(hvel_err, vel_err_conservative);
	}

	*ekf_evh = hvel_err;
	*ekf_evv = sqrtf(P(6, 6));
}

/*
返回估计器将飞行器保持在传感器限制内所需的以下飞行器控制限制。
vxy_max : 最大的地平相对速度 (m/s)。不需要限制时返回 NaN。
vz_max : 最大的垂直相对速度 (m/s)。不需要限制时返回 NaN。
hagl_min : 最小离地高度 (m)。不需要限制时返回 NaN。
hagl_max : 最大离地高度 (m)。不需要限制时返回 NaN。
*/
void Ekf::get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max) const
{
	// 计算测距仪限制
	const float rangefinder_hagl_min = _range_sensor.getValidMinVal();
	// 允许使用 75% 的测距仪最大量程以容纳角运动
	const float rangefinder_hagl_max = 0.75f * _range_sensor.getValidMaxVal();

	// 计算光流限制
	// 允许对地相对速度使用 50% 的可用光流传感器范围以容纳角运动
	const float flow_vxy_max = fmaxf(0.5f * _flow_max_rate * (_terrain_vpos - _state.pos(2)), 0.0f);
	const float flow_hagl_min = _flow_min_distance;
	const float flow_hagl_max = _flow_max_distance;

	// TODO : 计算视觉里程计限制

	const bool relying_on_rangefinder = _control_status.flags.rng_hgt && !_params.range_aid;

	const bool relying_on_optical_flow = isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow);

	// 默认不需要限制
	*vxy_max = NAN;
	*vz_max = NAN;
	*hagl_min = NAN;
	*hagl_max = NAN;

	// 当使用测距仪作为主要高度源时，保持在测距传感器限制内
	if (relying_on_rangefinder) {
		*vxy_max = NAN;
		*vz_max = NAN;
		*hagl_min = rangefinder_hagl_min;
		*hagl_max = rangefinder_hagl_max;
	}

	// 仅在使用光流时，保持在光流和测距传感器限制内
	if (relying_on_optical_flow) {
		*vxy_max = flow_vxy_max;
		*vz_max = NAN;
		*hagl_min = fmaxf(rangefinder_hagl_min, flow_hagl_min);
		*hagl_max = fminf(rangefinder_hagl_max, flow_hagl_max);
	}
}

void Ekf::resetImuBias()
{
	resetGyroBias();
	resetAccelBias();
}

void Ekf::resetGyroBias()
{
	// 将角度增量和速度增量偏差状态清零
	_state.delta_ang_bias.zero();

	// 将相应的协方差清零，并将方差设置为用于初始对齐的值
	P.uncorrelateCovarianceSetVariance<3>(10, sq(_params.switch_on_gyro_bias * FILTER_UPDATE_PERIOD_S));
}

void Ekf::resetAccelBias()
{
	// 将角度增量和速度增量偏差状态清零
	_state.delta_vel_bias.zero();

	// 将相应的协方差清零，并将方差设置为用于初始对齐的值
	P.uncorrelateCovarianceSetVariance<3>(13, sq(_params.switch_on_accel_bias * FILTER_UPDATE_PERIOD_S));

	// 设置前一帧的值
	_prev_dvel_bias_var = P.slice<3, 3>(13, 13).diag();
}

void Ekf::resetMagBias()
{
	// 将磁力计偏差状态清零
	_state.mag_B.zero();

	// 将相应的协方差清零，并将方差设置为用于初始对齐的值
	P.uncorrelateCovarianceSetVariance<3>(19, sq(_params.mag_noise));

	// 重置任何保存的协方差数据，以便在航向和 3 轴融合之间自动切换时重新使用
	// _saved_mag_bf_variance[0] 是地球 D (下) 轴
	_saved_mag_bf_variance[1] = 0;
	_saved_mag_bf_variance[2] = 0;
	_saved_mag_bf_variance[3] = 0;
}

// 获取 EKF 新息一致性检查状态信息，包括：
// status - 返回包含一致性检查通过/失败状态的整数位掩码
// Innovation Test Ratios - 这些是新息与接收阈值的比率。
// 值 > 1 表示传感器测量值超出了最大可接受水平，已被 EKF 拒绝。
// 当测量类型是向量量（如磁力计、GPS 位置等）时，将返回最大值。
void Ekf::get_innovation_test_status(uint16_t &status, float &mag, float &vel, float &pos, float &hgt, float &tas,
				     float &hagl, float &beta) const
{
	// 返回包含一致性检查通过/失败状态的整数位掩码
	status = _innov_check_fail_status.value;

	// 返回最大的磁力计新息测试比率
	mag = sqrtf(math::max(_yaw_test_ratio, _mag_test_ratio.max()));

	// 返回最大的速度和位置新息测试比率
	vel = NAN;
	pos = NAN;

	if (_control_status.flags.gps) {
		float gps_vel = sqrtf(math::max(_gps_vel_test_ratio(0), _gps_vel_test_ratio(1)));
		vel = math::max(gps_vel, FLT_MIN);

		float gps_pos = sqrtf(_gps_pos_test_ratio(0));
		pos = math::max(gps_pos, FLT_MIN);
	}

	if (_control_status.flags.ev_vel) {
		float ev_vel = sqrtf(math::max(_ev_vel_test_ratio(0), _ev_vel_test_ratio(1)));
		vel = math::max(math::max(vel, ev_vel), FLT_MIN);
	}

	if (_control_status.flags.ev_pos) {
		float ev_pos = sqrtf(_ev_pos_test_ratio(0));
		pos = math::max(math::max(pos, ev_pos), FLT_MIN);
	}

	if (isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow)) {
		float of_vel = sqrtf(_optflow_test_ratio);
		vel = math::max(of_vel, FLT_MIN);
	}

	// 返回垂直位置新息测试比率
	if (_control_status.flags.baro_hgt) {
		hgt = math::max(sqrtf(_baro_hgt_test_ratio(1)), FLT_MIN);

	} else if (_control_status.flags.gps_hgt) {
		hgt = math::max(sqrtf(_gps_pos_test_ratio(1)), FLT_MIN);

	} else if (_control_status.flags.rng_hgt) {
		hgt = math::max(sqrtf(_rng_hgt_test_ratio(1)), FLT_MIN);

	} else if (_control_status.flags.ev_hgt) {
		hgt = math::max(sqrtf(_ev_pos_test_ratio(1)), FLT_MIN);

	} else {
		hgt = NAN;
	}

	// 返回空速融合新息测试比率
	tas = sqrtf(_tas_test_ratio);

	// 返回地形高度新息测试比率
	hagl = sqrtf(_hagl_test_ratio);

	// 返回合成侧滑角新息测试比率
	beta = sqrtf(_beta_test_ratio);
}

// 返回描述哪些状态估计有效的位掩码整数
void Ekf::get_ekf_soln_status(uint16_t *status) const
{
	ekf_solution_status soln_status;
	// TODO: 这足够准确吗？
	soln_status.flags.attitude = _control_status.flags.tilt_align && _control_status.flags.yaw_align && (_fault_status.value == 0);
	soln_status.flags.velocity_horiz = (isHorizontalAidingActive() || (_control_status.flags.fuse_beta && _control_status.flags.fuse_aspd)) && (_fault_status.value == 0);
	soln_status.flags.velocity_vert = (_control_status.flags.baro_hgt || _control_status.flags.ev_hgt || _control_status.flags.gps_hgt || _control_status.flags.rng_hgt) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_rel = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.opt_flow) && (_fault_status.value == 0);
	soln_status.flags.pos_horiz_abs = (_control_status.flags.gps || _control_status.flags.ev_pos) && (_fault_status.value == 0);
	soln_status.flags.pos_vert_abs = soln_status.flags.velocity_vert;
	soln_status.flags.pos_vert_agl = isTerrainEstimateValid();
	soln_status.flags.const_pos_mode = !soln_status.flags.velocity_horiz;
	soln_status.flags.pred_pos_horiz_rel = soln_status.flags.pos_horiz_rel;
	soln_status.flags.pred_pos_horiz_abs = soln_status.flags.pos_horiz_abs;
	const bool gps_vel_innov_bad = (_gps_vel_test_ratio(0) > 1.0f) || (_gps_vel_test_ratio(1) > 1.0f);
	const bool gps_pos_innov_bad = (_gps_pos_test_ratio(0) > 1.0f);
	const bool mag_innov_good = (_mag_test_ratio.max() < 1.0f) && (_yaw_test_ratio < 1.0f);
	soln_status.flags.gps_glitch = (gps_vel_innov_bad || gps_pos_innov_bad) && mag_innov_good;
	soln_status.flags.accel_error = _fault_status.flags.bad_acc_vertical;
	*status = soln_status.value;
}

void Ekf::fuse(const Vector24f &K, float innovation)
{
	_state.quat_nominal -= K.slice<4, 1>(0, 0) * innovation;
	_state.quat_nominal.normalize();
	_state.vel -= K.slice<3, 1>(4, 0) * innovation;
	_state.pos -= K.slice<3, 1>(7, 0) * innovation;
	_state.delta_ang_bias -= K.slice<3, 1>(10, 0) * innovation;
	_state.delta_vel_bias -= K.slice<3, 1>(13, 0) * innovation;
	_state.mag_I -= K.slice<3, 1>(16, 0) * innovation;
	_state.mag_B -= K.slice<3, 1>(19, 0) * innovation;
	_state.wind_vel -= K.slice<2, 1>(22, 0) * innovation;
}

void Ekf::uncorrelateQuatFromOtherStates()
{
	P.slice<_k_num_states - 4, 4>(4, 0) = 0.f;
	P.slice<4, _k_num_states - 4>(0, 4) = 0.f;
}

// 如果我们完全依赖惯性航位推算进行定位，则返回 true
void Ekf::update_deadreckoning_status()
{
	const bool velPosAiding = (_control_status.flags.gps || _control_status.flags.ev_pos || _control_status.flags.ev_vel)
				&& (isRecent(_time_last_hor_pos_fuse, _params.no_aid_timeout_max)
				|| isRecent(_time_last_hor_vel_fuse, _params.no_aid_timeout_max)
				|| isRecent(_time_last_delpos_fuse, _params.no_aid_timeout_max));
	const bool optFlowAiding = _control_status.flags.opt_flow && isRecent(_time_last_of_fuse, _params.no_aid_timeout_max);
	const bool airDataAiding = _control_status.flags.wind &&
				   isRecent(_time_last_arsp_fuse, _params.no_aid_timeout_max) &&
				   isRecent(_time_last_beta_fuse, _params.no_aid_timeout_max);

	_is_wind_dead_reckoning = !velPosAiding && !optFlowAiding && airDataAiding;
	_is_dead_reckoning = !velPosAiding && !optFlowAiding && !airDataAiding;

	if (!_is_dead_reckoning) {
		_time_last_aiding = _time_last_imu - _params.no_aid_timeout_max;
	}

	// 报告我们是否进行了过长时间的航位推算，初始状态为航位推算直到辅助出现
	_deadreckon_time_exceeded = (_time_last_aiding == 0)
				    || isTimedOut(_time_last_aiding, (uint64_t)_params.valid_timeout_max);
}

// 计算等效旋转向量的方差
Vector3f Ekf::calcRotVecVariances()
{
	Vector3f rot_var_vec;
	float q0, q1, q2, q3;

	if (_state.quat_nominal(0) >= 0.0f) {
		q0 = _state.quat_nominal(0);
		q1 = _state.quat_nominal(1);
		q2 = _state.quat_nominal(2);
		q3 = _state.quat_nominal(3);

	} else {
		q0 = -_state.quat_nominal(0);
		q1 = -_state.quat_nominal(1);
		q2 = -_state.quat_nominal(2);
		q3 = -_state.quat_nominal(3);
	}
	float t2 = q0*q0;
	float t3 = acosf(q0);
	float t4 = -t2+1.0f;
	float t5 = t2-1.0f;
	if ((t4 > 1e-9f) && (t5 < -1e-9f)) {
		float t6 = 1.0f/t5;
		float t7 = q1*t6*2.0f;
		float t8 = 1.0f/powf(t4,1.5f);
		float t9 = q0*q1*t3*t8*2.0f;
		float t10 = t7+t9;
		float t11 = 1.0f/sqrtf(t4);
		float t12 = q2*t6*2.0f;
		float t13 = q0*q2*t3*t8*2.0f;
		float t14 = t12+t13;
		float t15 = q3*t6*2.0f;
		float t16 = q0*q3*t3*t8*2.0f;
		float t17 = t15+t16;
		rot_var_vec(0) = t10*(P(0,0)*t10+P(1,0)*t3*t11*2.0f)+t3*t11*(P(0,1)*t10+P(1,1)*t3*t11*2.0f)*2.0f;
		rot_var_vec(1) = t14*(P(0,0)*t14+P(2,0)*t3*t11*2.0f)+t3*t11*(P(0,2)*t14+P(2,2)*t3*t11*2.0f)*2.0f;
		rot_var_vec(2) = t17*(P(0,0)*t17+P(3,0)*t3*t11*2.0f)+t3*t11*(P(0,3)*t17+P(3,3)*t3*t11*2.0f)*2.0f;
	} else {
		rot_var_vec = 4.0f * P.slice<3,3>(1,1).diag();
	}

	return rot_var_vec;
}

// 使用旋转向量方差初始化四元数协方差
// 在四元数状态初始化之前不要调用
void Ekf::initialiseQuatCovariances(Vector3f &rot_vec_var)
{
	// 从四元数计算等效的旋转向量
	float q0,q1,q2,q3;
	if (_state.quat_nominal(0) >= 0.0f) {
		q0 = _state.quat_nominal(0);
		q1 = _state.quat_nominal(1);
		q2 = _state.quat_nominal(2);
		q3 = _state.quat_nominal(3);

	} else {
		q0 = -_state.quat_nominal(0);
		q1 = -_state.quat_nominal(1);
		q2 = -_state.quat_nominal(2);
		q3 = -_state.quat_nominal(3);
	}
	float delta = 2.0f*acosf(q0);
	float scaler = (delta/sinf(delta*0.5f));
	float rotX = scaler*q1;
	float rotY = scaler*q2;
	float rotZ = scaler*q3;

	// 使用 MATLAB 符号工具箱生成的自动代码
	float t2 = rotX*rotX;
	float t4 = rotY*rotY;
	float t5 = rotZ*rotZ;
	float t6 = t2+t4+t5;
	if (t6 > 1e-9f) {
		float t7 = sqrtf(t6);
		float t8 = t7*0.5f;
		float t3 = sinf(t8);
		float t9 = t3*t3;
		float t10 = 1.0f/t6;
		float t11 = 1.0f/sqrtf(t6);
		float t12 = cosf(t8);
		float t13 = 1.0f/powf(t6,1.5f);
		float t14 = t3*t11;
		float t15 = rotX*rotY*t3*t13;
		float t16 = rotX*rotZ*t3*t13;
		float t17 = rotY*rotZ*t3*t13;
		float t18 = t2*t10*t12*0.5f;
		float t27 = t2*t3*t13;
		float t19 = t14+t18-t27;
		float t23 = rotX*rotY*t10*t12*0.5f;
		float t28 = t15-t23;
		float t20 = rotY*rot_vec_var(1)*t3*t11*t28*0.5f;
		float t25 = rotX*rotZ*t10*t12*0.5f;
		float t31 = t16-t25;
		float t21 = rotZ*rot_vec_var(2)*t3*t11*t31*0.5f;
		float t22 = t20+t21-rotX*rot_vec_var(0)*t3*t11*t19*0.5f;
		float t24 = t15-t23;
		float t26 = t16-t25;
		float t29 = t4*t10*t12*0.5f;
		float t34 = t3*t4*t13;
		float t30 = t14+t29-t34;
		float t32 = t5*t10*t12*0.5f;
		float t40 = t3*t5*t13;
		float t33 = t14+t32-t40;
		float t36 = rotY*rotZ*t10*t12*0.5f;
		float t39 = t17-t36;
		float t35 = rotZ*rot_vec_var(2)*t3*t11*t39*0.5f;
		float t37 = t15-t23;
		float t38 = t17-t36;
		float t41 = rot_vec_var(0)*(t15-t23)*(t16-t25);
		float t42 = t41-rot_vec_var(1)*t30*t39-rot_vec_var(2)*t33*t39;
		float t43 = t16-t25;
		float t44 = t17-t36;

		// 将所有四元数协方差清零
		P.uncorrelateCovarianceSetVariance<2>(0, 0.0f);
		P.uncorrelateCovarianceSetVariance<2>(2, 0.0f);


		// 使用 MATLAB 符号工具箱生成的自动代码更新四元数内部协方差
		P(0,0) = rot_vec_var(0)*t2*t9*t10*0.25f+rot_vec_var(1)*t4*t9*t10*0.25f+rot_vec_var(2)*t5*t9*t10*0.25f;
		P(0,1) = t22;
		P(0,2) = t35+rotX*rot_vec_var(0)*t3*t11*(t15-rotX*rotY*t10*t12*0.5f)*0.5f-rotY*rot_vec_var(1)*t3*t11*t30*0.5f;
		P(0,3) = rotX*rot_vec_var(0)*t3*t11*(t16-rotX*rotZ*t10*t12*0.5f)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-rotY*rotZ*t10*t12*0.5f)*0.5f-rotZ*rot_vec_var(2)*t3*t11*t33*0.5f;
		P(1,0) = t22;
		P(1,1) = rot_vec_var(0)*(t19*t19)+rot_vec_var(1)*(t24*t24)+rot_vec_var(2)*(t26*t26);
		P(1,2) = rot_vec_var(2)*(t16-t25)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
		P(1,3) = rot_vec_var(1)*(t15-t23)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
		P(2,0) = t35-rotY*rot_vec_var(1)*t3*t11*t30*0.5f+rotX*rot_vec_var(0)*t3*t11*(t15-t23)*0.5f;
		P(2,1) = rot_vec_var(2)*(t16-t25)*(t17-t36)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
		P(2,2) = rot_vec_var(1)*(t30*t30)+rot_vec_var(0)*(t37*t37)+rot_vec_var(2)*(t38*t38);
		P(2,3) = t42;
		P(3,0) = rotZ*rot_vec_var(2)*t3*t11*t33*(-0.5f)+rotX*rot_vec_var(0)*t3*t11*(t16-t25)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-t36)*0.5f;
		P(3,1) = rot_vec_var(1)*(t15-t23)*(t17-t36)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
		P(3,2) = t42;
		P(3,3) = rot_vec_var(2)*(t33*t33)+rot_vec_var(0)*(t43*t43)+rot_vec_var(1)*(t44*t44);

	} else {
		// 联立方程条件恶劣，因此使用小角度近似
		P.uncorrelateCovarianceSetVariance<1>(0, 0.0f);
		P.uncorrelateCovarianceSetVariance<3>(1, 0.25f * rot_vec_var);
	}
}

void Ekf::setControlBaroHeight()
{
	_control_status.flags.baro_hgt = true;

	_control_status.flags.gps_hgt = false;
	_control_status.flags.rng_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlRangeHeight()
{
	_control_status.flags.rng_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.gps_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlGPSHeight()
{
	_control_status.flags.gps_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.rng_hgt = false;
	_control_status.flags.ev_hgt = false;
}

void Ekf::setControlEVHeight()
{
	_control_status.flags.ev_hgt = true;

	_control_status.flags.baro_hgt = false;
	_control_status.flags.gps_hgt = false;
	_control_status.flags.rng_hgt = false;
}

void Ekf::stopMagFusion()
{
	stopMag3DFusion();
	stopMagHdgFusion();
	clearMagCov();
}

void Ekf::stopMag3DFusion()
{
	// 如果当前正在进行 3 轴融合，则保存协方差数据以备重新使用
	if (_control_status.flags.mag_3D) {
		saveMagCovData();
		_control_status.flags.mag_3D = false;
	}
}

void Ekf::stopMagHdgFusion()
{
	_control_status.flags.mag_hdg = false;
}

void Ekf::startMagHdgFusion()
{
	stopMag3DFusion();
	_control_status.flags.mag_hdg = true;
}

void Ekf::startMag3DFusion()
{
	if (!_control_status.flags.mag_3D) {
		stopMagHdgFusion();
		zeroMagCov();
		loadMagCovData();
		_control_status.flags.mag_3D = true;
	}
}

void Ekf::startBaroHgtFusion()
{
	setControlBaroHeight();

	// 我们不需要设置高度传感器偏移量，
	// 因为我们跟踪了一个独立的 _baro_hgt_offset
	_hgt_sensor_offset = 0.0f;

	// 如果超时则关闭地面效应补偿
	if (_control_status.flags.gnd_effect) {
		if (isTimedOut(_time_last_gnd_effect_on, GNDEFFECT_TIMEOUT)) {

			_control_status.flags.gnd_effect = false;
		}
	}
}

void Ekf::startGpsHgtFusion()
{
	setControlGPSHeight();

	// 我们刚刚切换到使用 GPS 高度，计算高度传感器偏移量，使当前
	// 测量值与我们当前的高度估计值相匹配
	if (_control_status_prev.flags.gps_hgt != _control_status.flags.gps_hgt) {
		_hgt_sensor_offset = _gps_sample_delayed.hgt - _gps_alt_ref + _state.pos(2);
	}
}

void Ekf::updateBaroHgtOffset()
{
	// 如果我们不将气压计作为高度参考，则计算气压计原点与局部 NED 原点之间经过滤波的偏移量
	if (!_control_status.flags.baro_hgt && _baro_data_ready) {
		const float local_time_step = math::constrain(1e-6f * _delta_time_baro_us, 0.0f, 1.0f);

		// 对气压计偏移量应用 10 秒的一阶低通滤波器
		const float offset_rate_correction =  0.1f * (_baro_sample_delayed.hgt + _state.pos(2) -
								_baro_hgt_offset);
		_baro_hgt_offset += local_time_step * math::constrain(offset_rate_correction, -0.1f, 0.1f);
	}
}

float Ekf::getGpsHeightVariance()
{
	// 观测方差 - 接收机定义且受参数限制
	// 使用 1.5 作为 vacc/hacc 的典型比率
	const float lower_limit = fmaxf(1.5f * _params.gps_pos_noise, 0.01f);
	const float upper_limit = fmaxf(1.5f * _params.pos_noaid_noise, lower_limit);
	const float gps_alt_var = sq(math::constrain(_gps_sample_delayed.vacc, lower_limit, upper_limit));
	return gps_alt_var;
}

Vector3f Ekf::getVisionVelocityInEkfFrame() const
{
	Vector3f vel;
	// 针对相对于 IMU 的偏移量校正速度
	const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
	const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;

	// 如果需要，将测量值旋转到正确的地球坐标系
	switch(_ev_sample_delayed.vel_frame) {
		case velocity_frame_t::BODY_FRAME_FRD:
			vel = _R_to_earth * (_ev_sample_delayed.vel - vel_offset_body);
			break;
		case velocity_frame_t::LOCAL_FRAME_FRD:
			const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;
			if (_params.fusion_mode & MASK_ROTATE_EV)
			{
				vel = _R_ev_to_ekf *_ev_sample_delayed.vel - vel_offset_earth;
			} else {
				vel = _ev_sample_delayed.vel - vel_offset_earth;
			}
			break;
	}

	return vel;
}

Vector3f Ekf::getVisionVelocityVarianceInEkfFrame() const
{
	Matrix3f ev_vel_cov = _ev_sample_delayed.velCov;

	// 如果需要，将测量值旋转到正确的地球坐标系
	switch(_ev_sample_delayed.vel_frame) {
		case velocity_frame_t::BODY_FRAME_FRD:
			ev_vel_cov = _R_to_earth * ev_vel_cov * _R_to_earth.transpose();
			break;

		case velocity_frame_t::LOCAL_FRAME_FRD:
			if(_params.fusion_mode & MASK_ROTATE_EV)
			{
				ev_vel_cov = _R_ev_to_ekf * ev_vel_cov * _R_ev_to_ekf.transpose();
			}
			break;
	}

	return ev_vel_cov.diag();
}

// 更新将 EV (外部视觉) 测量值旋转到 EKF 导航坐标系的旋转矩阵
void Ekf::calcExtVisRotMat()
{
	// 计算在 EKF 融合时间窗处从 EV 旋转到 EKF 参考坐标系的四元数增量。
	const Quatf q_error((_state.quat_nominal * _ev_sample_delayed.quat.inversed()).normalized());
	_R_ev_to_ekf = Dcmf(q_error);
}

// 增加四元数的偏航误差方差
// 参数是额外的偏航方差，单位为 rad**2
void Ekf::increaseQuatYawErrVariance(float yaw_variance)
{
	// 推导过程请参阅 DeriveYawResetEquations.m，该文件在 C_code4.txt 文件中生成了代码片段
	// 自动代码已被清理，并删除了乘以零的项，得到以下结果：

	// 中间变量
	float SG[3];
	SG[0] = sq(_state.quat_nominal(0)) - sq(_state.quat_nominal(1)) - sq(_state.quat_nominal(2)) + sq(_state.quat_nominal(3));
	SG[1] = 2*_state.quat_nominal(0)*_state.quat_nominal(2) - 2*_state.quat_nominal(1)*_state.quat_nominal(3);
	SG[2] = 2*_state.quat_nominal(0)*_state.quat_nominal(1) + 2*_state.quat_nominal(2)*_state.quat_nominal(3);

	float SQ[4];
	SQ[0] = 0.5f * ((_state.quat_nominal(1)*SG[0]) - (_state.quat_nominal(0)*SG[2]) + (_state.quat_nominal(3)*SG[1]));
	SQ[1] = 0.5f * ((_state.quat_nominal(0)*SG[1]) - (_state.quat_nominal(2)*SG[0]) + (_state.quat_nominal(3)*SG[2]));
	SQ[2] = 0.5f * ((_state.quat_nominal(3)*SG[0]) - (_state.quat_nominal(1)*SG[1]) + (_state.quat_nominal(2)*SG[2]));
	SQ[3] = 0.5f * ((_state.quat_nominal(0)*SG[0]) + (_state.quat_nominal(1)*SG[2]) + (_state.quat_nominal(2)*SG[1]));

	// 限制偏航方差的增加以防止协方差矩阵条件恶化
	yaw_variance = fminf(yaw_variance, 1.0e-2f);

	// 将额外偏航不确定性的协方差添加到现有协方差中。
	// 这假设额外的偏航误差与现有误差不相关
	P(0,0) += yaw_variance*sq(SQ[2]);
	P(0,1) += yaw_variance*SQ[1]*SQ[2];
	P(1,1) += yaw_variance*sq(SQ[1]);
	P(0,2) += yaw_variance*SQ[0]*SQ[2];
	P(1,2) += yaw_variance*SQ[0]*SQ[1];
	P(2,2) += yaw_variance*sq(SQ[0]);
	P(0,3) -= yaw_variance*SQ[2]*SQ[3];
	P(1,3) -= yaw_variance*SQ[1]*SQ[3];
	P(2,3) -= yaw_variance*SQ[0]*SQ[3];
	P(3,3) += yaw_variance*sq(SQ[3]);
	P(1,0) += yaw_variance*SQ[1]*SQ[2];
	P(2,0) += yaw_variance*SQ[0]*SQ[2];
	P(2,1) += yaw_variance*SQ[0]*SQ[1];
	P(3,0) -= yaw_variance*SQ[2]*SQ[3];
	P(3,1) -= yaw_variance*SQ[1]*SQ[3];
	P(3,2) -= yaw_variance*SQ[0]*SQ[3];
}

// 保存协方差数据，以便在航向和 3 轴融合之间自动切换时重新使用
void Ekf::saveMagCovData()
{
	// 保存地球 D (下) 轴和机体 XYZ 轴磁场的方差
	for (uint8_t index = 0; index <= 3; index ++) {
		_saved_mag_bf_variance[index] = P(index + 18, index + 18);
	}

	// 保存 NE (北东) 轴协方差子矩阵
	_saved_mag_ef_covmat = P.slice<2, 2>(16, 16);
}

void Ekf::loadMagCovData()
{
	// 恢复地球 D (下) 轴和机体 XYZ 轴磁场的方差
	for (uint8_t index = 0; index <= 3; index ++) {
		P(index + 18, index + 18) = _saved_mag_bf_variance[index];
	}

	// 恢复 NE (北东) 轴协方差子矩阵
	P.slice<2, 2>(16, 16) = _saved_mag_ef_covmat;
}

void Ekf::startGpsFusion()
{
	resetHorizontalPositionToGps();

	// 使用光流时，
	// 无需重置速度
	if (!_control_status.flags.opt_flow) {
		resetVelocityToGps();
	}

	_information_events.flags.starting_gps_fusion = true;
	ECL_INFO("starting GPS fusion");
	_control_status.flags.gps = true;
}

void Ekf::stopGpsFusion()
{
	stopGpsPosFusion();
	stopGpsVelFusion();
	stopGpsYawFusion();

	// 我们不再需要知道真北方向
	// 可以再次启动外部视觉 (EV) 偏航
	_inhibit_ev_yaw_use = false;;
}

void Ekf::stopGpsPosFusion()
{
	_control_status.flags.gps = false;
	_control_status.flags.gps_hgt = false;
	_gps_pos_innov.setZero();
	_gps_pos_innov_var.setZero();
	_gps_pos_test_ratio.setZero();
}

void Ekf::stopGpsVelFusion()
{
	_gps_vel_innov.setZero();
	_gps_vel_innov_var.setZero();
	_gps_vel_test_ratio.setZero();
}

void Ekf::startGpsYawFusion()
{
	_control_status.flags.mag_dec = false;
	stopEvYawFusion();
	stopMagHdgFusion();
	stopMag3DFusion();
	_control_status.flags.gps_yaw = true;
}

void Ekf::stopGpsYawFusion()
{
	_control_status.flags.gps_yaw = false;
}

void Ekf::startEvPosFusion()
{
	_control_status.flags.ev_pos = true;
	resetHorizontalPosition();
	_information_events.flags.starting_vision_pos_fusion = true;
	ECL_INFO("starting vision pos fusion");
}

void Ekf::startEvVelFusion()
{
	_control_status.flags.ev_vel = true;
	resetVelocity();
	_information_events.flags.starting_vision_vel_fusion = true;
	ECL_INFO("starting vision vel fusion");
}

void Ekf::startEvYawFusion()
{
	// 开启外部视觉偏航测量融合并禁用所有磁力计融合
	_control_status.flags.ev_yaw = true;
	_control_status.flags.mag_dec = false;

	stopMagHdgFusion();
	stopMag3DFusion();

	_information_events.flags.starting_vision_yaw_fusion = true;
	ECL_INFO("starting vision yaw fusion");
}

void Ekf::stopEvFusion()
{
	stopEvPosFusion();
	stopEvVelFusion();
	stopEvYawFusion();
}

void Ekf::stopEvPosFusion()
{
	_control_status.flags.ev_pos = false;
	_ev_pos_innov.setZero();
	_ev_pos_innov_var.setZero();
	_ev_pos_test_ratio.setZero();
}

void Ekf::stopEvVelFusion()
{
	_control_status.flags.ev_vel = false;
	_ev_vel_innov.setZero();
	_ev_vel_innov_var.setZero();
	_ev_vel_test_ratio.setZero();
}

void Ekf::stopEvYawFusion()
{
	_control_status.flags.ev_yaw = false;
}

void Ekf::stopAuxVelFusion()
{
	_aux_vel_innov.setZero();
	_aux_vel_innov_var.setZero();
	_aux_vel_test_ratio.setZero();
}

void Ekf::stopFlowFusion()
{
	_control_status.flags.opt_flow = false;
	_flow_innov.setZero();
	_flow_innov_var.setZero();
	_optflow_test_ratio = 0.0f;
}

void Ekf::resetQuatStateYaw(float yaw, float yaw_variance, bool update_buffer)
{
	// 保存四元数状态的副本，以便稍后用于计算重置变化量
	const Quatf quat_before_reset = _state.quat_nominal;

	// 使用当前估计值更新从机体到世界坐标系的变换矩阵
	_R_to_earth = Dcmf(_state.quat_nominal);

	// 使用新的偏航角更新旋转矩阵
	_R_to_earth = updateYawInRotMat(yaw, _R_to_earth);

	// 计算四元数改变的量
	const Quatf quat_after_reset(_R_to_earth);
	const Quatf q_error((quat_after_reset * quat_before_reset.inversed()).normalized());

	// 更新四元数状态
	_state.quat_nominal = quat_after_reset;
	uncorrelateQuatFromOtherStates();

	// 记录状态变化
	_state_reset_status.quat_change = q_error;

	// 更新偏航角方差
	if (yaw_variance > FLT_EPSILON) {
		increaseQuatYawErrVariance(yaw_variance);
	}

	// 将重置量添加到输出观测器的缓冲数据中
	if (update_buffer) {
		for (uint8_t i = 0; i < _output_buffer.get_length(); i++) {
			_output_buffer[i].quat_nominal = _state_reset_status.quat_change * _output_buffer[i].quat_nominal;
		}

		// 将姿态四元数的变化应用到我们已经从输出缓冲区取出的最新四元数估计值上
		_output_new.quat_nominal = _state_reset_status.quat_change * _output_new.quat_nominal;

	}

	// 捕获重置事件
	_state_reset_status.quat_counter++;
}

// 将主导航 EKF 的偏航角重置为 EKF-GSF 偏航估计器的值
// 将水平速度和位置重置为默认导航传感器的值
// 如果重置成功则返回 true
bool Ekf::resetYawToEKFGSF()
{
	// 在滤波器开始融合速度数据且偏航角估计收敛之前，不允许使用 EKF-GSF 估计值进行重置
	float new_yaw, new_yaw_variance;

	if (!_yawEstimator.getYawData(&new_yaw, &new_yaw_variance)) {
		return false;
	}

	const bool has_converged = new_yaw_variance < sq(_params.EKFGSF_yaw_err_max);

	if (!has_converged) {
		return false;
	}

	resetQuatStateYaw(new_yaw, new_yaw_variance, true);

	// 将速度和位置状态重置为 GPS - 如果偏航角已固定，则滤波器应开始正常运行
	resetVelocity();
	resetHorizontalPosition();

	// 记录磁场对齐事件，以防止 EKF 在随后的飞行中试图将偏航角重置为磁力计值的可能性
	_flt_mag_align_start_time = _imu_sample_delayed.time_us;
	_control_status.flags.yaw_align = true;

	if (_params.mag_fusion_type == MAG_FUSE_TYPE_NONE) {
		_information_events.flags.yaw_aligned_to_imu_gps = true;
		ECL_INFO("Yaw aligned using IMU and GPS");

	} else {
		// 停止在主 EKF 中使用磁力计，否则其融合可能会拉偏偏航角并导致另一次导航失败
		_control_status.flags.mag_fault = true;
		_warning_events.flags.emergency_yaw_reset_mag_stopped = true;
		ECL_WARN("Emergency yaw reset - mag use stopped");
	}

	return true;
}

bool Ekf::getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF])
{
	return _yawEstimator.getLogData(yaw_composite, yaw_variance, yaw, innov_VN, innov_VE, weight);
}

void Ekf::runYawEKFGSF()
{
	float TAS;

	if (isTimedOut(_airspeed_sample_delayed.time_us, 1000000) && _control_status.flags.fixed_wing) {
		TAS = _params.EKFGSF_tas_default;

	} else {
		TAS = _airspeed_sample_delayed.true_airspeed;
	}

	const Vector3f imu_gyro_bias = getGyroBias();
	_yawEstimator.update(_imu_sample_delayed, _control_status.flags.in_air, TAS, imu_gyro_bias);

	// 对 GPS 速度数据进行基本的合理性检查
	if (_gps_data_ready && _gps_sample_delayed.vacc > FLT_EPSILON &&
	    ISFINITE(_gps_sample_delayed.vel(0)) && ISFINITE(_gps_sample_delayed.vel(1))) {
		_yawEstimator.setVelocity(_gps_sample_delayed.vel.xy(), _gps_sample_delayed.vacc);
	}
}

void Ekf::resetGpsDriftCheckFilters()
{
	_gps_velNE_filt.setZero();
	_gps_pos_deriv_filt.setZero();
}