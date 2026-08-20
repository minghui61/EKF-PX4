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
 * @file ekf.cpp
 * EKF 姿态和位置估计器的核心函数。
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 */

#include "ekf.h"

#include <ecl.h>
#include <mathlib/mathlib.h>

bool Ekf::init(uint64_t timestamp)
{
	bool ret = initialise_interface(timestamp);
	reset();
	return ret;
}

void Ekf::reset()
{
	_state.vel.setZero();
	_state.pos.setZero();
	_state.delta_ang_bias.setZero();
	_state.delta_vel_bias.setZero();
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();
	_state.quat_nominal.setIdentity();

	// TODO：谁负责重置输出缓冲区内容？
	_output_new.vel.setZero();
	_output_new.pos.setZero();
	_output_new.quat_nominal.setIdentity();

	_delta_angle_corr.setZero();

	_imu_updated = false;
	_NED_origin_initialised = false;
	_gps_speed_valid = false;

	_filter_initialised = false;
	_terrain_initialised = false;
	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_dt_ekf_avg = FILTER_UPDATE_PERIOD_S;

	_ang_rate_delayed_raw.zero();

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

	_accel_magnitude_filt = 0.0f;
	_ang_rate_magnitude_filt = 0.0f;
	_prev_dvel_bias_var.zero();

	_gps_alt_ref = 0.0f;

	resetGpsDriftCheckFilters();
}

bool Ekf::update()
{
	bool updated = false;

	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();

		if (!_filter_initialised) {
			return false;
		}
	}

	// 仅当缓冲区中的 IMU 数据已更新时才运行滤波器
	if (_imu_updated) {
		// 对主滤波器执行 状态和协方差预测
		// 在没有任何外部传感器（如 GPS、视觉）介入的情况下，纯靠上一时刻的状态和 IMU 的积分，推演出当前时刻机体的三维姿态、速度和位置。
		predictState();
		
		predictCovariance();

		// 控制观测数据的融合
		controlFusionModes();

		// 运行单独的地形估计滤波器
		runTerrainEstimator();

		updated = true;

		// 运行 EKF-GSF 偏航估计器
		runYawEKFGSF();
	}

	// 输出观测器始终运行
	// 在当前融合时间窗中使用全速率 IMU 数据
	calculateOutputStates(_newest_high_rate_imu_sample);

	return updated;
}

bool Ekf::initialiseFilter()
{
	// 过滤加速度以进行倾斜初始化
	const imuSample &imu_init = _imu_buffer.get_newest();

	// 防止零数据
	if (imu_init.delta_vel_dt < 1e-4f || imu_init.delta_ang_dt < 1e-4f) {
		return false;
	}

	if (_is_first_imu_sample) {
		_accel_lpf.reset(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.reset(imu_init.delta_ang / imu_init.delta_ang_dt);
		_is_first_imu_sample = false;

	} else {
		_accel_lpf.update(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.update(imu_init.delta_ang / imu_init.delta_ang_dt);
	}

	// 对磁力计测量值求和
	if (_mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed)) {
		if (_mag_sample_delayed.time_us != 0) {
			if (_mag_counter == 0) {
				_mag_lpf.reset(_mag_sample_delayed.mag);

			} else {
				_mag_lpf.update(_mag_sample_delayed.mag);
			}

			_mag_counter++;
		}
	}

	// 累积足够的高度测量值，以确保数据质量足够可信
	if (_baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed)) {
		if (_baro_sample_delayed.time_us != 0) {
			if (_baro_counter == 0) {
				_baro_hgt_offset = _baro_sample_delayed.hgt;

			} else {
				_baro_hgt_offset = 0.9f * _baro_hgt_offset + 0.1f * _baro_sample_delayed.hgt;
			}

			_baro_counter++;
		}
	}

	if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
		if (_mag_counter < _obs_buffer_length) {
			// 磁力计样本积累不足
			return false;
		}
	}

	if (_baro_counter < _obs_buffer_length) {
		// 气压计样本积累不足
		return false;
	}

	// 初始时使用气压计高度，待其通过检查后再切换到 GPS / 测距 / EV 高度来源。
	setControlBaroHeight();

	if (!initialiseTilt()) {
		return false;
	}

	// 计算初始磁场和偏航对齐
	// 但不要标记偏航对齐完成，因为它需要在整平阶段结束后重置
	resetMagHeading(_mag_lpf.getState(), false, false);

	// 现在已获得所有状态的起始值，初始化状态协方差矩阵
	initialiseCovariance();

	// 使用测量方差更新偏航角方差
	if (_params.mag_fusion_type <= MAG_FUSE_TYPE_3D) {
		// 使用磁航向调谐参数
		increaseQuatYawErrVariance(sq(fmaxf(_params.mag_heading_noise, 1.0e-2f)));
	}

	// 尝试初始化地形估计器
	_terrain_initialised = initHagl();

	// 重置关键融合超时计数器
	_time_last_hgt_fuse = _time_last_imu;
	_time_last_hor_pos_fuse = _time_last_imu;
	_time_last_delpos_fuse = _time_last_imu;
	_time_last_hor_vel_fuse = _time_last_imu;
	_time_last_hagl_fuse = _time_last_imu;
	_time_last_flow_terrain_fuse = _time_last_imu;
	_time_last_of_fuse = _time_last_imu;

	// 重置输出预测器状态历史，使其与 EKF 初始值一致
	alignOutputFilter();

	return true;
}

bool Ekf::initialiseTilt()
{
	const float accel_norm = _accel_lpf.getState().norm();
	const float gyro_norm = _gyro_lpf.getState().norm();

	if (accel_norm < 0.8f * CONSTANTS_ONE_G ||
	    accel_norm > 1.2f * CONSTANTS_ONE_G ||
	    gyro_norm > math::radians(15.0f)) {
		return false;
	}

	// 假设车辆静止，从增量速度向量获取初始滚转角和俯仰角估计
	const Vector3f gravity_in_body = _accel_lpf.getState().normalized();
	const float pitch = asinf(gravity_in_body(0));
	const float roll = atan2f(-gravity_in_body(1), -gravity_in_body(2));

	_state.quat_nominal = Quatf{Eulerf{roll, pitch, 0.0f}};
	_R_to_earth = Dcmf(_state.quat_nominal);

	return true;
}

void Ekf::predictState()
{
	// 应用 IMU 偏置修正
	Vector3f corrected_delta_ang = _imu_sample_delayed.delta_ang - _state.delta_ang_bias;

	// 扣除地球自转带来的角速率分量
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * _imu_sample_delayed.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// 使用四元数乘法将上一四元数乘以增量四元数
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();

	_R_to_earth = Dcmf(_state.quat_nominal);

	// 计算地球坐标系中的增量速度
	const Vector3f corrected_delta_vel = _imu_sample_delayed.delta_vel - _state.delta_vel_bias;
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// 计算 1 秒时间常数下的滤波后水平加速度
	// 这些值用于其他位置的机动检测
	const float alpha = 1.0f - _imu_sample_delayed.delta_vel_dt;
	_accel_lpf_NE = _accel_lpf_NE * alpha + corrected_delta_vel_ef.xy();

	// 保存上一时刻速度值，以便使用梯形积分
	const Vector3f vel_last = _state.vel;

	// 使用当前姿态计算速度增量
	_state.vel += corrected_delta_vel_ef;

	// 补偿重力加速度
	_state.vel(2) += CONSTANTS_ONE_G * _imu_sample_delayed.delta_vel_dt;

	// 通过速度梯形积分预测位置状态
	_state.pos += (vel_last + _state.vel) * _imu_sample_delayed.delta_vel_dt * 0.5f;

	constrainStates();

	// 计算平均滤波器更新时间
	float input = 0.5f * (_imu_sample_delayed.delta_vel_dt + _imu_sample_delayed.delta_ang_dt);

	// 对输入进行滤波并限制在标称值的 -50% 到 +100% 之间
	input = math::constrain(input, 0.5f * FILTER_UPDATE_PERIOD_S, 2.0f * FILTER_UPDATE_PERIOD_S);
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;

	// 代码中其他位置的一些计算需要原始角速率向量，因此在此处计算以避免重复
	// 防止上一帧时间滑移导致的极小时间步可能在角速率中产生尖峰
	// 由于平均不足
	if (_imu_sample_delayed.delta_ang_dt > 0.25f * FILTER_UPDATE_PERIOD_S) {
		_ang_rate_delayed_raw = _imu_sample_delayed.delta_ang / _imu_sample_delayed.delta_ang_dt;
	}

}

/*
 * Implement a strapdown INS algorithm using the latest IMU data at the current time horizon.
 * Buffer the INS states and calculate the difference with the EKF states at the delayed fusion time horizon.
 * Calculate delta angle, delta velocity and velocity corrections from the differences and apply them at the
 * current time horizon so that the INS states track the EKF states at the delayed fusion time horizon.
 * The inspiration for using a complementary filter to correct for time delays in the EKF
 * is based on the work by A Khosravian:
 * “Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements”
 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
*/
void Ekf::calculateOutputStates(const imuSample &imu)
{
	// 在当前融合时间窗中使用全速率 IMU 数据

	// 修正偏置造成的增量角误差
	const float dt_scale_correction = _dt_imu_avg / _dt_ekf_avg;

	// 应用修正以使增量角能够跟踪 EKF 融合时间窗上的四元数状态
	const Vector3f delta_angle(imu.delta_ang - _state.delta_ang_bias * dt_scale_correction + _delta_angle_corr);

	// 计算绕地球坐标系竖轴的偏航变化
	const float spin_del_ang_D = delta_angle.dot(Vector3f(_R_to_earth_now.row(2)));
	_yaw_delta_ef += spin_del_ang_D;

	// 计算用于磁力计融合模式选择逻辑的滤波后偏航速率
	// 注意：采用固定系数以减少运算，精确时间常数并不重要。
	_yaw_rate_lpf_ef = 0.95f * _yaw_rate_lpf_ef + 0.05f * spin_del_ang_D / imu.delta_ang_dt;

	const Quatf dq(AxisAnglef{delta_angle});

	// 将上一时刻 INS 四元数按增量四元数旋转
	_output_new.time_us = imu.time_us;
	_output_new.quat_nominal = _output_new.quat_nominal * dq;

	// 四元数在修改后必须始终归一化
	_output_new.quat_nominal.normalize();

	// 计算机体到地球坐标系的旋转矩阵
	_R_to_earth_now = Dcmf(_output_new.quat_nominal);

	// 修正偏置造成的增量速度误差
	const Vector3f delta_vel_body{imu.delta_vel - _state.delta_vel_bias * dt_scale_correction};

	// 将增量速度旋转到地球坐标系
	Vector3f delta_vel_earth{_R_to_earth_now * delta_vel_body};

	// 修正由重力引起的测量加速度
	delta_vel_earth(2) += CONSTANTS_ONE_G * imu.delta_vel_dt;

	// 计算地球坐标系中的速度导数
	if (imu.delta_vel_dt > 1e-4f) {
		_vel_deriv = delta_vel_earth * (1.0f / imu.delta_vel_dt);
	}

	// 保存上一时刻速度值，以便使用梯形积分
	const Vector3f vel_last(_output_new.vel);

	// 将测量值与修正项相加，递增 INS 速度状态
	// 对替代修正算法使用的垂直状态执行相同处理
	_output_new.vel += delta_vel_earth;
	_output_vert_new.vert_vel += delta_vel_earth(2);

	// 使用梯形积分计算 INS 位置状态
	// 对替代修正算法使用的垂直状态执行相同处理
	const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (imu.delta_vel_dt * 0.5f);
	_output_new.pos += delta_pos_NED;
	_output_vert_new.vert_vel_integ += delta_pos_NED(2);

	// 累计每次更新的时间
	_output_vert_new.dt += imu.delta_vel_dt;

	// 修正 IMU 偏移造成的速度误差
	if (imu.delta_ang_dt > 1e-4f) {
		// 计算最近一次 IMU 更新中的平均角速率
		const Vector3f ang_rate = imu.delta_ang * (1.0f / imu.delta_ang_dt);

		// 计算 IMU 相对于机体原点的速度
		const Vector3f vel_imu_rel_body = ang_rate % _params.imu_pos_body;

		// 将相对速度旋转到地球坐标系
		_vel_imu_rel_body_ned = _R_to_earth_now * vel_imu_rel_body;
	}

	// 将 INS 状态存入与 IMU 数据缓冲区长度和时间坐标一致的环形缓冲区
	if (_imu_updated) {
		_output_buffer.push(_output_new);
		_output_vert_buffer.push(_output_vert_new);

		// 从环形缓冲区获取最早的 INS 状态数据
		// 这些数据将位于 EKF 融合时间窗上
		// TODO：不能保证数据位于延迟融合时间窗上
		//       我们不应该使用 pop_first_older_than 吗？
		const outputSample &output_delayed = _output_buffer.get_oldest();
		const outputVert &output_vert_delayed = _output_vert_buffer.get_oldest();

		// 计算 EKF 融合时间窗上的 INS 和 EKF 四元数之间的四元数差分
		const Quatf q_error((_state.quat_nominal.inversed() * output_delayed.quat_nominal).normalized());

		// 将四元数差分转换为增量角
		const float scalar = (q_error(0) >= 0.0f) ? -2.f : 2.f;

		const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};

		// 计算用于实现估计器姿态状态紧密跟踪，并
		// 根据时间延迟变化进行调节，以保持约 0.7 的一致阻尼比
		const float time_delay = fmaxf((imu.time_us - _imu_sample_delayed.time_us) * 1e-6f, _dt_imu_avg);
		const float att_gain = 0.5f * _dt_imu_avg / time_delay;

		// 计算增量角修正量
		// 使 INS 跟踪 EKF 四元数
		_delta_angle_corr = delta_ang_error * att_gain;
		_output_tracking_error(0) = delta_ang_error.norm();

		/*
		 * 遍历输出滤波器状态历史，并将修正应用到速度和位置状态。
		 * 由于需要进行四元数运算，此方法对姿态状态来说过于昂贵，
		 * 但因为它消除了“修正循环”中的时间延迟，所以允许使用更高的跟踪增益，
		 * 并相对 EKF 状态减少跟踪误差。
		 */

		// 互补滤波增益
		const float vel_gain = _dt_ekf_avg / math::constrain(_params.vel_Tau, _dt_ekf_avg, 10.0f);
		const float pos_gain = _dt_ekf_avg / math::constrain(_params.pos_Tau, _dt_ekf_avg, 10.0f);

		// 计算竖直速度和位置跟踪误差
		const float vert_vel_err = (_state.vel(2) - output_vert_delayed.vert_vel);
		const float vert_vel_integ_err = (_state.pos(2) - output_vert_delayed.vert_vel_integ);

		// 计算将应用于输出状态历史的速度修正量
		// 使用针对 5% 超调调校的 PD 反馈
		const float vert_vel_correction = vert_vel_integ_err * pos_gain + vert_vel_err * vel_gain * 1.1f;

		applyCorrectionToVerticalOutputBuffer(vert_vel_correction);

		// 计算速度和位置跟踪误差
		const Vector3f vel_err(_state.vel - output_delayed.vel);
		const Vector3f pos_err(_state.pos - output_delayed.pos);

		_output_tracking_error(1) = vel_err.norm();
		_output_tracking_error(2) = pos_err.norm();

		// 计算将应用于输出状态历史的速度修正量
		_vel_err_integ += vel_err;
		const Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

		// 计算将应用于输出状态历史的位置修正量
		_pos_err_integ += pos_err;
		const Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

		applyCorrectionToOutputBuffer(vel_correction, pos_correction);
	}
}

/*
* 计算应用于 vert_vel 的修正量，使 vert_vel_integ 能跟踪 EKF
* 在融合时间窗上的下向位置状态，此算法与 vel 和 pos 状态跟踪所用算法不同。
* 该算法会对 vert_vel 状态历史应用修正，并使用修正后的 vert_vel 历史向前传播 vert_vel_integ。
* 这提供了更接近位置一阶导数的替代垂直速度输出，
* 但相对 EKF 状态会降低跟踪精度。
*/
void Ekf::applyCorrectionToVerticalOutputBuffer(float vert_vel_correction)
{
	// 从最旧状态开始遍历垂直输出滤波器状态历史，并将修正应用到
	// vert_vel 状态，并使用修正后的 vert_vel 向前传播 vert_vel_integ
	uint8_t index = _output_vert_buffer.get_oldest_index();

	const uint8_t size = _output_vert_buffer.get_length();

	for (uint8_t counter = 0; counter < (size - 1); counter++) {
		const uint8_t index_next = (index + 1) % size;
		outputVert &current_state = _output_vert_buffer[index];
		outputVert &next_state = _output_vert_buffer[index_next];

		// 修正速度
		if (counter == 0) {
			current_state.vert_vel += vert_vel_correction;
		}

		next_state.vert_vel += vert_vel_correction;

		// 位置使用修正后的速度和梯形积分器向前传播
		next_state.vert_vel_integ = current_state.vert_vel_integ + (current_state.vert_vel + next_state.vert_vel) * 0.5f * next_state.dt;

		// 推进索引
		index = (index + 1) % size;
	}

	// 将输出状态更新为修正后的值
	_output_vert_new = _output_vert_buffer.get_newest();

	// 将时间增量重置为零，以便下一次累计全速率 IMU 数据
	_output_vert_new.dt = 0.0f;
}

/*
* 计算应用于 vel 和 pos 输出状态历史的修正量。
* vel 和 pos 状态历史会分别修正，以便它们在融合时间窗上跟踪 EKF 状态。
* 此选项提供最准确的 EKF 状态跟踪。
*/
void Ekf::applyCorrectionToOutputBuffer(const Vector3f &vel_correction, const Vector3f &pos_correction)
{
	// 遍历输出滤波器状态历史，并将修正应用到速度和位置状态
	for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
		// 应用恒定速度修正
		_output_buffer[index].vel += vel_correction;

		// 应用恒定位置修正
		_output_buffer[index].pos += pos_correction;
	}

	// 将输出状态更新为修正后的值
	_output_new = _output_buffer.get_newest();
}

/*
* 使用最新 IMU 增量角数据，向前预测上一四元数输出状态。
*/
Quatf Ekf::calculate_quaternion() const
{
	// 使用 EKF 的偏置状态估计修正增量角数据中的偏置误差，并应用
	// 跟踪 EKF 四元数状态所需的修正
	const Vector3f delta_angle{_newest_high_rate_imu_sample.delta_ang - _state.delta_ang_bias * (_dt_imu_avg / _dt_ekf_avg) + _delta_angle_corr};

	// 使用修正后的增量角向量更新四元数
	// 四元数在修改后必须始终归一化
	return Quatf{_output_new.quat_nominal * AxisAnglef{delta_angle}}.unit();
}
