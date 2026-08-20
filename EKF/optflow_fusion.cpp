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
 * @file vel_pos_fusion.cpp
 * 用于融合 GPS 和气压计测量值的函数
 * 公式使用 EKF/python/ekf_derivation/main.py 生成。
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 *
 */

#include "ekf.h"
#include <ecl.h>
#include <mathlib/mathlib.h>
#include <float.h>
#include "utils.hpp"

void Ekf::fuseOptFlow()
{
	float gndclearance = fmaxf(_params.rng_gnd_clearance, 0.1f);

	// 获取最新估计姿态
	const float q0 = _state.quat_nominal(0);
	const float q1 = _state.quat_nominal(1);
	const float q2 = _state.quat_nominal(2);
	const float q3 = _state.quat_nominal(3);

	// 获取地球坐标系中的最新速度
	const float vn = _state.vel(0);
	const float ve = _state.vel(1);
	const float vd = _state.vel(2);

	// 计算光流观测方差
	const float R_LOS = calcOptFlowMeasVar();

	// 获取从地球到机体的旋转矩阵
	const Dcmf earth_to_body = quatToInverseRotMat(_state.quat_nominal);

	// 计算传感器相对 IMU 的位置
	const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

	// 计算传感器相对 IMU 的速度（机体坐标系）
	// 注意：_flow_sample_delayed.gyro_xyz 是机体角速度的负值，因此使用减号
	const Vector3f vel_rel_imu_body = Vector3f(-_flow_sample_delayed.gyro_xyz / _flow_sample_delayed.dt) % pos_offset_body;

	// 计算传感器在地球坐标系中的速度
	const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

	// 旋转到机体坐标系
	const Vector3f vel_body = earth_to_body * vel_rel_earth;

	// IMU 离地高度
	float heightAboveGndEst = _terrain_vpos - _state.pos(2);

	// 计算传感器相对 IMU 在地球坐标系中的位置
	const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

	// 计算光流相机离地高度。由于地球坐标系为 NED
	// 因此地球坐标系中的正偏移会导致离地高度变小。
	heightAboveGndEst -= pos_offset_earth(2);

	// 约束最小离地高度
	heightAboveGndEst = math::max(heightAboveGndEst, gndclearance);

	// 计算从焦点到图像中心的距离
	const float range = heightAboveGndEst / earth_to_body(2, 2); // 视野中帧区域的绝对距离

	// 使用去除机体角速度贡献后的光流速率计算光流 LOS 速率
	// 修正用于运动补偿的数据中的陀螺仪偏置误差
	// 注意使用的符号约定：正的 LOS 速率表示场景围绕该轴按右手规则旋转。
	const Vector2f opt_flow_rate = _flow_compensated_XY_rad / _flow_sample_delayed.dt + Vector2f(_flow_gyro_bias);

	// 根据校正后的光流测量计算机体和局部坐标系中的速度
	// 仅用于记录日志
	_flow_vel_body(0) = -opt_flow_rate(1) * range;
	_flow_vel_body(1) = opt_flow_rate(0) * range;
	_flow_vel_ne = Vector2f(_R_to_earth * Vector3f(_flow_vel_body(0), _flow_vel_body(1), 0.f));

	_flow_innov(0) =  vel_body(1) / range - opt_flow_rate(0); // flow around the X axis
	_flow_innov(1) = -vel_body(0) / range - opt_flow_rate(1); // flow around the Y axis

	// 推导允许存在任意的机体到光流传感器坐标系旋转，
	// 但当前 EKF 不支持这种情况，因此假设传感器坐标系与机体坐标系对齐
	const Dcmf Tbs = matrix::eye<float, 3>();

	// 中间变量表达式
	const float HK0 = -Tbs(1,0)*q2 + Tbs(1,1)*q1 + Tbs(1,2)*q0;
	const float HK1 = Tbs(1,0)*q3 + Tbs(1,1)*q0 - Tbs(1,2)*q1;
	const float HK2 = Tbs(1,0)*q0 - Tbs(1,1)*q3 + Tbs(1,2)*q2;
	const float HK3 = HK0*vd + HK1*ve + HK2*vn;
	const float HK4 = 1.0F/range;
	const float HK5 = 2*HK4;
	const float HK6 = Tbs(1,0)*q1 + Tbs(1,1)*q2 + Tbs(1,2)*q3;
	const float HK7 = -HK0*ve + HK1*vd + HK6*vn;
	const float HK8 = HK0*vn - HK2*vd + HK6*ve;
	const float HK9 = -HK1*vn + HK2*ve + HK6*vd;
	const float HK10 = q0*q2;
	const float HK11 = q1*q3;
	const float HK12 = HK10 + HK11;
	const float HK13 = 2*Tbs(1,2);
	const float HK14 = q0*q3;
	const float HK15 = q1*q2;
	const float HK16 = HK14 - HK15;
	const float HK17 = 2*Tbs(1,1);
	const float HK18 = ecl::powf(q1, 2);
	const float HK19 = ecl::powf(q2, 2);
	const float HK20 = -HK19;
	const float HK21 = ecl::powf(q0, 2);
	const float HK22 = ecl::powf(q3, 2);
	const float HK23 = HK21 - HK22;
	const float HK24 = HK18 + HK20 + HK23;
	const float HK25 = HK12*HK13 - HK16*HK17 + HK24*Tbs(1,0);
	const float HK26 = HK14 + HK15;
	const float HK27 = 2*Tbs(1,0);
	const float HK28 = q0*q1;
	const float HK29 = q2*q3;
	const float HK30 = HK28 - HK29;
	const float HK31 = -HK18;
	const float HK32 = HK19 + HK23 + HK31;
	const float HK33 = -HK13*HK30 + HK26*HK27 + HK32*Tbs(1,1);
	const float HK34 = HK28 + HK29;
	const float HK35 = HK10 - HK11;
	const float HK36 = HK20 + HK21 + HK22 + HK31;
	const float HK37 = HK17*HK34 - HK27*HK35 + HK36*Tbs(1,2);
	const float HK38 = 2*HK3;
	const float HK39 = 2*HK7;
	const float HK40 = 2*HK8;
	const float HK41 = 2*HK9;
	const float HK42 = HK25*P(0,4) + HK33*P(0,5) + HK37*P(0,6) + HK38*P(0,0) + HK39*P(0,1) + HK40*P(0,2) + HK41*P(0,3);
	const float HK43 = ecl::powf(range, -2);
	const float HK44 = HK25*P(4,6) + HK33*P(5,6) + HK37*P(6,6) + HK38*P(0,6) + HK39*P(1,6) + HK40*P(2,6) + HK41*P(3,6);
	const float HK45 = HK25*P(4,5) + HK33*P(5,5) + HK37*P(5,6) + HK38*P(0,5) + HK39*P(1,5) + HK40*P(2,5) + HK41*P(3,5);
	const float HK46 = HK25*P(4,4) + HK33*P(4,5) + HK37*P(4,6) + HK38*P(0,4) + HK39*P(1,4) + HK40*P(2,4) + HK41*P(3,4);
	const float HK47 = HK25*P(2,4) + HK33*P(2,5) + HK37*P(2,6) + HK38*P(0,2) + HK39*P(1,2) + HK40*P(2,2) + HK41*P(2,3);
	const float HK48 = HK25*P(3,4) + HK33*P(3,5) + HK37*P(3,6) + HK38*P(0,3) + HK39*P(1,3) + HK40*P(2,3) + HK41*P(3,3);
	const float HK49 = HK25*P(1,4) + HK33*P(1,5) + HK37*P(1,6) + HK38*P(0,1) + HK39*P(1,1) + HK40*P(1,2) + HK41*P(1,3);
	// const float HK50 = HK4/(HK25*HK43*HK46 + HK33*HK43*HK45 + HK37*HK43*HK44 + HK38*HK42*HK43 + HK39*HK43*HK49 + HK40*HK43*HK47 + HK41*HK43*HK48 + R_LOS);

	// 计算 X 轴观测的创新方差，并防止数值病态导致的错误计算
	_flow_innov_var(0) = (HK25*HK43*HK46 + HK33*HK43*HK45 + HK37*HK43*HK44 + HK38*HK42*HK43 + HK39*HK43*HK49 + HK40*HK43*HK47 + HK41*HK43*HK48 + R_LOS);

	if (_flow_innov_var(0) < R_LOS) {
		// 需要重新初始化协方差矩阵并中止本次融合步骤
		initialiseCovariance();
		return;
	}
	const float HK50 = HK4/_flow_innov_var(0);

	const float HK51 = Tbs(0,1)*q1;
	const float HK52 = Tbs(0,2)*q0;
	const float HK53 = Tbs(0,0)*q2;
	const float HK54 = HK51 + HK52 - HK53;
	const float HK55 = Tbs(0,0)*q3;
	const float HK56 = Tbs(0,1)*q0;
	const float HK57 = Tbs(0,2)*q1;
	const float HK58 = HK55 + HK56 - HK57;
	const float HK59 = Tbs(0,0)*q0;
	const float HK60 = Tbs(0,2)*q2;
	const float HK61 = Tbs(0,1)*q3;
	const float HK62 = HK59 + HK60 - HK61;
	const float HK63 = HK54*vd + HK58*ve + HK62*vn;
	const float HK64 = Tbs(0,0)*q1 + Tbs(0,1)*q2 + Tbs(0,2)*q3;
	const float HK65 = HK58*vd + HK64*vn;
	const float HK66 = -HK54*ve + HK65;
	const float HK67 = HK54*vn + HK64*ve;
	const float HK68 = -HK62*vd + HK67;
	const float HK69 = HK62*ve + HK64*vd;
	const float HK70 = -HK58*vn + HK69;
	const float HK71 = 2*Tbs(0,1);
	const float HK72 = 2*Tbs(0,2);
	const float HK73 = HK12*HK72 + HK24*Tbs(0,0);
	const float HK74 = -HK16*HK71 + HK73;
	const float HK75 = 2*Tbs(0,0);
	const float HK76 = HK26*HK75 + HK32*Tbs(0,1);
	const float HK77 = -HK30*HK72 + HK76;
	const float HK78 = HK34*HK71 + HK36*Tbs(0,2);
	const float HK79 = -HK35*HK75 + HK78;
	const float HK80 = 2*HK63;
	const float HK81 = 2*HK65 + 2*ve*(-HK51 - HK52 + HK53);
	const float HK82 = 2*HK67 + 2*vd*(-HK59 - HK60 + HK61);
	const float HK83 = 2*HK69 + 2*vn*(-HK55 - HK56 + HK57);
	const float HK84 = HK71*(-HK14 + HK15) + HK73;
	const float HK85 = HK72*(-HK28 + HK29) + HK76;
	const float HK86 = HK75*(-HK10 + HK11) + HK78;
	const float HK87 = HK80*P(0,0) + HK81*P(0,1) + HK82*P(0,2) + HK83*P(0,3) + HK84*P(0,4) + HK85*P(0,5) + HK86*P(0,6);
	const float HK88 = HK80*P(0,6) + HK81*P(1,6) + HK82*P(2,6) + HK83*P(3,6) + HK84*P(4,6) + HK85*P(5,6) + HK86*P(6,6);
	const float HK89 = HK80*P(0,5) + HK81*P(1,5) + HK82*P(2,5) + HK83*P(3,5) + HK84*P(4,5) + HK85*P(5,5) + HK86*P(5,6);
	const float HK90 = HK80*P(0,4) + HK81*P(1,4) + HK82*P(2,4) + HK83*P(3,4) + HK84*P(4,4) + HK85*P(4,5) + HK86*P(4,6);
	const float HK91 = HK80*P(0,2) + HK81*P(1,2) + HK82*P(2,2) + HK83*P(2,3) + HK84*P(2,4) + HK85*P(2,5) + HK86*P(2,6);
	const float HK92 = 2*HK43;
	const float HK93 = HK80*P(0,3) + HK81*P(1,3) + HK82*P(2,3) + HK83*P(3,3) + HK84*P(3,4) + HK85*P(3,5) + HK86*P(3,6);
	const float HK94 = HK80*P(0,1) + HK81*P(1,1) + HK82*P(1,2) + HK83*P(1,3) + HK84*P(1,4) + HK85*P(1,5) + HK86*P(1,6);
	// const float HK95 = HK4/(HK43*HK74*HK90 + HK43*HK77*HK89 + HK43*HK79*HK88 + HK43*HK80*HK87 + HK66*HK92*HK94 + HK68*HK91*HK92 + HK70*HK92*HK93 + R_LOS);

	// 计算 Y 轴观测的创新方差，并防止数值病态导致的错误计算
	_flow_innov_var(1) = (HK43*HK74*HK90 + HK43*HK77*HK89 + HK43*HK79*HK88 + HK43*HK80*HK87 + HK66*HK92*HK94 + HK68*HK91*HK92 + HK70*HK92*HK93 + R_LOS);
	if (_flow_innov_var(1) < R_LOS) {
		// 需要重新初始化协方差矩阵并中止本次融合步骤
		initialiseCovariance();
		return;
	}
	const float HK95 = HK4/_flow_innov_var(1);


	// 运行创新一致性检查并记录结果
	bool flow_fail = false;
	float test_ratio[2];
	test_ratio[0] = sq(_flow_innov(0)) / (sq(math::max(_params.flow_innov_gate, 1.0f)) * _flow_innov_var(0));
	test_ratio[1] = sq(_flow_innov(1)) / (sq(math::max(_params.flow_innov_gate, 1.0f)) * _flow_innov_var(1));
	_optflow_test_ratio = math::max(test_ratio[0],test_ratio[1]);

	for (uint8_t obs_index = 0; obs_index <= 1; obs_index++) {
		if (test_ratio[obs_index] > 1.0f) {
			flow_fail = true;
			_innov_check_fail_status.value |= (1 << (obs_index + 10));

		} else {
			_innov_check_fail_status.value &= ~(1 << (obs_index + 10));

		}
	}

	// 如果任一轴不通过，就中止本次融合
	if (flow_fail) {
		return;

	}

	// 按顺序融合各个观测轴
	SparseVector24f<0,1,2,3,4,5,6> Hfusion; // 光流观测雅可比矩阵
	Vector24f Kfusion; // 光流卡尔曼增益
	for (uint8_t obs_index = 0; obs_index <= 1; obs_index++) {

		// 计算观测雅可比和卡尔曼增益
		if (obs_index == 0) {
			// 观测雅可比 - 轴 0
			Hfusion.at<0>() = HK3*HK5;
			Hfusion.at<1>() = HK5*HK7;
			Hfusion.at<2>() = HK5*HK8;
			Hfusion.at<3>() = HK5*HK9;
			Hfusion.at<4>() = HK25*HK4;
			Hfusion.at<5>() = HK33*HK4;
			Hfusion.at<6>() = HK37*HK4;

			// 卡尔曼增益 - 轴 0
			Kfusion(0) = HK42*HK50;
			Kfusion(1) = HK49*HK50;
			Kfusion(2) = HK47*HK50;
			Kfusion(3) = HK48*HK50;
			Kfusion(4) = HK46*HK50;
			Kfusion(5) = HK45*HK50;
			Kfusion(6) = HK44*HK50;

			for (unsigned row = 7; row <= 23; row++) {
				Kfusion(row) = HK50*(HK25*P(4,row) + HK33*P(5,row) + HK37*P(6,row) + HK38*P(0,row) + HK39*P(1,row) + HK40*P(2,row) + HK41*P(3,row));
			}

		} else {
			// 观测雅可比 - 轴 1
			Hfusion.at<0>() = -HK5*HK63;
			Hfusion.at<1>() = -HK5*HK66;
			Hfusion.at<2>() = -HK5*HK68;
			Hfusion.at<3>() = -HK5*HK70;
			Hfusion.at<4>() = -HK4*HK74;
			Hfusion.at<5>() = -HK4*HK77;
			Hfusion.at<6>() = -HK4*HK79;

			// 卡尔曼增益 - 轴 1
			Kfusion(0) = -HK87*HK95;
			Kfusion(1) = -HK94*HK95;
			Kfusion(2) = -HK91*HK95;
			Kfusion(3) = -HK93*HK95;
			Kfusion(4) = -HK90*HK95;
			Kfusion(5) = -HK89*HK95;
			Kfusion(6) = -HK88*HK95;

			for (unsigned row = 7; row <= 23; row++) {
				Kfusion(row) = -HK95*(HK80*P(0,row) + HK81*P(1,row) + HK82*P(2,row) + HK83*P(3,row) + HK84*P(4,row) + HK85*P(5,row) + HK86*P(6,row));
			}

		}

		const bool is_fused = measurementUpdate(Kfusion, Hfusion, _flow_innov(obs_index));

		if (obs_index == 0) {
			_fault_status.flags.bad_optflow_X = !is_fused;

		} else if (obs_index == 1) {
			_fault_status.flags.bad_optflow_Y = !is_fused;
		}

		if (is_fused) {
			_time_last_of_fuse = _time_last_imu;
		}
	}
}

// 计算光流体轴角速率补偿
// 如果没有可用的偏置校正后的体轴角速率数据，则返回 false
bool Ekf::calcOptFlowBodyRateComp()
{
	// 如果时间间隔过大，则重置累加器
	if (_delta_time_of > 1.0f) {
		_imu_del_ang_of.setZero();
		_delta_time_of = 0.0f;
		return false;
	}

	const bool use_flow_sensor_gyro =  ISFINITE(_flow_sample_delayed.gyro_xyz(0)) && ISFINITE(_flow_sample_delayed.gyro_xyz(1)) && ISFINITE(_flow_sample_delayed.gyro_xyz(2));

	if (use_flow_sensor_gyro) {

		// if accumulation time differences are not excessive and accumulation time is adequate
		// compare the optical flow and and navigation rate data and calculate a bias error
		if ((_delta_time_of > FLT_EPSILON)
		    && (_flow_sample_delayed.dt > FLT_EPSILON)
		    && (fabsf(_delta_time_of - _flow_sample_delayed.dt) < 0.1f)) {

			const Vector3f reference_body_rate(_imu_del_ang_of * (1.0f / _delta_time_of));

			const Vector3f measured_body_rate(_flow_sample_delayed.gyro_xyz * (1.0f / _flow_sample_delayed.dt));

			// calculate the bias estimate using  a combined LPF and spike filter
			_flow_gyro_bias = _flow_gyro_bias * 0.99f + matrix::constrain(measured_body_rate - reference_body_rate, -0.1f, 0.1f) * 0.01f;
		}

	} else {
		// 如果光流传感器陀螺仪数据不可用，则使用 EKF 陀螺仪数据
		// 进一步说明符号规则，见 common.h 中 flowSample 和 imuSample 的定义
		_flow_sample_delayed.gyro_xyz = -_imu_del_ang_of;
		_flow_gyro_bias.zero();
	}

	// 重置累加器
	_imu_del_ang_of.setZero();
	_delta_time_of = 0.0f;
	return true;
}

// 计算光流传感器的测量方差 (rad/sec)^2
float Ekf::calcOptFlowMeasVar()
{
	// 计算观测噪声方差——在光流质量范围内线性缩放噪声
	const float R_LOS_best = fmaxf(_params.flow_noise, 0.05f);
	const float R_LOS_worst = fmaxf(_params.flow_noise_qual_min, 0.05f);

	// 计算一个权重：当光流质量最佳时为 1，最差时为 0
	float weighting = (255.0f - (float)_params.flow_qual_min);

	if (weighting >= 1.0f) {
		weighting = math::constrain(((float)_flow_sample_delayed.quality - (float)_params.flow_qual_min) / weighting, 0.0f,
					    1.0f);

	} else {
		weighting = 0.0f;
	}

	// take the weighted average of the observation noise for the best and wort flow quality
	const float R_LOS = sq(R_LOS_best * weighting + R_LOS_worst * (1.0f - weighting));

	return R_LOS;
}
