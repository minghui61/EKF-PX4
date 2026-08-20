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
 * @file terrain_estimator.cpp
 * 用于融合测距仪测量值以估计地形垂直位置的函数
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#include "ekf.h"
#include <ecl.h>
#include <mathlib/mathlib.h>

bool Ekf::initHagl()
{
    bool initialized = false;

    if (!_control_status.flags.in_air) {
        // 如果在地面上，不要信任测距传感器，而是假设一个离地间隙
        _terrain_vpos = _state.pos(2) + _params.rng_gnd_clearance;
        // 使用离地间隙值作为我们的不确定度
        _terrain_var = sq(_params.rng_gnd_clearance);
        _time_last_fake_hagl_fuse = _time_last_imu;
        initialized = true;

    } else if (shouldUseRangeFinderForHagl()
           && _range_sensor.isDataHealthy()) {
        // 如果我们有最新的测量值，用它来初始化地形估计器
        _terrain_vpos = _state.pos(2) + _range_sensor.getDistBottom();
        // 将状态方差初始化为测量值的方差
        _terrain_var = sq(_params.range_noise);
        // 成功
        initialized = true;

    } else if (shouldUseOpticalFlowForHagl()
           && _flow_for_terrain_data_ready) {
        // 将地形垂直位置初始化为原点，因为这是我们目前最好的猜测
        _terrain_vpos = fmaxf(0.0f,  _state.pos(2));
        _terrain_var = 100.0f;
        initialized = true;

    } else {
        // 没有信息 - 无法初始化
    }

    if (initialized) {
        // 已使用有效数据完成初始化
        _time_last_hagl_fuse = _time_last_imu;
    }

    return initialized;
}

void Ekf::runTerrainEstimator()
{
    // 如果在地面上，则存储本地位置和时间，作为参考值
    if (!_control_status.flags.in_air) {
        _last_on_ground_posD = _state.pos(2);
    }

    // 执行初始化检查，
    // 在地面上持续重置地形估计器
    if (!_terrain_initialised || !_control_status.flags.in_air) {
        _terrain_initialised = initHagl();

    } else {

        // 预测状态方差增长，其中状态是飞行器下方地形的垂直位置

        // 由于飞行器高度估计误差引起的过程噪声
        _terrain_var += sq(_imu_sample_delayed.delta_vel_dt * _params.terrain_p_noise);

        // 由于地形坡度引起的过程噪声
        _terrain_var += sq(_imu_sample_delayed.delta_vel_dt * _params.terrain_gradient)
                * (sq(_state.vel(0)) + sq(_state.vel(1)));

        // 限制方差，防止其变得严重病态
        _terrain_var = math::constrain(_terrain_var, 0.0f, 1e4f);

        // 如果可用则融合测距仪数据
        if (shouldUseRangeFinderForHagl()
            && _range_sensor.isDataHealthy()) {
            fuseHagl();
        }

        if (shouldUseOpticalFlowForHagl()
            && _flow_for_terrain_data_ready) {
            fuseFlowForTerrain();
            _flow_for_terrain_data_ready = false;
        }

        // 将 _terrain_vpos 约束为至少比 _state.pos(2) 大 _params.rng_gnd_clearance
        if (_terrain_vpos - _state.pos(2) < _params.rng_gnd_clearance) {
            _terrain_vpos = _params.rng_gnd_clearance + _state.pos(2);
        }
    }

    updateTerrainValidity();
}

void Ekf::fuseHagl()
{
    // 假设地球为平面，从测距仪获取离地高度测量值
    const float meas_hagl = _range_sensor.getDistBottom();

    // 根据飞行器位置和地形高度预测离地高度 (hagl)
    const float pred_hagl = _terrain_vpos - _state.pos(2);

    // 计算新息 (观测值与预测值之差)
    _hagl_innov = pred_hagl - meas_hagl;

    // 计算观测方差，并加入飞行器自身高度不确定度的方差
    const float obs_variance = fmaxf(P(9,9) * _params.vehicle_variance_scaler, 0.0f)
                 + sq(_params.range_noise)
                 + sq(_params.range_noise_scaler * _range_sensor.getRange());

    // 计算新息方差，限制其以防止融合状态变差
    _hagl_innov_var = fmaxf(_terrain_var + obs_variance, obs_variance);

    // 执行新息一致性检查，仅在通过时融合数据
    const float gate_size = fmaxf(_params.range_innov_gate, 1.0f);
    _hagl_test_ratio = sq(_hagl_innov) / (sq(gate_size) * _hagl_innov_var);

    if (_hagl_test_ratio <= 1.0f) {
        // 计算卡尔曼增益
        const float gain = _terrain_var / _hagl_innov_var;
        // 校正状态
        _terrain_vpos -= gain * _hagl_innov;
        // 校正方差
        _terrain_var = fmaxf(_terrain_var * (1.0f - gain), 0.0f);
        // 记录最后一次成功的融合事件
        _time_last_hagl_fuse = _time_last_imu;
        _innov_check_fail_status.flags.reject_hagl = false;

    } else {
        // 如果我们拒绝测距数据的时间过长，则重置为测量值
        const uint64_t timeout = static_cast<uint64_t>(_params.terrain_timeout * 1e6f);
        if (isTimedOut(_time_last_hagl_fuse, timeout)) {
            _terrain_vpos = _state.pos(2) + meas_hagl;
            _terrain_var = obs_variance;
            _terrain_vpos_reset_counter++;

        } else {
            _innov_check_fail_status.flags.reject_hagl = true;
        }
    }
}

void Ekf::fuseFlowForTerrain()
{
    // 使用已移除机体角速率贡献的光流率来计算光学视线 (LOS) 速率
    // 对用于运动补偿的数据中的陀螺仪偏差误差进行校正
    // 注意使用的符号约定：正的 LOS 速率表示场景绕该轴进行右手 (RH) 旋转。
    const Vector2f opt_flow_rate = _flow_compensated_XY_rad / _flow_sample_delayed.dt + Vector2f(_flow_gyro_bias);

    // 获取最新的估计姿态
    const float q0 = _state.quat_nominal(0);
    const float q1 = _state.quat_nominal(1);
    const float q2 = _state.quat_nominal(2);
    const float q3 = _state.quat_nominal(3);

    // 计算光流观测方差
    const float R_LOS = calcOptFlowMeasVar();

    // 获取从地球坐标系到机体坐标系的旋转矩阵
    const Dcmf earth_to_body = quatToInverseRotMat(_state.quat_nominal);

    // 计算传感器相对于 IMU 的位置
    const Vector3f pos_offset_body = _params.flow_pos_body - _params.imu_pos_body;

    // 计算机体坐标系下传感器相对于 IMU 的速度
    // 注意：_flow_sample_delayed.gyro_xyz 是机体角速度的负值，因此使用负号
    const Vector3f vel_rel_imu_body = Vector3f(-_flow_sample_delayed.gyro_xyz / _flow_sample_delayed.dt) % pos_offset_body;

    // 计算地球坐标系下传感器的速度
    const Vector3f vel_rel_earth = _state.vel + _R_to_earth * vel_rel_imu_body;

    // 旋转到机体坐标系
    const Vector3f vel_body = earth_to_body * vel_rel_earth;

    const float t0 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 将地形约束为允许的最小值并预测离地高度
    _terrain_vpos = fmaxf(_terrain_vpos, _params.rng_gnd_clearance + _state.pos(2));
    const float pred_hagl_inv = 1.f / (_terrain_vpos - _state.pos(2));

    // 计算绕飞行器 x 轴的光流观测矩阵
    const float Hx = vel_body(1) * t0 * pred_hagl_inv * pred_hagl_inv;

    // 约束地形方差为非负值
    _terrain_var = fmaxf(_terrain_var, 0.0f);

    // 计算新息方差
    _flow_innov_var(0) = Hx * Hx * _terrain_var + R_LOS;

    // 计算 x 轴光流测量的卡尔曼增益
    const float Kx = _terrain_var * Hx / _flow_innov_var(0);

    // 计算绕 x 轴的预测光流
    const float pred_flow_x = vel_body(1) * earth_to_body(2, 2) * pred_hagl_inv;

    // 计算光流新息 (x 轴)
    _flow_innov(0) = pred_flow_x - opt_flow_rate(0);

    // 计算地形方差的校正项
    const float KxHxP =  Kx * Hx * _terrain_var;

    // 新息一致性检查
    const float gate_size = fmaxf(_params.flow_innov_gate, 1.0f);
    float flow_test_ratio = sq(_flow_innov(0)) / (sq(gate_size) * _flow_innov_var(0));

    // 如果条件恶劣，则不执行测量更新
    if (flow_test_ratio <= 1.0f) {
        _terrain_vpos += Kx * _flow_innov(0);
        // 防止方差为负
        _terrain_var = fmaxf(_terrain_var - KxHxP, 0.0f);
        _time_last_flow_terrain_fuse = _time_last_imu;
    }

    // 计算绕飞行器 y 轴的光流观测矩阵
    const float Hy = -vel_body(0) * t0 * pred_hagl_inv * pred_hagl_inv;

    // 计算新息方差
    _flow_innov_var(1) = Hy * Hy * _terrain_var + R_LOS;

    // 计算 y 轴光流测量的卡尔曼增益
    const float Ky = _terrain_var * Hy / _flow_innov_var(1);

    // 计算绕 y 轴的预测光流
    const float pred_flow_y = -vel_body(0) * earth_to_body(2, 2) * pred_hagl_inv;

    // 计算光流新息 (y 轴)
    _flow_innov(1) = pred_flow_y - opt_flow_rate(1);

    // 计算地形方差的校正项
    const float KyHyP =  Ky * Hy * _terrain_var;

    // 新息一致性检查
    flow_test_ratio = sq(_flow_innov(1)) / (sq(gate_size) * _flow_innov_var(1));

    if (flow_test_ratio <= 1.0f) {
        _terrain_vpos += Ky * _flow_innov(1);
        // 防止方差为负
        _terrain_var = fmaxf(_terrain_var - KyHyP, 0.0f);
        _time_last_flow_terrain_fuse = _time_last_imu;
    }
}

void Ekf::updateTerrainValidity()
{
    // 在过去 5 秒内我们一直在融合测距仪测量值
    const bool recent_range_fusion = isRecent(_time_last_hagl_fuse, (uint64_t)5e6);

    // 在过去 5 秒内我们一直在融合用于地形估计的光流测量值
    // 只有在主滤波器不融合光流的情况下才会出现这种情况
    const bool recent_flow_for_terrain_fusion = isRecent(_time_last_flow_terrain_fuse, (uint64_t)5e6);

    _hagl_valid = (_terrain_initialised && (recent_range_fusion || recent_flow_for_terrain_fusion));

    _hagl_sensor_status.flags.range_finder = shouldUseRangeFinderForHagl()
                         && recent_range_fusion
                         && (_time_last_fake_hagl_fuse != _time_last_hagl_fuse);

    _hagl_sensor_status.flags.flow = shouldUseOpticalFlowForHagl() && recent_flow_for_terrain_fusion;
}