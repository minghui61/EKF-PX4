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
 * @file ekf.h
 * EKF 姿态和位置估计器核心功能类。
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 *
 */

#pragma once

#include "estimator_interface.h"

#include "EKFGSF_yaw.h"

class Ekf final : public EstimatorInterface
{
public:
    static constexpr uint8_t _k_num_states{24};     ///< EKF 状态的数量
    typedef matrix::Vector<float, _k_num_states> Vector24f;
    typedef matrix::SquareMatrix<float, _k_num_states> SquareMatrix24f;
    typedef matrix::SquareMatrix<float, 2> Matrix2f;
    typedef matrix::Vector<float, 4> Vector4f;
    template<int ... Idxs>
    using SparseVector24f = matrix::SparseVectorf<24, Idxs...>;

    Ekf() = default;
    virtual ~Ekf() = default;

    // 初始化变量为合理值（也会初始化接口类）
    bool init(uint64_t timestamp) override;

    // 每次将新数据推入滤波器时都应调用
    bool update();

    void getGpsVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
    void getGpsVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
    void getGpsVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const;

    void getEvVelPosInnov(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
    void getEvVelPosInnovVar(float hvel[2], float &vvel, float hpos[2], float &vpos) const;
    void getEvVelPosInnovRatio(float &hvel, float &vvel, float &hpos, float &vpos) const;

    void getBaroHgtInnov(float &baro_hgt_innov) const { baro_hgt_innov = _baro_hgt_innov(2); }
    void getBaroHgtInnovVar(float &baro_hgt_innov_var) const { baro_hgt_innov_var = _baro_hgt_innov_var(2); }
    void getBaroHgtInnovRatio(float &baro_hgt_innov_ratio) const { baro_hgt_innov_ratio = _baro_hgt_test_ratio(1); }

    void getRngHgtInnov(float &rng_hgt_innov) const { rng_hgt_innov = _rng_hgt_innov(2); }
    void getRngHgtInnovVar(float &rng_hgt_innov_var) const { rng_hgt_innov_var = _rng_hgt_innov_var(2); }
    void getRngHgtInnovRatio(float &rng_hgt_innov_ratio) const { rng_hgt_innov_ratio = _rng_hgt_test_ratio(1); }

    void getAuxVelInnov(float aux_vel_innov[2]) const;
    void getAuxVelInnovVar(float aux_vel_innov[2]) const;
    void getAuxVelInnovRatio(float &aux_vel_innov_ratio) const { aux_vel_innov_ratio = _aux_vel_test_ratio(0); }

    void getFlowInnov(float flow_innov[2]) const { _flow_innov.copyTo(flow_innov); }
    void getFlowInnovVar(float flow_innov_var[2]) const { _flow_innov_var.copyTo(flow_innov_var); }
    void getFlowInnovRatio(float &flow_innov_ratio) const { flow_innov_ratio = _optflow_test_ratio; }
    const Vector2f &getFlowVelBody() const { return _flow_vel_body; }
    const Vector2f &getFlowVelNE() const { return _flow_vel_ne; }
    const Vector2f &getFlowCompensated() const { return _flow_compensated_XY_rad; }
    const Vector2f &getFlowUncompensated() const { return _flow_sample_delayed.flow_xy_rad; }
    const Vector3f &getFlowGyro() const { return _flow_sample_delayed.gyro_xyz; }

    void getHeadingInnov(float &heading_innov) const { heading_innov = _heading_innov; }
    void getHeadingInnovVar(float &heading_innov_var) const { heading_innov_var = _heading_innov_var; }

    void getHeadingInnovRatio(float &heading_innov_ratio) const { heading_innov_ratio = _yaw_test_ratio; }
    void getMagInnov(float mag_innov[3]) const { _mag_innov.copyTo(mag_innov); }
    void getMagInnovVar(float mag_innov_var[3]) const { _mag_innov_var.copyTo(mag_innov_var); }
    void getMagInnovRatio(float &mag_innov_ratio) const { mag_innov_ratio = _mag_test_ratio.max(); }

    void getDragInnov(float drag_innov[2]) const { _drag_innov.copyTo(drag_innov); }
    void getDragInnovVar(float drag_innov_var[2]) const { _drag_innov_var.copyTo(drag_innov_var); }
    void getDragInnovRatio(float drag_innov_ratio[2]) const { _drag_test_ratio.copyTo(drag_innov_ratio); }

    void getAirspeedInnov(float &airspeed_innov) const { airspeed_innov = _airspeed_innov; }
    void getAirspeedInnovVar(float &airspeed_innov_var) const { airspeed_innov_var = _airspeed_innov_var; }
    void getAirspeedInnovRatio(float &airspeed_innov_ratio) const { airspeed_innov_ratio = _tas_test_ratio; }

    void getBetaInnov(float &beta_innov) const { beta_innov = _beta_innov; }
    void getBetaInnovVar(float &beta_innov_var) const { beta_innov_var = _beta_innov_var; }
    void getBetaInnovRatio(float &beta_innov_ratio) const { beta_innov_ratio = _beta_test_ratio; }

    void getHaglInnov(float &hagl_innov) const { hagl_innov = _hagl_innov; }
    void getHaglInnovVar(float &hagl_innov_var) const { hagl_innov_var = _hagl_innov_var; }
    void getHaglInnovRatio(float &hagl_innov_ratio) const { hagl_innov_ratio = _hagl_test_ratio; }

    // 获取延迟时间窗口上的状态向量
    matrix::Vector<float, 24> getStateAtFusionHorizonAsVector() const;

    // 获取风速（m/s）
    const Vector2f &getWindVelocity() const { return _state.wind_vel; };

    // 获取风速方差
    Vector2f getWindVelocityVariance() const { return P.slice<2, 2>(22, 22).diag(); }

    // 获取真实空速（m/s）
    void get_true_airspeed(float *tas) const;

    // 获取完整协方差矩阵
    const matrix::SquareMatrix<float, 24> &covariances() const { return P; }

    // 获取协方差矩阵对角线元素
    matrix::Vector<float, 24> covariances_diagonal() const { return P.diag(); }

    // 获取姿态（四元数）协方差
    matrix::SquareMatrix<float, 4> orientation_covariances() const { return P.slice<4, 4>(0, 0); }

    // 获取线速度协方差
    matrix::SquareMatrix<float, 3> velocity_covariances() const { return P.slice<3, 3>(4, 4); }

    // 获取位置协方差
    matrix::SquareMatrix<float, 3> position_covariances() const { return P.slice<3, 3>(7, 7); }

    // 请求估计器决定是否采集传感器数据，并在需要时做预处理；若未定义则返回 true
    bool collect_gps(const gps_message &gps) override;

    // 获取 EKF WGS-84 原点位置、高度，以及该原点最后一次设置时的系统时间
    // 若原点有效则返回 true
    bool getEkfGlobalOrigin(uint64_t &origin_time, double &latitude, double &longitude, float &origin_alt) const;
    bool setEkfGlobalOrigin(const double latitude, const double longitude, const float altitude);

    float getEkfGlobalOriginAltitude() const { return _gps_alt_ref; }
    bool setEkfGlobalOriginAltitude(const float altitude);


    // 获取 EKF WGS-84 位置的 1σ 水平与垂直位置不确定度
    void get_ekf_gpos_accuracy(float *ekf_eph, float *ekf_epv) const;

    // 获取 EKF 本地位置的 1σ 水平与垂直位置不确定度
    void get_ekf_lpos_accuracy(float *ekf_eph, float *ekf_epv) const;

    // 获取 1σ 水平和垂直速度不确定度
    void get_ekf_vel_accuracy(float *ekf_evh, float *ekf_evv) const;

    // 获取估计器为保持在传感器限制内所需的车辆控制限值
    void get_ekf_ctrl_limits(float *vxy_max, float *vz_max, float *hagl_min, float *hagl_max) const;

    // 将所有 IMU 偏置状态和协方差重置为初始对齐值。
    void resetImuBias();
    void resetGyroBias();
    void resetAccelBias();

    // 将所有磁力计偏置状态和协方差重置为初始对齐值。
    void resetMagBias();

    Vector3f getVelocityVariance() const { return P.slice<3, 3>(4, 4).diag(); };

    Vector3f getPositionVariance() const { return P.slice<3, 3>(7, 7).diag(); }

    // 返回包含输出预测器角度、速度和位置跟踪误差幅值的数组
    // 误差单位分别为（rad）、（m/sec）、（m）
    const Vector3f &getOutputTrackingError() const { return _output_tracking_error; }

    /*
    第一个参数返回 GPS 漂移指标，其数组位置含义如下：
    0 : 水平位置漂移率（m/s）
    1 : 垂直位置漂移率（m/s）
    2 : 滤波后的水平速度（m/s）
    第二个参数在 IMU 运动阻止漂移计算时返回 true
    该函数在指标已更新且尚未被本函数返回前返回 true
    */
    bool get_gps_drift_metrics(float drift[3], bool *blocked);

    // 若全球位置估计有效则返回 true
    // 若原点已设置、并且当前不处于无约束自由惯性导航中
    // 且尚未开始使用合成位置观测来约束漂移，则返回 true
    bool global_position_is_valid() const
    {
        return (_NED_origin_initialised && local_position_is_valid());
    }

    // 若局部位置估计有效则返回 true
    bool local_position_is_valid() const
    {
        return (!_deadreckon_time_exceeded && !_using_synthetic_position);
    }

    bool isTerrainEstimateValid() const { return _hagl_valid; };

    uint8_t getTerrainEstimateSensorBitfield() const { return _hagl_sensor_status.value; }

    // 获取相对于 NED 原点的地形垂直位置估计值
    float getTerrainVertPos() const { return _terrain_vpos; };

    // 获取垂直地形位置被重置的次数
    uint8_t getTerrainVertPosResetCounter() const { return _terrain_vpos_reset_counter; };

    // 获取地形方差
    float get_terrain_var() const { return _terrain_var; }

    Vector3f getGyroBias() const { return _state.delta_ang_bias / _dt_ekf_avg; } // 获取陀螺仪偏置（rad/s）
    Vector3f getAccelBias() const { return _state.delta_vel_bias / _dt_ekf_avg; } // 获取加速度计偏置（m/s**2）
    const Vector3f &getMagBias() const { return _state.mag_B; }

    Vector3f getGyroBiasVariance() const { return Vector3f{P(10, 10), P(11, 11), P(12, 12)} / _dt_ekf_avg; } // 获取陀螺仪偏置方差（rad/s）
    Vector3f getAccelBiasVariance() const { return Vector3f{P(13, 13), P(14, 14), P(15, 15)} / _dt_ekf_avg; } // 获取加速度计偏置方差（m/s**2）
    Vector3f getMagBiasVariance() const { return Vector3f{P(19, 19), P(20, 20), P(21, 21)}; }

    // 获取 GPS 检查状态
    void get_gps_check_status(uint16_t *val) const { *val = _gps_check_fail_status.value; }

    const auto &state_reset_status() const { return _state_reset_status; }

    // 返回上次重置后局部垂直位置变化量，以及重置事件次数
    void get_posD_reset(float *delta, uint8_t *counter) const
    {
        *delta = _state_reset_status.posD_change;
        *counter = _state_reset_status.posD_counter;
    }

    // 返回上次重置后局部垂直速度变化量，以及重置事件次数
    void get_velD_reset(float *delta, uint8_t *counter) const
    {
        *delta = _state_reset_status.velD_change;
        *counter = _state_reset_status.velD_counter;
    }

    // 返回上次重置后局部水平位置变化量，以及重置事件次数
    void get_posNE_reset(float delta[2], uint8_t *counter) const
    {
        _state_reset_status.posNE_change.copyTo(delta);
        *counter = _state_reset_status.posNE_counter;
    }

    // 返回上次重置后局部水平速度变化量，以及重置事件次数
    void get_velNE_reset(float delta[2], uint8_t *counter) const
    {
        _state_reset_status.velNE_change.copyTo(delta);
        *counter = _state_reset_status.velNE_counter;
    }

    // 返回上次重置后四元数变化量，以及重置事件次数
    void get_quat_reset(float delta_quat[4], uint8_t *counter) const
    {
        _state_reset_status.quat_change.copyTo(delta_quat);
        *counter = _state_reset_status.quat_counter;
    }

    // 获取 EKF 创新一致性检查状态信息，包含：
    // status - 每个 EKF 测量创新一致性检查的通过/失败状态位掩码整数
    // innovation test ratios - 创新量相对于接受阈值的比值
    // 当值 > 1 时，表示传感器测量量已超过最大可接受水平，已被 EKF 拒绝
    // 对于矢量型测量量（例如磁力计、GPS 位置等），返回其最大值
    void get_innovation_test_status(uint16_t &status, float &mag, float &vel, float &pos, float &hgt, float &tas,
                    float &hagl, float &beta) const;

    // 返回一个位掩码整数，描述哪些状态估计值可用于飞行控制
    void get_ekf_soln_status(uint16_t *status) const;

    // 返回从外部视觉到 EKF 参考坐标系的旋转四元数
    matrix::Quatf getVisionAlignmentQuaternion() const { return Quatf(_R_ev_to_ekf); };

    // 使用当前时间窗口上的最新 IMU 数据。
    Quatf calculate_quaternion() const;

    // 设置在 GPS 失效前需要持续保持健康状态的最短时间
    void set_min_required_gps_health_time(uint32_t time_us) { _min_gps_health_time_us = time_us; }

    // 获取 EKF-GSF 紧急偏航估计器的解数据
    // 当数据不可用时返回 false
    bool getDataEKFGSF(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF],
               float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]);

private:

    // 将内部状态和状态标志重置为默认值
    void reset();

    bool initialiseTilt();

    // 请求 EKF 将偏航重置为内部 EKF-GSF 滤波器给出的估计值
    // 并将速度和位置状态重置到 GPS。这样 EKF 会在后续飞行中忽略磁力计。
    // 这应仅在启动导航丢失故障保护前作为最后手段使用
    void requestEmergencyNavReset() { _do_ekfgsf_yaw_reset = true; }

    // 检查 EKF 是否仅使用惯性数据进行水平速度的航迹推算
    void update_deadreckoning_status();

    void updateTerrainValidity();

    struct {
        uint8_t velNE_counter;  ///< 最后一次水平位置重置事件的计数（若超过 255 可循环）
        uint8_t velD_counter;   ///< 最后一次垂直速度重置事件的计数（若超过 255 可循环）
        uint8_t posNE_counter;  ///< 最后一次水平位置重置事件的计数（若超过 255 可循环）
        uint8_t posD_counter;   ///< 最后一次垂直位置重置事件的计数（若超过 255 可循环）
        uint8_t quat_counter;   ///< 最后一次四元数重置事件的计数（若超过 255 可循环）
        Vector2f velNE_change;  ///< 上次重置后北东速度变化量 (m)
        float velD_change;  ///< 上次重置后下向速度变化量 (m/sec)
        Vector2f posNE_change;  ///< 上次重置后北、东位置变化量 (m)
        float posD_change;  ///< 上次重置后下向位置变化量 (m)
        Quatf quat_change;  ///< 上次重置后的四元数增量 - 乘以重置前四元数得到重置后四元数
    } _state_reset_status{};    ///< 复位事件监控结构，包含速度、位置、高度和偏航复位信息

    float _dt_ekf_avg{FILTER_UPDATE_PERIOD_S}; ///< EKF 的平均更新速率

    Vector3f _ang_rate_delayed_raw; ///< 融合时间窗上的未校正角速率向量 (rad/sec)

    stateSample _state{};       ///< 延迟时间窗上的 EKF 状态结构体

    bool _filter_initialised{false};    ///< 当 EKF 状态和协方差已初始化时为 true

    // 在使用相对位置里程计模型融合位置数据时使用的变量
    bool _fuse_hpos_as_odom{false};     ///< 当 NE 位置数据使用里程计假设进行融合时为 true
    Vector3f _pos_meas_prev;        ///< 使用里程计假设融合的 NED 位置测量值的上一时刻值 (m)
    Vector2f _hpos_pred_prev;       ///< 里程计融合所用的 NE 位置状态上一时刻预测值 (m)
    bool _hpos_prev_available{false};   ///< 当估计值和测量值的上一时刻值可用时为 true
    Dcmf _R_ev_to_ekf;          ///< 将观测量从 EV 旋转到 EKF 导航坐标系的变换矩阵，初始为单位矩阵
    bool _inhibit_ev_yaw_use{false};    ///< 当视觉偏航数据不应使用时为 true（例如：NE 融合要求真北）

    // 在融合时间窗上有新传感器数据可用时为 true 的布尔值
    bool _gps_data_ready{false};    ///< 当新 GPS 数据落后于融合时间窗并可用于融合时为 true
    bool _mag_data_ready{false};    ///< 当新磁力计数据落后于融合时间窗并可用于融合时为 true
    bool _baro_data_ready{false};   ///< 当新气压高度数据落后于融合时间窗并可用于融合时为 true
    bool _flow_data_ready{false};   ///< 当光流积分周期前沿落后于融合时间窗时为 true
    bool _ev_data_ready{false}; ///< 当新外部视觉系统数据落后于融合时间窗并可用于融合时为 true
    bool _tas_data_ready{false};    ///< 当新真实空速数据落后于融合时间窗并可用于融合时为 true
    bool _flow_for_terrain_data_ready{false}; /// 与 "_flow_data_ready" 相同，但用于单独地形估计器

    uint64_t _time_prev_gps_us{0};  ///< 从缓冲区读取上一个 GPS 数据的时间戳 (uSec)
    uint64_t _time_last_aiding{0};  ///< 已进行惯性仅航迹推算的持续时间 (uSec)
    bool _using_synthetic_position{false};  ///< 当使用合成位置来约束漂移时为 true

    uint64_t _time_last_hor_pos_fuse{0};    ///< 上次融合水平位置测量的时间 (uSec)
    uint64_t _time_last_hgt_fuse{0};    ///< 上次融合垂直位置测量的时间 (uSec)
    uint64_t _time_last_hor_vel_fuse{0};    ///< 上次融合水平速度测量的时间 (uSec)
    uint64_t _time_last_ver_vel_fuse{0};    ///< 上次融合垂直速度测量的时间 (uSec)
    uint64_t _time_last_delpos_fuse{0}; ///< 上次融合增量水平位置测量的时间 (uSec)
    uint64_t _time_last_of_fuse{0};     ///< 上次融合光流测量的时间 (uSec)
    uint64_t _time_last_flow_terrain_fuse{0}; ///< 上次融合地形估计光流测量的时间 (uSec)
    uint64_t _time_last_arsp_fuse{0};   ///< 上次融合空速测量的时间 (uSec)
    uint64_t _time_last_beta_fuse{0};   ///< 上次融合合成侧滑测量的时间 (uSec)
    uint64_t _time_last_fake_pos{0};    ///< 在没有外部辅助时为约束倾斜误差而伪造位置测量的最后时间 (uSec)

    uint64_t _time_last_gps_yaw_fuse{0};    ///< 上次融合 GPS 偏航测量的时间 (uSec)
    uint64_t _time_last_gps_yaw_data{0};    ///< 上一次 GPS 偏航测量可用的时间 (uSec)
    uint8_t _nb_gps_yaw_reset_available{0}; ///< 切换到其他辅助源前允许的剩余复位次数

    Vector2f _last_known_posNE;     ///< 最后已知的局部 NE (北东) 位置向量 (m)
    float _imu_collection_time_adj{0.0f};   ///< 为满足 FILTER_UPDATE_PERIOD_MS 设定的目标，IMU 采集需要提前的时间量 (秒)

    uint64_t _time_acc_bias_check{0};   ///< 上次加速度计偏置检查通过的时间 (uSec)
    uint64_t _delta_time_baro_us{0};    ///< 缓冲区中两个连续的延迟气压计样本之间的时间差 (uSec)

    Vector3f _earth_rate_NED;   ///< 地球自转向量 (NED坐标系)，单位：rad/s

    Dcmf _R_to_earth;   ///< 上次 EKF 预测的从机体坐标系到地球坐标系的变换矩阵

    // 用于磁力计融合模式选择
    Vector2f _accel_lpf_NE;         ///< 低通滤波后的水平地球坐标系加速度 (m/sec**2)
    float _yaw_delta_ef{0.0f};      ///< 最近绕地球坐标系 D (下) 轴测量的偏航角变化量 (rad)
    float _yaw_rate_lpf_ef{0.0f};       ///< 绕地球坐标系 D 轴滤波后的角速率 (rad/sec)
    bool _mag_bias_observable{false};   ///< 当有足够的旋转使得磁力计偏置误差可观测时为 true
    bool _yaw_angle_observable{false};  ///< 当有足够的水平加速度使得偏航角可观测时为 true
    uint64_t _time_yaw_started{0};      ///< 上次检测到偏航旋转机动的系统时间 (uSec)
    uint8_t _num_bad_flight_yaw_events{0};  ///< 在飞行中检测到不良航向并需要重置偏航角的次数
    uint64_t _mag_use_not_inhibit_us{0};    ///< 磁力计被禁用前的最后系统时间 (uSec)
    bool _mag_inhibit_yaw_reset_req{false}; ///< 当磁力计禁用状态持续足够长的时间，以至于在条件改善时需要重置偏航角时为 true
    float _last_static_yaw{0.0f};       ///< 当地面运动检查通过时记录的最后一个偏航角 (rad)
    bool _mag_yaw_reset_req{false};     ///< 当请求使用磁力计数据重置偏航角时为 true
    bool _mag_decl_cov_reset{false};    ///< 在磁场重置事件后，使用 fuseDeclination() 函数修改地球磁场协方差后为 true
    bool _synthetic_mag_z_active{false};    ///< 如果我们正在生成合成磁力计 Z 轴测量值则为 true
    bool _non_mag_yaw_aiding_running_prev{false};  ///< 当正在融合来自非磁力计的其他数据源（例如 EV 或 GPS）的航向时为 true

    bool _is_yaw_fusion_inhibited{false};       ///< 当偏航传感器被禁用时为 true

    SquareMatrix24f P;  ///< 状态协方差矩阵

    Vector3f _delta_vel_bias_var_accum;     ///< 用于速度增量偏置方差的 Kahan 求和算法累加器
    Vector3f _delta_angle_bias_var_accum;   ///< 用于角度增量偏置方差的 Kahan 求和算法累加器

    Vector3f _last_vel_obs;         ///< 上次速度观测值 (m/s)
    Vector3f _last_vel_obs_var;     ///< 上次速度观测方差 (m/s)**2
    Vector2f _last_fail_hvel_innov;     ///< 上次失败的水平速度新息 (m/s)**2
    float _vert_pos_innov_ratio{0.f};   ///< 垂直位置新息除以估计的新息标准差（新息测试比率）
    uint64_t _vert_pos_fuse_attempt_time_us{0}; ///< 上次尝试融合垂直位置测量值的系统时间 (uSec)
    float _vert_vel_innov_ratio{0.f};       ///< 垂直速度新息的标准差
    uint64_t _vert_vel_fuse_time_us{0}; ///< 上次尝试融合垂直速度测量值的系统时间 (uSec)

    Vector3f _gps_vel_innov;    ///< GPS 速度新息 (m/sec)
    Vector3f _gps_vel_innov_var;    ///< GPS 速度新息方差 ((m/sec)**2)

    Vector3f _gps_pos_innov;    ///< GPS 位置新息 (m)
    Vector3f _gps_pos_innov_var;    ///< GPS 位置新息方差 (m**2)

    Vector3f _ev_vel_innov; ///< 外部视觉速度新息 (m/sec)
    Vector3f _ev_vel_innov_var; ///< 外部视觉速度新息方差 ((m/sec)**2)

    Vector3f _ev_pos_innov; ///< 外部视觉位置新息 (m)
    Vector3f _ev_pos_innov_var; ///< 外部视觉位置新息方差 (m**2)

    Vector3f _baro_hgt_innov;       ///< 气压高度新息 (m)
    Vector3f _baro_hgt_innov_var;   ///< 气压高度新息方差 (m**2)

    Vector3f _rng_hgt_innov;    ///< 测距仪高度新息 (m)
    Vector3f _rng_hgt_innov_var;    ///< 测距仪高度新息方差 (m**2)

    Vector3f _aux_vel_innov;    ///< 水平辅助速度新息: (m/sec)
    Vector3f _aux_vel_innov_var;    ///< 水平辅助速度新息方差: ((m/sec)**2)

    float _heading_innov{0.0f}; ///< 航向测量新息 (rad)
    float _heading_innov_var{0.0f}; ///< 航向测量新息方差 (rad**2)

    Vector3f _mag_innov;        ///< 地球磁场新息 (Gauss)
    Vector3f _mag_innov_var;    ///< 地球磁场新息方差 (Gauss**2)

    Vector2f _drag_innov;       ///< 多旋翼阻力测量新息 (m/sec**2)
    Vector2f _drag_innov_var;   ///< 多旋翼阻力测量新息方差 ((m/sec**2)**2)

    float _airspeed_innov{0.0f};        ///< 空速测量新息 (m/sec)
    float _airspeed_innov_var{0.0f};    ///< 空速测量新息方差 ((m/sec)**2)

    float _beta_innov{0.0f};    ///< 合成侧滑角测量新息 (rad)
    float _beta_innov_var{0.0f};    ///< 合成侧滑角测量新息方差 (rad**2)

    float _hagl_innov{0.0f};        ///< 上次离地高度测量的新息 (m)
    float _hagl_innov_var{0.0f};        ///< 上次离地高度测量的新息方差 (m**2)

    // 光流处理
    Vector2f _flow_innov;       ///< 光流测量新息 (rad/sec)
    Vector2f _flow_innov_var;   ///< 光流新息方差 ((rad/sec)**2)
    Vector3f _flow_gyro_bias;   ///< 光流传感器角速率陀螺仪输出的偏置误差 (rad/sec)
    Vector2f _flow_vel_body;    ///< 经过校正的光流测量得到的速度 (机体坐标系)(m/s)
    Vector2f _flow_vel_ne;      ///< 经过校正的光流测量得到的速度 (局部坐标系) (m/s)
    Vector3f _imu_del_ang_of;   ///< 在与光流速率相同的时间段内累积的经偏置校正的角度增量测量值 (rad)
    float _delta_time_of{0.0f}; ///< 累积 _imu_del_ang_of 的时间，单位秒 (sec)
    uint64_t _time_bad_motion_us{0};    ///< 上次地面运动超出限制的系统时间 (uSec)
    uint64_t _time_good_motion_us{0};   ///< 上次地面运动在限制范围内的系统时间 (uSec)
    bool _inhibit_flow_use{false};  ///< 当光流和测距仪的使用被禁用时为 true
    Vector2f _flow_compensated_XY_rad;  ///< 去除机体旋转后测得的图像绕机体 X 和 Y 轴的角度增量 (rad)，右手定则旋转为正

    // 输出预测器状态
    Vector3f _delta_angle_corr; ///< 角度增量校正向量 (rad)
    Vector3f _vel_err_integ;    ///< 速度跟踪误差的积分 (m)
    Vector3f _pos_err_integ;    ///< 位置跟踪误差的积分 (m.s)
    Vector3f _output_tracking_error; ///< 包含角度、速度和位置跟踪误差的幅值 (rad, m/s, m)

    // 用于 GPS 质量检查的变量
    Vector3f _gps_pos_deriv_filt;   ///< GPS NED 位置导数 (m/sec)
    Vector2f _gps_velNE_filt;   ///< 滤波后的 GPS 北向和东向速度 (m/sec)
    float _gps_velD_diff_filt{0.0f};    ///< 滤波后的 GPS 下向速度 (m/sec)
    uint64_t _last_gps_fail_us{0};      ///< 上次 GPS 未通过检查的系统时间 (uSec)
    uint64_t _last_gps_pass_us{0};      ///< 上次 GPS 通过检查的系统时间 (uSec)
    float _gps_error_norm{1.0f};        ///< 归一化的 GPS 误差
    uint32_t _min_gps_health_time_us{10000000}; ///< 仅在这段时间后才将 GPS 标记为健康
    bool _gps_checks_passed{false};     ///> 当所有激活的 GPS 检查都通过时为 true

    // 用于发布 EKF 局部 NED 原点的 WGS-84 位置的变量
    uint64_t _last_gps_origin_time_us{0};   ///< 上次设置原点的时间 (uSec)
    float _gps_alt_ref{0.0f};       ///< WGS-84 高度 (m)

    // 用于初始滤波器对齐的变量
    bool _is_first_imu_sample{true};
    uint32_t _baro_counter{0};      ///< 初始化期间读取的气压计样本数
    uint32_t _mag_counter{0};       ///< 初始化期间读取的磁力计样本数
    AlphaFilter<Vector3f> _accel_lpf{0.1f}; ///< 用于对齐倾角的滤波后的加速度计测量值 (m/s/s)
    AlphaFilter<Vector3f> _gyro_lpf{0.1f};  ///< 用于对齐过度运动检查的滤波后的陀螺仪测量值 (rad/sec)

    // 用于在飞行中执行重置并在高度源之间切换的变量
    AlphaFilter<Vector3f> _mag_lpf{0.1f};   ///< 用于即时重置的滤波后的磁力计测量值 (Gauss)
    float _hgt_sensor_offset{0.0f};     ///< 如果希望在高度重置后保持相同高度，则按需设置此偏移量 (m)
    float _baro_hgt_offset{0.0f};       ///< 局部 NED 原点处的气压计高度读数 (m)

    // 用于控制起飞后功能激活的变量
    float _last_on_ground_posD{0.0f};   ///< 当 in_air 状态为 false 时的最后垂直位置 (m)
    uint64_t _flt_mag_align_start_time{0};  ///< 飞行中磁场对齐开始的时间 (uSec)
    uint64_t _time_last_mov_3d_mag_suitable{0}; ///< 上次检测到有足够运动可使用 3 轴磁力计融合的系统时间 (uSec)
    float _saved_mag_bf_variance[4] {}; ///< 保存的磁场状态方差，以供下次初始化时使用 (Gauss**2)
    Matrix2f _saved_mag_ef_covmat;      ///< 保存的 NE 磁场状态协方差子矩阵，以供下次初始化时使用 (Gauss**2)
    bool _velpos_reset_request{false};  ///< 当修复了较大的偏航误差且需要重置速度和位置状态时为 true

    gps_check_fail_status_u _gps_check_fail_status{};

    // 用于禁用加速度计偏置学习的变量
    bool _accel_bias_inhibit[3] {};     ///< 当指定轴的加速度计偏置学习被禁用时为 true
    Vector3f _accel_vec_filt;       ///< 应用低通滤波器后的加速度向量 (m/sec**2)
    float _accel_magnitude_filt{0.0f};  ///< 应用衰减包络滤波器后的加速度幅值 (rad/sec)
    float _ang_rate_magnitude_filt{0.0f};       ///< 应用衰减包络滤波器后的角速率幅值 (rad/sec)
    Vector3f _prev_dvel_bias_var;       ///< 保存的速度增量 XYZ 偏置方差 (m/sec)**2

    // 地形高度状态估计
    float _terrain_vpos{0.0f};      ///< 局部 NED 坐标系下飞行器下方地形的估计垂直位置 (m)
    float _terrain_var{1e4f};       ///< 地形位置估计的方差 (m**2)
    uint8_t _terrain_vpos_reset_counter{0}; ///< _terrain_vpos 被重置的次数
    uint64_t _time_last_hagl_fuse{0};       ///< 地形估计器上次融合测距样本的系统时间
    uint64_t _time_last_fake_hagl_fuse{0};  ///< 地形估计器上次融合伪测距样本的系统时间
    bool _terrain_initialised{false};   ///< 当地形估计器已初始化时为 true
    bool _hagl_valid{false};        ///< 当离地高度估计有效时为 true
    terrain_fusion_status_u _hagl_sensor_status{}; ///< 指示用于估计离地高度的传感器类型的结构体

    // 高度传感器状态
    bool _baro_hgt_faulty{false};       ///< 如果没有可用的有效气压计数据则为 true
    bool _gps_hgt_intermittent{false};  ///< 如果缓冲区的 GPS 高度数据断断续续则为 true
    bool _is_gps_yaw_faulty{false};     ///< 如果 GPS 偏航数据被滤波器拒绝的时间过长则为 true

    // IMU 故障状态
    uint64_t _time_bad_vert_accel{0};   ///< 上次检测到不良垂直加速度的时间 (uSec)
    uint64_t _time_good_vert_accel{0};  ///< 上次检测到良好垂直加速度的时间 (uSec)
    bool _bad_vert_accel_detected{false};   ///< 当检测到不良的垂直加速度计数据时为 true
    uint16_t _clip_counter{0};      ///< 发生限幅时递增、未发生时递减的计数器

    // 用于控制测距辅助功能的变量
    bool _is_range_aid_suitable{false}; ///< 当飞行中可使用测距仪代替主高度传感器作为高度参考时为 true

    float _height_rate_lpf{0.0f};

    // 更新实时互补滤波器状态。包括预测和校正步骤
    void calculateOutputStates(const imuSample &imu);
    void applyCorrectionToVerticalOutputBuffer(float vert_vel_correction);
    void applyCorrectionToOutputBuffer(const Vector3f &vel_correction, const Vector3f &pos_correction);

    // 初始化延迟 EKF 和实时互补滤波器的状态
    bool initialiseFilter(void);

    // 初始化 EKF 协方差矩阵
    void initialiseCovariance();

    // 预测 EKF 状态
    void predictState();

    // 预测 EKF 协方差
    void predictCovariance();

    // 磁力计测量的 EKF 顺序融合
    void fuseMag();

    // 以 321 或 312 转角序列中的第一个欧拉角作为观测量进行融合（目前使用磁力计测量偏航）
    void fuseHeading();

    // 以 321 Tait-Bryan 转角序列中的第一个旋转角定义偏航角
    // yaw : 321 Tait-Bryan 转角序列中第一个旋转角定义的角度观测值（rad）
    // yaw_variance : 偏航角观测方差（rad^2）
    // zero_innovation : 以零创新量融合数据
    void fuseYaw321(const float yaw, const float yaw_variance, bool zero_innovation);

    // 以 312 Tait-Bryan 转角序列中的第一个旋转角定义偏航角
    // yaw : 312 Tait-Bryan 转角序列中第一个旋转角定义的角度观测值（rad）
    // yaw_variance : 偏航角观测方差（rad^2）
    // zero_innovation : 以零创新量融合数据
    void fuseYaw312(const float yaw, const float yaw_variance, bool zero_innovation);

    // 使用创新量、观测方差和雅可比向量更新四元数状态和协方差
    // innovation : 预测值 - 测量值
    // variance : 观测方差
    // gate_sigma : 创新一致性检查门限大小（Sigma）
    // jacobian : 4x1 向量，表示观测量对每个四元数状态的偏导数
    void updateQuaternion(const float innovation, const float variance, const float gate_sigma,
                  const Vector4f &yaw_jacobian);

    // 融合双天线 GPS 单元获得的偏航角
    void fuseGpsYaw();

    // 使用双天线 GPS 获得的偏航角重置四元数状态
    // 返回 true 表示重置成功
    bool resetYawToGps();

    // 融合磁力计偏角测量
    // 传入参数是偏角不确定度（弧度）
    void fuseDeclination(float decl_sigma);

    // 对偏角与 NE 磁场状态估计长度施加合理限制
    void limitDeclination();

    // 融合气速测量
    void fuseAirspeed();

    // 融合合成零侧滑测量
    void fuseSideslip();

    // 融合机体坐标系中的阻力比力，用于多旋翼风估计
    void fuseDrag();

    // 融合单个速度和位置测量
    void fuseVelPosHeight(const float innov, const float innov_var, const int obs_index);

    void resetVelocity();

    void resetVelocityToGps();

    inline void resetHorizontalVelocityToOpticalFlow();

    inline void resetVelocityToVision();

    inline void resetHorizontalVelocityToZero();

    inline void resetVelocityTo(const Vector3f &vel);

    inline void resetHorizontalVelocityTo(const Vector2f &new_horz_vel);

    inline void resetVerticalVelocityTo(float new_vert_vel);

    void resetHorizontalPosition();

    void resetHorizontalPositionToGps();

    inline void resetHorizontalPositionToVision();

    inline void resetHorizontalPositionTo(const Vector2f &new_horz_pos);

    inline void resetVerticalPositionTo(const float &new_vert_pos);

    void resetHeight();

    // 融合光流视线速率测量
    void fuseOptFlow();

    bool fuseHorizontalVelocity(const Vector3f &innov, const Vector2f &innov_gate, const Vector3f &obs_var,
                    Vector3f &innov_var, Vector2f &test_ratio);

    bool fuseVerticalVelocity(const Vector3f &innov, const Vector2f &innov_gate, const Vector3f &obs_var,
                  Vector3f &innov_var, Vector2f &test_ratio);

    bool fuseHorizontalPosition(const Vector3f &innov, const Vector2f &innov_gate, const Vector3f &obs_var,
                    Vector3f &innov_var, Vector2f &test_ratiov, bool inhibit_gate = false);

    bool fuseVerticalPosition(const Vector3f &innov, const Vector2f &innov_gate, const Vector3f &obs_var,
                  Vector3f &innov_var, Vector2f &test_ratio);

    // 计算光流体轴角速率补偿
    // 如果偏置校正后的体轴角速率数据不可用，则返回 false
    bool calcOptFlowBodyRateComp();

    // 初始化地形垂直位置估计器
    // 若初始化成功则返回 true
    bool initHagl();

    bool shouldUseRangeFinderForHagl() const { return (_params.terrain_fusion_mode & TerrainFusionMask::TerrainFuseRangeFinder); }
    bool shouldUseOpticalFlowForHagl() const { return (_params.terrain_fusion_mode & TerrainFusionMask::TerrainFuseOpticalFlow); }

    // 运行地形估计器
    void runTerrainEstimator();

    // 使用测距仪测得的离地高度更新地形垂直位置估计
    void fuseHagl();

    // 使用光流测量更新地形垂直位置估计
    void fuseFlowForTerrain();

    // 使用偏角和磁力计测量重置航向和磁场状态
    // 返回 true 表示成功
    bool resetMagHeading(const Vector3f &mag_init, bool increase_yaw_var = true, bool update_buffer = true);

    // 使用外部视觉测量重置航向
    // 返回 true 表示成功
    bool resetYawToEv();

    // 强制重新对齐偏航角，使其与 GPS 提供的水平速度矢量对齐。
    // 该功能用于固定翼飞行器在起飞或离地后对偏航角进行对齐。
    bool realignYawGPS();

    // 返回用于对齐和融合处理的磁偏角（弧度）
    float getMagDeclination();

    // 调整输出滤波器以匹配融合时间窗上的 EKF 状态
    void alignOutputFilter();

    // 更新将 EV 导航坐标系观测量转换到 NED 的旋转矩阵
    void calcExtVisRotMat();

    Vector3f getVisionVelocityInEkfFrame() const;

    Vector3f getVisionVelocityVarianceInEkfFrame() const;

    // 用于计算 K<24,1> * H<1,24> * P<24,24> 的矩阵向量乘法，通过利用 H 的稀疏性进行了优化
    template <size_t ...Idxs>
    SquareMatrix24f computeKHP(const Vector24f &K, const SparseVector24f<Idxs...> &H) const
    {
        SquareMatrix24f KHP;
        constexpr size_t non_zeros = sizeof...(Idxs);
        float KH[non_zeros];

        for (unsigned row = 0; row < _k_num_states; row++) {
            for (unsigned i = 0; i < H.non_zeros(); i++) {
                KH[i] = K(row) * H.atCompressedIndex(i);
            }

            for (unsigned column = 0; column < _k_num_states; column++) {
                float tmp = 0.f;

                for (unsigned i = 0; i < H.non_zeros(); i++) {
                    const size_t index = H.index(i);
                    tmp += KH[i] * P(index, column);
                }

                KHP(row, column) = tmp;
            }
        }

        return KHP;
    }

    // 单个测量值的测量更新
    // 如果执行了融合则返回 true
    template <size_t ...Idxs>
    bool measurementUpdate(Vector24f &K, const SparseVector24f<Idxs...> &H, float innovation)
    {
        for (unsigned i = 0; i < 3; i++) {
            if (_accel_bias_inhibit[i]) {
                K(13 + i) = 0.0f;
            }
        }

        // 通过 P_new = (I -K*H)*P 应用协方差校正
        // 首先计算 KHP 的表达式
        // 然后计算 P - KHP
        const SquareMatrix24f KHP = computeKHP(K, H);

        const bool is_healthy = checkAndFixCovarianceUpdate(KHP);

        if (is_healthy) {
            // 应用协方差校正
            P -= KHP;

            fixCovarianceErrors(true);

            // 应用状态校正
            fuse(K, innovation);
        }

        return is_healthy;
    }

    // 如果协方差校正会导致负方差，则
    // 协方差矩阵处于不健康状态，必须进行修正
    bool checkAndFixCovarianceUpdate(const SquareMatrix24f &KHP);

    // 限制协方差矩阵的对角线元素
    // 当参数为 true 时强制对称
    void fixCovarianceErrors(bool force_symmetry);

    // 约束 EKF 状态
    void constrainStates();

    // 通用函数，在给定卡尔曼增益 K 和标量新息值的情况下执行融合步骤
    void fuse(const Vector24f &K, float innovation);

    float compensateBaroForDynamicPressure(float baro_alt_uncompensated) const override;

    // 从给定的纬度计算地球自转向量
    Vector3f calcEarthRateNED(float lat_rad) const;

    // 如果 GPS 质量足够好以设置原点并开始辅助，则返回 true
    bool gps_is_good(const gps_message &gps);

    // 控制滤波器融合模式
    void controlFusionModes();

    // 控制外部视觉观测的融合
    void controlExternalVisionFusion();

    // 控制光流观测的融合
    void controlOpticalFlowFusion();
    void updateOnGroundMotionForOpticalFlowChecks();
    void resetOnGroundMotionForOpticalFlowChecks();

    // 控制 GPS 观测数据的融合
    void controlGpsFusion();
    void controlGpsYawFusion(bool gps_checks_passing, bool gps_checks_failing);

    // 控制磁力计观测数据的融合
    void controlMagFusion();

    bool noOtherYawAidingThanMag() const;
    bool otherHeadingSourcesHaveStopped();

    void checkHaglYawResetReq();
    float getTerrainVPos() const { return isTerrainEstimateValid() ? _terrain_vpos : _last_on_ground_posD; }

    void runOnGroundYawReset();
    bool isYawResetAuthorized() const { return !_is_yaw_fusion_inhibited; }
    bool canResetMagHeading() const;
    void runInAirYawReset();
    bool canRealignYawUsingGps() const { return _control_status.flags.fixed_wing; }
    void runVelPosReset();

    void selectMagAuto();
    void check3DMagFusionSuitability();
    void checkYawAngleObservability();
    void checkMagBiasObservability();
    bool isYawAngleObservable() const { return _yaw_angle_observable; }
    bool isMagBiasObservable() const { return _mag_bias_observable; }
    bool canUse3DMagFusion() const;

    void checkMagDeclRequired();
    void checkMagInhibition();
    bool shouldInhibitMag() const;
    void checkMagFieldStrength();
    bool isStrongMagneticDisturbance() const { return _control_status.flags.mag_field_disturbed; }
    bool isMeasuredMatchingGpsMagStrength() const;
    bool isMeasuredMatchingAverageMagStrength() const;
    static bool isMeasuredMatchingExpected(float measured, float expected, float gate);
    void runMagAndMagDeclFusions();
    void run3DMagAndDeclFusions();

    // 控制测距仪观测数据的融合
    void controlRangeFinderFusion();

    // 控制空气数据观测的融合
    void controlAirDataFusion();

    // 控制合成侧滑角观测数据的融合
    void controlBetaFusion();

    // 控制多旋翼阻力比力观测数据的融合
    void controlDragFusion();

    // 控制气压高度观测数据的融合
    void controlBaroFusion();

    // 控制伪位置观测数据的融合以约束漂移
    void controlFakePosFusion();

    // 控制辅助速度观测数据的融合
    void controlAuxVelFusion();

    // 控制高度传感器超时、传感器更改和状态重置
    void controlHeightSensorTimeouts();

    void checkVerticalAccelerationHealth();

    // 控制组合高度融合模式（为气压计和测距仪高度之间的切换而实现）
    void controlHeightFusion();

    // 判断飞行条件是否适合使用测距仪代替主高度传感器
    void checkRangeAidSuitability();
    bool isRangeAidSuitable() const { return _is_range_aid_suitable; }

    // 设置控制标志以使用气压计高度
    void setControlBaroHeight();

    // 设置控制标志以使用测距仪高度
    void setControlRangeHeight();

    // 设置控制标志以使用 GPS 高度
    void setControlGPSHeight();

    // 设置控制标志以使用外部视觉高度
    void setControlEVHeight();

    void stopMagFusion();
    void stopMag3DFusion();
    void stopMagHdgFusion();
    void startMagHdgFusion();
    void startMag3DFusion();

    void startBaroHgtFusion();
    void startGpsHgtFusion();

    void updateBaroHgtOffset();

    // 返回 GPS 高度方差的估计值
    float getGpsHeightVariance();

    // 计算光流传感器的测量方差
    float calcOptFlowMeasVar();

    // 将四元数协方差旋转为等效旋转向量的方差
    Vector3f calcRotVecVariances();

    // 使用旋转向量方差初始化四元数协方差
    // 在四元数状态初始化之前不要调用
    void initialiseQuatCovariances(Vector3f &rot_vec_var);

    // 对与磁场相关的状态协方差执行有限的重置
    void resetMagRelatedCovariances();

    void resetQuatCov();
    void zeroQuatCov();
    void resetMagCov();

    // 对风状态协方差执行有限的重置
    void resetWindCovariance();

    // 执行风状态的重置
    void resetWindStates();

    // 检查测距仪数据是否连续
    void updateRangeDataContinuity();

    // 增加四元数的偏航误差方差
    // 参数为附加偏航方差，单位为 rad**2
    void increaseQuatYawErrVariance(float yaw_variance);

    // 加载和保存磁场状态协方差数据以备重用
    void loadMagCovData();
    void saveMagCovData();
    void clearMagCov();
    void zeroMagCov();

    void resetZDeltaAngBiasCov();

    // 取消四元数状态与其他状态的关联（解耦）
    void uncorrelateQuatFromOtherStates();

    // 给定 3D 磁力计传感器测量值，计算磁力计 Z 轴分量的合成值
    float calculate_synthetic_mag_z_measurement(const Vector3f &mag_meas, const Vector3f &mag_earth_predicted);

    bool isTimedOut(uint64_t last_sensor_timestamp, uint64_t timeout_period) const
    {
        return last_sensor_timestamp + timeout_period < _time_last_imu;
    }

    bool isRecent(uint64_t sensor_timestamp, uint64_t acceptance_interval) const
    {
        return sensor_timestamp + acceptance_interval > _time_last_imu;
    }

    void startGpsFusion();
    void stopGpsFusion();
    void stopGpsPosFusion();
    void stopGpsVelFusion();

    void startGpsYawFusion();
    void stopGpsYawFusion();

    void startEvPosFusion();
    void startEvVelFusion();
    void startEvYawFusion();

    void stopEvFusion();
    void stopEvPosFusion();
    void stopEvVelFusion();
    void stopEvYawFusion();

    void stopAuxVelFusion();

    void stopFlowFusion();

    void setVelPosFaultStatus(const int index, const bool status);

    // 将四元数状态和协方差重置为新的偏航值，同时保留横滚和俯仰
    // yaw : 欧拉偏航角 (rad)
    // yaw_variance : 偏航误差方差 (rad^2)
    // update_buffer : 如果状态变化也应应用于输出观测器缓冲区则为 true
    void resetQuatStateYaw(float yaw, float yaw_variance, bool update_buffer);

    // 用于控制 EKF-GSF 偏航估计器使用的声明

    // 偏航估计器实例
    EKFGSF_yaw _yawEstimator;

    int64_t _ekfgsf_yaw_reset_time{0};  ///< 上次紧急偏航重置的时间戳（uSec）
    bool _do_ekfgsf_yaw_reset{false};   // 当已请求紧急偏航重置时为 true
    uint8_t _ekfgsf_yaw_reset_count{0}; // 已将偏航重置为 EKF-GSF 估计值的次数

    // 在每次 _imu_sample_delayed 更新完成所有主 EKF 数据融合操作后调用一次
    void runYawEKFGSF();

    // 将主导航 EKF 的偏航重置为 EKF-GSF 偏航估计器给出的值
    // 同时将水平速度和位置重置为默认导航传感器值
    // 若重置成功则返回 true
    bool resetYawToEKFGSF();

    void resetGpsDriftCheckFilters();
};