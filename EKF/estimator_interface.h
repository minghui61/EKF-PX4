/****************************************************************************
 *
 *   Copyright (c) 2015-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * @file estimator_interface.h
 * 姿态估计器基类定义。
 *
 * @author Roman Bast <bapstroman@gmail.com>
 *
 */

#pragma once

#include <ecl.h>
#include "common.h"
#include "RingBuffer.h"
#include <AlphaFilter/AlphaFilter.hpp>
#include "imu_down_sampler.hpp"
#include "sensor_range_finder.hpp"
#include "utils.hpp"

#include <geo/geo.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

using namespace estimator;

class EstimatorInterface
{
public:
    // 请求估计器决定是否采集传感器数据，并在需要时做预处理；若未定义则返回 true
    virtual bool collect_gps(const gps_message &gps) = 0;

    void setIMUData(const imuSample &imu_sample);

    /*
    返回以下 IMU 振动指标，按数组位置如下：
    0 : 陀螺仪增量角圆锥运动指标 = 过滤后 (delta_angle x prev_delta_angle) 的长度
    1 : 陀螺仪高频振动 = 过滤后 (delta_angle - prev_delta_angle) 的长度
    2 : 加速度计高频振动 = 过滤后 (delta_velocity - prev_delta_velocity) 的长度
    */
    const Vector3f &getImuVibrationMetrics() const { return _vibe_metrics; }

    void setMagData(const magSample &mag_sample);

    void setGpsData(const gps_message &gps);

    void setBaroData(const baroSample &baro_sample);

    void setAirspeedData(const airspeedSample &airspeed_sample);

    void setRangeData(const rangeSample &range_sample);

    // 如果光流传感器陀螺仪增量角不可用，则将 gyro_xyz 向量字段置为 NaN，EKF 会改用其内部增量角数据
    void setOpticalFlowData(const flowSample &flow);

    // 设置外部视觉位置和姿态数据
    void setExtVisionData(const extVisionSample &evdata);

    void setAuxVelData(const auxVelSample &auxvel_sample);

    // 返回参数结构体的地址
    // 以便应用层访问
    parameters *getParamHandle() { return &_params; }

    // 设置飞行器着陆状态数据
    void set_in_air_status(bool in_air)
    {
        if (!in_air) {
            _time_last_on_ground_us = _time_last_imu;

        } else {
            _time_last_in_air = _time_last_imu;
        }
        _control_status.flags.in_air = in_air;
    }

    // 返回姿态是否可用
    bool attitude_valid() const { return ISFINITE(_output_new.quat_nominal(0)) && _control_status.flags.tilt_align; }

    // 获取飞行器着陆状态数据
    bool get_in_air_status() const { return _control_status.flags.in_air; }

    // 获取风估计状态
    bool get_wind_status() const { return _control_status.flags.wind; }

    // 设置飞行器是否为固定翼状态
    void set_is_fixed_wing(bool is_fixed_wing) { _control_status.flags.fixed_wing = is_fixed_wing; }

    // 设置是否应融合合成侧滑测量值
    void set_fuse_beta_flag(bool fuse_beta) { _control_status.flags.fuse_beta = (fuse_beta && _control_status.flags.in_air); }

    // 设置是否预计会出现地面效应导致的静压升高
    // 使用 _params.gnd_effect_deadzone 调整对预期静压升高的补偿
    // 标志在 GNDEFFECT_TIMEOUT 微秒后清除
    void set_gnd_effect_flag(bool gnd_effect)
    {
        _control_status.flags.gnd_effect = gnd_effect;
        _time_last_gnd_effect_on = _time_last_imu;
    }

    // 设置多旋翼比力阻力融合所使用的空气密度
    void set_air_density(float air_density) { _air_density = air_density; }

    // 设置测距仪报告的传感器限制
    void set_rangefinder_limits(float min_distance, float max_distance)
    {
        _range_sensor.setLimits(min_distance, max_distance);
    }

    // 设置光流传感器报告的传感器限制
    void set_optical_flow_limits(float max_flow_rate, float min_distance, float max_distance)
    {
        _flow_max_rate = max_flow_rate;
        _flow_min_distance = min_distance;
        _flow_max_distance = max_distance;
    }

    // 考虑的标志为 opt_flow、gps、ev_vel 和 ev_pos
    bool isOnlyActiveSourceOfHorizontalAiding(bool aiding_flag) const;

    /*
     * 检查是否还有其他水平辅助源处于活动状态
     * 警告：不能说明所选源是否
     * 活动，请使用 isOnlyActiveSourceOfHorizontalAiding() 进行确认
     *
     * 考虑的标志为 opt_flow、gps、ev_vel 和 ev_pos
     *
     * @param aiding_flag _control_status.flags 中的某个标志
     * @return true 表示存在除 aiding_flag 之外的活动辅助源
     */
    bool isOtherSourceOfHorizontalAidingThan(bool aiding_flag) const;

    // 返回是否至少有一个水平辅助源处于活动状态
    // 考虑的标志为 opt_flow、gps、ev_vel 和 ev_pos
    bool isHorizontalAidingActive() const;

    int getNumberOfActiveHorizontalAidingSources() const;

    // 返回 EKF 是否仅使用惯性数据进行位置无源推算（dead reckoning）
    bool inertial_dead_reckoning() const { return _is_dead_reckoning; }

    const matrix::Quatf &getQuaternion() const { return _output_new.quat_nominal; }

    // 获取机体原点在本地 NED 地球坐标系中的速度
    Vector3f getVelocity() const { return _output_new.vel - _vel_imu_rel_body_ned; }

    // 获取地球坐标系中的速度导数
    const Vector3f &getVelocityDerivative() const { return _vel_deriv; }

    // 获取机体原点在本地 NED 地球坐标系中的垂直位置导数
    float getVerticalPositionDerivative() const { return _output_vert_new.vert_vel - _vel_imu_rel_body_ned(2); }

    // 获取机体原点在本地地球坐标系中的位置
    Vector3f getPosition() const
    {
        // 将 IMU 相对机体原点的位置旋转到地球坐标系
        const Vector3f pos_offset_earth = _R_to_earth_now * _params.imu_pos_body;
        // 从 EKF 位置（位于 IMU）中减去该偏移，以得到机体原点位置
        return _output_new.pos - pos_offset_earth;
    }

    // 获取磁偏角（度），以便在下一次启动时保存使用
    // 当磁偏角可保存时返回 true
    // 下一次启动时，将 param.mag_declination_deg 设置为已保存的值
    bool get_mag_decl_deg(float *val) const
    {
        if (_NED_origin_initialised && (_params.mag_declination_source & MASK_SAVE_GEO_DECL)) {
            *val = math::degrees(_mag_declination_gps);
            return true;

        } else {
            return false;
        }
    }

    // 获取 EKF 模式状态
    const filter_control_status_u &control_status() const { return _control_status; }
    const decltype(filter_control_status_u::flags) &control_status_flags() const { return _control_status.flags; }

    const filter_control_status_u &control_status_prev() const { return _control_status_prev; }
    const decltype(filter_control_status_u::flags) &control_status_prev_flags() const { return _control_status_prev.flags; }

    // 获取 EKF 内部故障状态
    const fault_status_u &fault_status() const { return _fault_status; }
    const decltype(fault_status_u::flags) &fault_status_flags() const { return _fault_status.flags; }

    const innovation_fault_status_u &innov_check_fail_status() const { return _innov_check_fail_status; }
    const decltype(innovation_fault_status_u::flags) &innov_check_fail_status_flags() const { return _innov_check_fail_status.flags; }

    const warning_event_status_u &warning_event_status() const { return _warning_events; }
    const decltype(warning_event_status_u::flags) &warning_event_flags() const { return _warning_events.flags; }
    void clear_warning_events() { _warning_events.value = 0; }

    const information_event_status_u &information_event_status() const { return _information_events; }
    const decltype(information_event_status_u::flags) &information_event_flags() const { return _information_events.flags; }
    void clear_information_events() { _information_events.value = 0; }

    bool isVehicleAtRest() const { return _control_status.flags.vehicle_at_rest; }

    // 获取平均 IMU 更新周期 (秒)
    float get_dt_imu_avg() const { return _dt_imu_avg; }

    // 获取在延迟时间窗上的 IMU 样本
    const imuSample &get_imu_sample_delayed() const { return _imu_sample_delayed; }

    // 获取在延迟时间窗上的气压计样本
    const baroSample &get_baro_sample_delayed() const { return _baro_sample_delayed; }

    const bool& global_origin_valid() const { return _NED_origin_initialised; }
    const map_projection_reference_s& global_origin() const { return _pos_ref; }

    void print_status();

    static constexpr unsigned FILTER_UPDATE_PERIOD_MS{10};  // EKF 预测周期（毫秒）- 理想情况下这应该是 IMU 时间增量的整数倍
    static constexpr float FILTER_UPDATE_PERIOD_S{FILTER_UPDATE_PERIOD_MS * 0.001f};

protected:

    EstimatorInterface() = default;
    virtual ~EstimatorInterface() = default;

    virtual bool init(uint64_t timestamp) = 0;

    parameters _params;     // 滤波器参数

    /*
     OBS_BUFFER_LENGTH 定义了我们可以缓冲多少个观测值（非 IMU 测量值）
     这设定了我们可以处理非 IMU 测量值的最大频率。如果测量值
     在上一测量值之后过早到达，将不会被处理。
     最大频率 (Hz) = (OBS_BUFFER_LENGTH - 1) / (IMU_BUFFER_LENGTH * FILTER_UPDATE_PERIOD_S)
     可以调整此值以匹配最大传感器数据速率，并为抖动留出一些裕量。
    */
    uint8_t _obs_buffer_length{0};

    /*
     IMU_BUFFER_LENGTH 定义了我们缓冲多少个 IMU 样本，这设定了从当前时间到
     EKF 融合时间窗的时间延迟，也就是我们能补偿的、相对于 IMU 的最大传感器时间偏移。
     最大传感器时间偏移 (msec) =  IMU_BUFFER_LENGTH * FILTER_UPDATE_PERIOD_MS
     可以将其调整为比最大观测时间延迟长 FILTER_UPDATE_PERIOD_MS 的值。
    */
    uint8_t _imu_buffer_length{0};

    float _dt_imu_avg{0.0f};    // 平均 IMU 更新周期 (秒)

    imuSample _imu_sample_delayed{};    // 捕获在延迟时间窗上的 IMU 样本

    // 捕获在延迟时间窗上的测量值的测量样本
    magSample _mag_sample_delayed{};
    baroSample _baro_sample_delayed{};
    gpsSample _gps_sample_delayed{};
    sensor::SensorRangeFinder _range_sensor{};
    airspeedSample _airspeed_sample_delayed{};
    flowSample _flow_sample_delayed{};
    extVisionSample _ev_sample_delayed{};
    dragSample _drag_sample_delayed{};
    dragSample _drag_down_sampled{};    // 降采样后的阻力比力数据 (滤波器预测速率 -> 观测速率)
    auxVelSample _auxvel_sample_delayed{};

    float _air_density{CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C};        // 空气密度 (kg/m**3)

    // 传感器限制
    float _flow_max_rate{0.0f}; ///< 光流传感器能测量的最大角速度 (rad/s)
    float _flow_min_distance{0.0f}; ///< 光流传感器能运行的最小距离 (m)
    float _flow_max_distance{0.0f}; ///< 光流传感器能运行的最大距离 (m)

    // 输出预测器
    outputSample _output_new{};     // 非延迟时间窗（实时）上的滤波器输出
    outputVert _output_vert_new{};      // 非延迟时间窗（实时）上的垂直滤波器输出
    imuSample _newest_high_rate_imu_sample{};       // 捕获最新 IMU 数据的 IMU 样本
    Matrix3f _R_to_earth_now;       // 当前时间从机体坐标系到地球坐标系的旋转矩阵
    Vector3f _vel_imu_rel_body_ned;     // 在 NED 地球坐标系下 IMU 相对于机体原点的速度
    Vector3f _vel_deriv;        // 在 NED 地球坐标系下 IMU 处的速度导数 (m/s/s)

    bool _imu_updated{false};      // 如果 EKF 应该更新（已完成降采样过程）则为 true
    bool _initialised{false};      // 如果 EKF 接口实例（数据缓冲）已初始化则为 true

    bool _NED_origin_initialised{false};
    bool _gps_speed_valid{false};
    float _gps_origin_eph{0.0f}; // GPS 原点的水平位置不确定度
    float _gps_origin_epv{0.0f}; // GPS 原点的垂直位置不确定度
    struct map_projection_reference_s _pos_ref {};   // 包含 EKF 原点的 WGS-84 经纬度位置 (弧度)
    struct map_projection_reference_s _gps_pos_prev {};   // 包含上一条 GPS 消息的 WGS-84 经纬度位置 (弧度)
    float _gps_alt_prev{0.0f};  // 上一条 GPS 消息的高度 (m)
    float _gps_yaw_offset{0.0f};    // 用于偏航角估计的双 GPS 天线的偏航偏移角 (弧度)。

    // 新息一致性检查的监控比率
    float _yaw_test_ratio{};        // 偏航新息一致性检查比率
    Vector3f _mag_test_ratio;       // 磁力计 XYZ 轴新息一致性检查比率
    Vector2f _gps_vel_test_ratio;       // GPS 速度新息一致性检查比率
    Vector2f _gps_pos_test_ratio;       // GPS 位置新息一致性检查比率
    Vector2f _ev_vel_test_ratio;        // 外部视觉 (EV) 速度新息一致性检查比率
    Vector2f _ev_pos_test_ratio ;       // 外部视觉 (EV) 位置新息一致性检查比率
    Vector2f _aux_vel_test_ratio;       // 辅助水平速度新息一致性检查比率
    Vector2f _baro_hgt_test_ratio;      // 气压计高度新息一致性检查比率
    Vector2f _rng_hgt_test_ratio;       // 测距仪高度新息一致性检查比率
    float _optflow_test_ratio{};        // 光流新息一致性检查比率
    float _tas_test_ratio{};        // 真实空速 (TAS) 新息一致性检查比率
    float _hagl_test_ratio{};       // 离地高度测量新息一致性检查比率
    float _beta_test_ratio{};       // 侧滑角新息一致性检查比率
    Vector2f _drag_test_ratio;      // 阻力新息一致性检查比率
    innovation_fault_status_u _innov_check_fail_status{};

    bool _is_dead_reckoning{false};     // 如果我们不再融合可约束水平速度漂移的测量值，则为 true (无源航位推算)
    bool _deadreckon_time_exceeded{true};   // 如果水平导航解进行航位推算的时间过长且已无效，则为 true
    bool _is_wind_dead_reckoning{false};    // 如果我们依靠相对风的测量值进行导航，则为 true

    float _gps_drift_metrics[3] {}; // 包含 GPS 漂移指标的数组
                    // [0] 水平位置漂移率 (m/s)
                    // [1] 垂直位置漂移率 (m/s)
                    // [2] 滤波后的水平速度 (m/s)
    uint64_t _time_last_move_detect_us{0};  // 上次移动检测事件的时间戳 (微秒)
    uint64_t _time_last_on_ground_us{0};    ///< 上次我们在地面的时间 (uSec)
    uint64_t _time_last_in_air{0};      ///< 上次我们在空中的时间 (uSec)
    bool _gps_drift_updated{false}; // 当 _gps_drift_metrics 已更新并准备好被获取时为 true

    // 数据缓冲区实例
    RingBuffer<imuSample> _imu_buffer{12};           // 缓冲区长度为 12，使用默认参数
    RingBuffer<outputSample> _output_buffer{12};
    RingBuffer<outputVert> _output_vert_buffer{12};

    RingBuffer<gpsSample> _gps_buffer;
    RingBuffer<magSample> _mag_buffer;
    RingBuffer<baroSample> _baro_buffer;
    RingBuffer<rangeSample> _range_buffer;
    RingBuffer<airspeedSample> _airspeed_buffer;
    RingBuffer<flowSample>  _flow_buffer;
    RingBuffer<extVisionSample> _ext_vision_buffer;
    RingBuffer<dragSample> _drag_buffer;
    RingBuffer<auxVelSample> _auxvel_buffer;

    // 缓冲区中保存的最新测量值的时间戳（微秒）
    uint64_t _time_last_imu{0};
    uint64_t _time_last_gps{0};
    uint64_t _time_last_mag{0}; ///< 上一次磁力计采样的测量时间 (uSec)
    uint64_t _time_last_baro{0};
    uint64_t _time_last_range{0};
    uint64_t _time_last_airspeed{0};
    uint64_t _time_last_ext_vision{0};
    uint64_t _time_last_optflow{0};
    uint64_t _time_last_auxvel{0};
    // 最后一次在外部开启气压计地面效应补偿的时间 (uSec)
    uint64_t _time_last_gnd_effect_on{0};

    fault_status_u _fault_status{};

    // 分配数据缓冲区并初始化接口变量
    bool initialise_interface(uint64_t timestamp);

    float _mag_declination_gps{NAN};         // geo 库使用最后一个有效 GPS 位置返回的磁偏角 (rad)
    float _mag_inclination_gps{NAN};      // geo 库使用最后一个有效 GPS 位置返回的磁倾角 (rad)
    float _mag_strength_gps{NAN};            // geo 库使用最后一个有效 GPS 位置返回的磁场强度 (T)

    // 这是滤波器控制模式的当前状态
    filter_control_status_u _control_status{};

    // 这是滤波器控制模式的上一状态 - 用于检测模式转换
    filter_control_status_u _control_status_prev{};

    virtual float compensateBaroForDynamicPressure(const float baro_alt_uncompensated) const = 0;

    // 这些用于记录单帧事件以供外部监控，并且不应被用于
    // 状态逻辑，因为它们在被读取后会在外部被清除。
    warning_event_status_u _warning_events{};
    information_event_status_u _information_events{};

private:

    inline void setDragData(const imuSample &imu);

    inline void computeVibrationMetric(const imuSample &imu);
    inline bool checkIfVehicleAtRest(float dt, const imuSample &imu);

    void printBufferAllocationFailed(const char *buffer_name);

    ImuDownSampler _imu_down_sampler{FILTER_UPDATE_PERIOD_S};

    unsigned _min_obs_interval_us{0}; // 能保证数据不丢失的两次观测之间的最小时间间隔 (usec)

    // IMU 振动和运动监控
    Vector3f _delta_ang_prev;   // 上一次 IMU 测量的角度增量
    Vector3f _delta_vel_prev;   // 上一次 IMU 测量的速度增量
    Vector3f _vibe_metrics; // IMU 振动指标
                    // [0] IMU 角度增量中的圆锥运动振动水平 (rad^2)
                    // [1] IMU 角度增量数据中的高频振动水平 (rad)
                    // [2] IMU 速度增量数据中的高频振动水平 (m/s)

    // 用于气压计数据的降采样
    uint64_t _baro_timestamp_sum{0};    // 累加的时间戳，用于提供平均样本的时间戳
    float _baro_alt_sum{0.0f};          // 累加的气压高度读数 (m)
    uint8_t _baro_sample_count{0};      // 累加的气压高度测量值的数量

    // 供多旋翼特有的阻力融合使用
    uint8_t _drag_sample_count{0};  // 以滤波器预测速率累积的阻力比力样本数量
    float _drag_sample_time_dt{0.0f};   // 用于构成 _drag_down_sampled 的所有样本的时间积分 (sec)

    // 用于磁力计数据的降采样
    uint64_t _mag_timestamp_sum{0};
    Vector3f _mag_data_sum;
    uint8_t _mag_sample_count{0};

    // 观测缓冲区最终分配失败标志
    bool _gps_buffer_fail{false};
    bool _mag_buffer_fail{false};
    bool _baro_buffer_fail{false};
    bool _range_buffer_fail{false};
    bool _airspeed_buffer_fail{false};
    bool _flow_buffer_fail{false};
    bool _ev_buffer_fail{false};
    bool _drag_buffer_fail{false};
    bool _auxvel_buffer_fail{false};

};