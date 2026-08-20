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
 * @file common.h
 * 姿态估计器基类定义。
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Siddharth Bharat Purohit <siddharthbharatpurohit@gmail.com>
 *
 */
#pragma once

#include <matrix/math.hpp>

namespace estimator
{
// 进入 estimator 命名空间
// 所有EKF相关的类型和函数都在这个命名空间中
// 避免与其他库的命名冲突
using matrix::AxisAnglef;   // 轴角表示（浮点），用于角度增量→四元数转换
using matrix::Dcmf;         // 方向余弦矩阵（3×3旋转矩阵，浮点）
using matrix::Eulerf;       // 欧拉角表示（roll, pitch, yaw）
using matrix::Matrix3f;     // 3×3矩阵（浮点）
using matrix::Quatf;        // 四元数表示（q0, q1, q2, q3）
using matrix::Vector2f;     // 2维向量（浮点）
using matrix::Vector3f;     // 3维向量（浮点）
using matrix::wrap_pi;      // 将角度限制在[-π, π]范围的函数

enum class velocity_frame_t : uint8_t {
    LOCAL_FRAME_FRD, // 局部坐标系 前-右-下 (FRD)
    BODY_FRAME_FRD   // 机体坐标系 前-右-下 (FRD)
};

// GPS接收器直接输出的原始数据格式
struct gps_message {
    uint64_t time_usec{0};
    int32_t lat;        ///< 纬度，单位：1E-7 度
    int32_t lon;        ///< 经度，单位：1E-7 度
    int32_t alt;        ///< 高于平均海平面 (MSL) 的海拔高度，单位：1E-3 米 (毫米)
    float yaw;          ///< 偏航角。如果未设置则为 NaN (用于双天线 GPS)，单位：弧度 (rad)，范围 [-PI, PI]
    float yaw_offset;   ///< 双天线 GPS 的航向/偏航角偏移量 - 详见 GPS_YAW_OFFSET 的描述
    uint8_t fix_type;   ///< 0-1: 无定位, 2: 2D 定位, 3: 3D 定位, 4: RTCM 伪距差分, 5: RTK (实时动态差分)
    float eph;          ///< GPS 水平位置精度，单位：米
    float epv;          ///< GPS 垂直位置精度，单位：米
    float sacc;         ///< GPS 速度精度，单位：米/秒
    float vel_m_s;      ///< GPS 地面速度，单位：米/秒
    Vector3f vel_ned;   ///< GPS NED (北-东-地) 地面速度
    bool vel_ned_valid; ///< GPS 地面速度是否有效
    uint8_t nsats;      ///< 使用的卫星数量
    float pdop;         ///< 位置精度衰减因子 (PDOP)
};

struct outputSample {
    uint64_t    time_us{0}; ///< 测量时间戳，单位：微秒 (uSec)
    Quatf  quat_nominal;    ///< 描述飞行器姿态的标称四元数
    Vector3f    vel;        ///< 地球坐标系下的 NED 速度估计值，单位：米/秒
    Vector3f    pos;        ///< 地球坐标系下的 NED 位置估计值，单位：米
};

struct outputVert {
    uint64_t    time_us{0};     ///< 测量时间戳，单位：微秒 (uSec)
    float       vert_vel;       ///< 使用备用算法计算出的垂直速度，单位：米/秒
    float       vert_vel_integ; ///< 垂直速度的积分，单位：米
    float       dt;             ///< 时间间隔 (delta time)，单位：秒
};

struct imuSample {
    uint64_t    time_us{0};     ///< 测量时间戳，单位：微秒 (uSec)
    Vector3f    delta_ang;      ///< 机体坐标系下的角度增量 (陀螺仪测量值的积分)，单位：弧度
    Vector3f    delta_vel;      ///< 机体坐标系下的速度增量 (加速度计测量值的积分)，单位：米/秒
    float       delta_ang_dt;   ///< 角度增量积分周期，单位：秒
    float       delta_vel_dt;   ///< 速度增量积分周期，单位：秒
    bool        delta_vel_clipping[3]{}; ///< 如果样本包含任何加速度计限幅 (饱和)，则相应轴为 true
};

struct gpsSample {
    uint64_t    time_us{0}; ///< 测量时间戳，单位：微秒 (uSec)
    Vector2f    pos;        ///< NE (北-东) 地球坐标系下的 GPS 水平位置测量值，单位：米
    float       hgt;        ///< GPS 高度测量值，单位：米
    Vector3f    vel;        ///< NED (北-东-地) 地球坐标系下的 GPS 速度测量值，单位：米/秒
    float       yaw;        ///< 偏航角。如果未设置则为 NaN (用于双天线 GPS)，单位：弧度，范围 [-PI, PI]
    float       hacc;       ///< 水平位置 1 倍标准差 (1-std) 误差，单位：米
    float       vacc;       ///< 垂直位置 1 倍标准差 (1-std) 误差，单位：米
    float       sacc;       ///< 速度 1 倍标准差 (1-std) 误差，单位：米/秒
};

struct magSample {
    uint64_t    time_us{0}; ///< 测量时间戳，单位：微秒 (uSec)
    Vector3f    mag;        ///< NED 磁力计在机体坐标系下的测量值，单位：高斯 (Gauss)
};

struct baroSample {
    uint64_t    time_us{0}; ///< 测量时间戳，单位：微秒 (uSec)
    float       hgt;        ///< 海拔气压高度，单位：米
};

struct rangeSample {
    uint64_t    time_us{0}; ///< 测量时间戳，单位：微秒 (uSec)
    float       rng;        ///< 测距 (到地面的距离) 测量值，单位：米
    int8_t      quality;    ///< 信号质量百分比 (0...100%)，0 = 无效信号，100 = 完美信号，-1 = 未知信号质量
};

struct airspeedSample {
    uint64_t    time_us{0};     ///< 测量时间戳，单位：微秒 (uSec)
    float       true_airspeed;  ///< 真实空速测量值，单位：米/秒
    float       eas2tas;        ///< 等效空速到真实空速的转换系数
};

struct flowSample {
    uint64_t time_us{0};    ///< 积分周期起始前沿的时间戳，单位：微秒 (uSec)
    Vector2f flow_xy_rad;   ///< 测量的图像绕机体 X 和 Y 轴的角度增量 (弧度)，右手定则旋转为正
    Vector3f gyro_xyz;      ///< 由角速率陀螺仪测量得到的惯性系绕机体轴的角度增量 (弧度)，右手定则旋转为正
    float    dt;            ///< 积分时间长度，单位：秒
    uint8_t  quality;       ///< 质量指示器，范围 0 到 255
};

struct extVisionSample {
    uint64_t time_us{0};    ///< 测量时间戳，单位：微秒 (uSec)
    Vector3f pos;   ///< 外部视觉局部参考坐标系下的 XYZ 位置 (米) - Z 轴必须与向下 (down) 轴对齐
    Vector3f vel;   ///< 定义在 vel_frame 变量指定的参考坐标系中的 FRD 速度 (米/秒) - Z 轴必须与向下轴对齐
    Quatf quat;     ///< 定义从机体坐标系到地球坐标系旋转的四元数
    Vector3f posVar;    ///< XYZ 位置方差 (米**2)
    Matrix3f velCov;    ///< XYZ 速度协方差 ((米/秒)**2)
    float angVar;       ///< 航向角度方差 (弧度**2)
    velocity_frame_t vel_frame = velocity_frame_t::BODY_FRAME_FRD;
};

struct dragSample {
    uint64_t time_us{0};    ///< 测量时间戳，单位：微秒 (uSec)
    Vector2f accelXY;   ///< 沿机体 X 和 Y 轴测量的比力 (米/秒**2)
};

struct auxVelSample {
    uint64_t time_us{0};    ///< 测量时间戳，单位：微秒 (uSec)
    Vector3f vel;       ///< 相对于局部原点的测量的 NE (北-东) 速度 (米/秒)
    Vector3f velVar;    ///< NE 速度估计误差方差 (米/秒)**2
};

// vdist_sensor_type 的整数定义
#define VDIST_SENSOR_BARO  0    ///< 使用气压计高度
#define VDIST_SENSOR_GPS   1    ///< 使用 GPS 高度
#define VDIST_SENSOR_RANGE 2    ///< 使用测距仪高度
#define VDIST_SENSOR_EV    3    ///< 使用外部视觉高度

// mag_declination_source 的位位置定义
#define MASK_USE_GEO_DECL   (1<<0)  ///< 当 GPS 位置可用时，设为 true 以使用 geo 库中的地磁偏角；设为 false 时始终使用 EKF2_MAG_DECL 值
#define MASK_SAVE_GEO_DECL  (1<<1)  ///< 设为 true 时，将 geo 库返回的值保存到 EKF2_MAG_DECL 参数中
#define MASK_FUSE_DECL      (1<<2)  ///< 在执行 3 轴融合时，设为 true 表示始终将偏角作为观测量进行融合以约束漂移

// fusion_mode 的位位置定义
#define MASK_USE_GPS    (1<<0)      ///< 设为 true 以使用 GPS 数据
#define MASK_USE_OF     (1<<1)      ///< 设为 true 以使用光流数据
#define MASK_INHIBIT_ACC_BIAS (1<<2)    ///< 设为 true 以抑制加速度计增量速度偏差估计
#define MASK_USE_EVPOS  (1<<3)      ///< 设为 true 以使用外部视觉位置数据
#define MASK_USE_EVYAW  (1<<4)      ///< 设为 true 以使用外部视觉四元数数据进行偏航估计
#define MASK_USE_DRAG  (1<<5)       ///< 设为 true 以使用多旋翼阻力模型来估计风速
#define MASK_ROTATE_EV  (1<<6)      ///< 当外部视觉 (EV) 观测不在 NED 参考系中时，设为 true 以在使用前进行旋转转换
#define MASK_USE_GPSYAW  (1<<7)     ///< 若可用，设为 true 以使用 GPS 偏航数据
#define MASK_USE_EVVEL  (1<<8)      ///< 设为 true 以使用外部视觉速度数据

enum TerrainFusionMask : int32_t {
    TerrainFuseRangeFinder = (1 << 0),
    TerrainFuseOpticalFlow = (1 << 1)
};

// mag_fusion_type 的整数定义
#define MAG_FUSE_TYPE_AUTO      0   ///< 自动选择使用航向融合或 3D 磁力计融合
#define MAG_FUSE_TYPE_HEADING   1   ///< 始终使用简单的偏航角融合。这种方式精度较低，但受地球磁场畸变影响较小。不应用于超出 -60 到 +60 度范围的俯仰角
#define MAG_FUSE_TYPE_3D        2   ///< 始终使用磁力计 3 轴融合。这种方式精度较高，但受局部地球磁场畸变影响较大
#define MAG_FUSE_TYPE_UNUSED    3   ///< 未实现
#define MAG_FUSE_TYPE_INDOOR    4   ///< 与选项 0 相同，但除非使用了地球坐标系的外部辅助(如 GPS 或外部视觉)，否则不会使用磁力计或偏航融合。这可防止室内操作相关的不一致磁场导致状态估计性能下降。
#define MAG_FUSE_TYPE_NONE  5   ///< 在任何情况下都不使用磁力计。如果通过 EKF2_AID_MASK 参数选择了其他偏航数据源，则可以使用其他偏航数据源。

// 传感器最大间隔，单位为微秒 (uSec)
#define GPS_MAX_INTERVAL  (uint64_t)5e5 ///< GPS 测量值之间的最大允许时间间隔 (uSec)
#define BARO_MAX_INTERVAL (uint64_t)2e5 ///< 气压高度测量值之间的最大允许时间间隔 (uSec)
#define RNG_MAX_INTERVAL  (uint64_t)2e5 ///< 测距仪测量值之间的最大允许时间间隔 (uSec)
#define EV_MAX_INTERVAL   (uint64_t)2e5 ///< 外部视觉系统测量值之间的最大允许时间间隔 (uSec)

// 坏加速度计检测与缓解
#define BADACC_PROBATION  (uint64_t)10e6    ///< 被判定为不良的加速度计数据，必须连续通过检查多长时间才能重新被判定为良好 (uSec)
#define BADACC_BIAS_PNOISE  4.9f    ///< 当加速度计数据被判定为不良时，将速度增量的过程噪声设置为该值 (米/秒**2)

// 地面效应补偿
#define GNDEFFECT_TIMEOUT   10E6    ///< 地面效应保护在上次开启后将保持激活的最大时间周期 (uSec)

struct parameters {
    // 测量数据源控制
    int32_t fusion_mode{MASK_USE_GPS};      ///< 用于选择使用哪些辅助数据源的位掩码整数
    int32_t vdist_sensor_type{VDIST_SENSOR_BARO};   ///< 选择高度数据的主要来源
    int32_t terrain_fusion_mode{TerrainFusionMask::TerrainFuseRangeFinder |
                    TerrainFusionMask::TerrainFuseOpticalFlow}; ///< 地形估计器的辅助数据源选择位掩码
    int32_t sensor_interval_min_ms{20};     ///< 非 IMU 传感器更新之间的最小到达时间差。用于设置观测缓冲区的大小。(毫秒)

    // 测量时间延迟
    float mag_delay_ms{0.0f};       ///< 磁力计测量相对于 IMU 的延迟 (毫秒)
    float baro_delay_ms{0.0f};      ///< 气压计高度测量相对于 IMU 的延迟 (毫秒)
    float gps_delay_ms{110.0f};     ///< GPS 测量相对于 IMU 的延迟 (毫秒)
    float airspeed_delay_ms{100.0f};    ///< 空速测量相对于 IMU 的延迟 (毫秒)
    float flow_delay_ms{5.0f};      ///< 光流测量相对于 IMU 的延迟 (毫秒) - 这是指到光流积分区间中点的时间
    float range_delay_ms{5.0f};     ///< 测距仪测量相对于 IMU 的延迟 (毫秒)
    float ev_delay_ms{175.0f};      ///< 机外视觉测量相对于 IMU 的延迟 (毫秒)
    float auxvel_delay_ms{5.0f};        ///< 辅助速度测量相对于 IMU 的延迟 (毫秒)

    // 输入噪声
    float gyro_noise{1.5e-2f};      ///< 用于协方差预测的 IMU 角速率噪声 (弧度/秒)
    float accel_noise{3.5e-1f};     ///< 用于协方差预测的 IMU 加速度噪声 (米/秒**2)

    // 过程噪声
    float gyro_bias_p_noise{1.0e-3f};   ///< 用于 IMU 速率陀螺仪偏差预测的过程噪声 (弧度/秒**2)
    float accel_bias_p_noise{1.0e-2f};  ///< 用于 IMU 加速度计偏差预测的过程噪声 (米/秒**3)
    float mage_p_noise{1.0e-3f};        ///< 用于地球磁场预测的过程噪声 (高斯/秒)
    float magb_p_noise{1.0e-4f};        ///< 用于机体磁场预测的过程噪声 (高斯/秒)
    float wind_vel_p_noise{1.0e-1f};    ///< 用于风速预测的过程噪声 (米/秒**2)
    const float wind_vel_p_noise_scaler{0.5f};  ///< 风速过程噪声随垂直速度的缩放比例
    float terrain_p_noise{5.0f};        ///< 地形偏移的过程噪声 (米/秒)
    float terrain_gradient{0.5f};       ///< 用于估计由于位置变化引起的过程噪声的地形梯度 (米/米)
    const float terrain_timeout{10.f};      ///< 在重置地形估计之前，允许无效底部距离测量的最大时间 (秒)

    // 初始化误差
    float switch_on_gyro_bias{0.1f};    ///< 开机时的 1 倍标准差 (1-sigma) 陀螺仪偏差不确定度 (弧度/秒)
    float switch_on_accel_bias{0.2f};   ///< 开机时的 1 倍标准差加速度计偏差不确定度 (米/秒**2)
    float initial_tilt_err{0.1f};       ///< 使用重力向量进行初始对齐后的 1 倍标准差倾斜误差 (弧度)
    const float initial_wind_uncertainty{1.0f}; ///< 风速的 1 倍标准差初始不确定度 (米/秒)

    // 位置和速度融合
    float gps_vel_noise{5.0e-1f};       ///< GPS 速度融合允许的最小观测噪声 (米/秒)
    float gps_pos_noise{0.5f};      ///< GPS 位置融合允许的最小观测噪声 (米)
    float pos_noaid_noise{10.0f};       ///< 无辅助位置融合的观测噪声 (米)
    float baro_noise{2.0f};         ///< 气压高度融合的观测噪声 (米)
    float baro_innov_gate{5.0f};        ///< 气压和 GPS 高度新息一致性门限大小 (标准差)
    float gps_pos_innov_gate{5.0f};     ///< GPS 水平位置新息一致性门限大小 (标准差)
    float gps_vel_innov_gate{5.0f};     ///< GPS 速度新息一致性门限大小 (标准差)
    float gnd_effect_deadzone{5.0f};    ///< 当地面效应补偿激活时，应用于负气压新息的死区大小 (米)
    float gnd_effect_max_hgt{0.5f};     ///< 气压计地面效应变得不显著的离地高度 (米)

    // 磁力计融合
    float mag_heading_noise{3.0e-1f};   ///< 用于简单航向融合的测量噪声 (弧度)
    float mag_noise{5.0e-2f};       ///< 用于 3 轴磁力计融合的测量噪声 (高斯)
    float mag_declination_deg{0.0f};    ///< 磁偏角 (度)
    float heading_innov_gate{2.6f};     ///< 航向融合新息一致性门限大小 (标准差)
    float mag_innov_gate{3.0f};     ///< 磁力计融合新息一致性门限大小 (标准差)
    int32_t mag_declination_source{7};  ///< 用于控制如何处理磁偏角数据的位掩码
    int32_t mag_fusion_type{0};     ///< 用于指定所使用的磁力计融合类型的整数
    float mag_acc_gate{0.5f};       ///< 在自动选择模式下，当机动加速度低于此值时将使用航向融合 (米/秒**2)
    float mag_yaw_rate_gate{0.25f};     ///< 模式选择逻辑使用的偏航率阈值 (弧度/秒)
    const float quat_max_variance{0.0001f}; ///< 当四元数方差之和小于此值时，将不会融合零新息的偏航测量值

    // 空速融合
    float tas_innov_gate{5.0f};     ///< 真实空速新息一致性门限大小 (标准差)
    float eas_noise{1.4f};          ///< 用于空速融合的等效空速 (EAS) 测量噪声标准差 (米/秒)
    float arsp_thr{2.0f};           ///< 空速融合阈值。值为零将停用空速融合

    // 合成侧滑角融合
    float beta_innov_gate{5.0f};        ///< 合成侧滑角新息一致性门限大小，以标准差 (STD) 计
    float beta_noise{0.3f};         ///< 合成侧滑角噪声 (弧度)
    const float beta_avg_ft_us{150000.0f};  ///< 两次合成侧滑角测量之间的平均时间 (uSec)

    // 测距仪融合
    float range_noise{0.1f};        ///< 测距仪测量值的观测噪声 (米)
    float range_innov_gate{5.0f};       ///< 测距仪融合新息一致性门限大小 (标准差)
    float rng_gnd_clearance{0.1f};      ///< 在地面上时测距仪的最小有效值 (米)
    float rng_sens_pitch{0.0f};     ///< 测距传感器的俯仰偏移角 (弧度)。当偏移量为零时，传感器沿 Z 轴指向外侧。绕 Y 轴右手定则旋转为正。
    float range_noise_scaler{0.0f};     ///< 从测距测量值到噪声的缩放因子 (米/米)
    const float vehicle_variance_scaler{0.0f};  ///< 在计算离地高度观测方差时，应用于飞行器高度方差的增益
    float max_hagl_for_range_aid{5.0f}; ///< 允许使用测距仪作为高度数据源的最大离地高度 (如果 range_aid == 1)
    float max_vel_for_range_aid{1.0f};  ///< 允许使用测距仪作为高度数据源的最大地面速度 (如果 range_aid == 1)
    int32_t range_aid{0};           ///< 如果满足某些条件，允许将主要高度源切换至测距仪
    float range_aid_innov_gate{1.0f};   ///< 用于测距辅助融合的新息一致性检查的门限大小
    float range_valid_quality_s{1.0f};  ///< 要被声明为有效，报告的测距仪信号质量需要保持非零状态的最短持续时间 (秒)
    float range_cos_max_tilt{0.7071f};  ///< 允许使用测距仪和光流数据的最大偏离垂直方向的倾斜角的余弦值

    // 视觉位置融合
    float ev_vel_innov_gate{3.0f};      ///< 视觉速度融合新息一致性门限大小 (标准差)
    float ev_pos_innov_gate{5.0f};      ///< 视觉位置融合新息一致性门限大小 (标准差)

    // 光流融合
    float flow_noise{0.15f};        ///< 光流视线 (LOS) 速率测量的观测噪声 (弧度/秒)
    float flow_noise_qual_min{0.5f};    ///< 当光流传感器质量处于最低可用水平时，光流视线速率测量的观测噪声 (弧度/秒)
    int32_t flow_qual_min{1};       ///< 传感器可接受的最低光流质量整数
    float flow_innov_gate{3.0f};        ///< 光流融合新息一致性门限大小 (标准差)

    // 这些参数控制 GPS 质量检查的严格程度，用于确定 GPS
    // 是否足够好以设定本地原点并开始辅助导航
    int32_t gps_check_mask{21};     ///< 控制使用哪些 GPS 质量检查的位掩码
    float req_hacc{5.0f};           ///< 可接受的最大水平位置误差 (米)
    float req_vacc{8.0f};           ///< 可接受的最大垂直位置误差 (米)
    float req_sacc{1.0f};           ///< 可接受的最大速度误差 (米/秒)
    int32_t req_nsats{6};           ///< 可接受的最低卫星数
    float req_pdop{2.0f};           ///< 可接受的最大位置精度衰减因子 (PDOP)
    float req_hdrift{0.3f};         ///< 可接受的最大水平漂移速度 (米/秒)
    float req_vdrift{0.5f};         ///< 可接受的最大垂直漂移速度 (米/秒)

    // 传感器在机体坐标轴下的 XYZ 偏移 (米)
    Vector3f imu_pos_body;          ///< IMU 在机体坐标系下的 xyz 位置 (米)
    Vector3f gps_pos_body;          ///< GPS 天线在机体坐标系下的 xyz 位置 (米)
    Vector3f rng_pos_body;          ///< 测距传感器在机体坐标系下的 xyz 位置 (米)
    Vector3f flow_pos_body;         ///< 测距传感器焦距点在机体坐标系下的 xyz 位置 (米)
    Vector3f ev_pos_body;           ///< VI (视觉惯性) 传感器焦距点在机体坐标系下的 xyz 位置 (米)

    // 输出互补滤波器整定
    float vel_Tau{0.25f};           ///< 速度状态校正时间常数 (1/秒)
    float pos_Tau{0.25f};           ///< 位置状态校正时间常数 (1/秒)

    // 加速度计偏差学习控制
    float acc_bias_lim{0.4f};       ///< 最大加速度偏差幅度 (米/秒**2)
    float acc_bias_learn_acc_lim{25.0f};    ///< 如果 IMU 加速度向量的大小大于此值，则禁用学习 (米/秒**2)
    float acc_bias_learn_gyr_lim{3.0f}; ///< 如果 IMU 角速率向量的大小大于此值，则禁用学习 (弧度/秒)
    float acc_bias_learn_tc{0.5f};      ///< 用于控制应用于加速度和陀螺仪幅度的衰减包络滤波器的时间常数 (秒)

    const unsigned reset_timeout_max{7000000};  ///< 在尝试将状态重置为测量值，或在数据不可用时更改 _control_status 之前，我们允许进行水平惯性航位推算的最大时间 (uSec)
    const unsigned no_aid_timeout_max{1000000}; ///< 从最后一次融合可约束水平速度漂移的测量值起，到 EKF 判定该传感器不再提供辅助功能的最大流逝时间 (uSec)

    int32_t valid_timeout_max{5000000}; ///< 估计器在将状态估计报告为无效之前，花费在惯性航位推算上的时间量 (uSec)

    // 沿机体轴的静态气压计压力位置误差系数
    float static_pressure_coef_xp {0.0f};   // (-)
    float static_pressure_coef_xn {0.0f};   // (-)
    float static_pressure_coef_yp {0.0f};   // (-)
    float static_pressure_coef_yn {0.0f};   // (-)
    float static_pressure_coef_z {0.0f};    // (-)
    // 用于校正的空速上限 (米/秒**2)
    float max_correction_airspeed {20.0f};

    // 多旋翼阻力比力融合
    float drag_noise{2.5f};         ///< 阻力比力测量的观测噪声方差 (米/秒**2)**2
    float bcoef_x{100.0f};          ///< X 轴的钝体阻力弹道系数 (kg/米**2)
    float bcoef_y{100.0f};          ///< Y 轴的钝体阻力弹道系数 (kg/米**2)
    float mcoef{0.1f};          ///< X 和 Y 轴的旋翼动量阻力系数 (1/秒)

    // 加速度计误差检测和缓解 (IMU限幅) 控制
    const float vert_innov_test_lim{3.0f};  ///< 在判定垂直速度和位置联合测试失败之前，允许的标准差数
    const int bad_acc_reset_delay_us{500000};   ///< 在重置状态之前，垂直位置和速度新息测试必须持续失败的时间 (uSec)

    // 辅助速度融合
    const float auxvel_noise{0.5f};     ///< 最小观测噪声，如果报告的噪声更大则使用报告噪声 (米/秒)
    const float auxvel_gate{5.0f};      ///< 速度融合新息一致性门限大小 (标准差)

    // 在地移动检查的控制
    float is_moving_scaler{1.0f};       ///< 用于调整在地移动检测阈值的增益缩放器。较大的值会使测试变得较不敏感。

    // 如果可能，计算合成磁力计 Z 值
    int32_t synthesize_mag_z{0};
    int32_t check_mag_strength{0};

    // 用于控制何时将偏航角重置为 EKF-GSF 偏航估计器值的参数
    float EKFGSF_tas_default{15.0f};    ///< 如果在固定翼飞行期间没有可用的空速测量，则假设的默认空速值 (米/秒)
    const unsigned EKFGSF_reset_delay{1000000}; ///< 在偏航角重置为 EKF-GSF 值之前，起飞后直接阶段主滤波器处于不良新息状态的微秒数 (uSec)
    const float EKFGSF_yaw_err_max{0.262f};     ///< 用于检查收敛性的复合偏航角 1 倍标准差不确定性阈值 (弧度)
    const unsigned EKFGSF_reset_count_limit{3}; ///< 偏航角能够重置为 EKF-GSF 偏航估计器值的最大次数
};

struct stateSample {
    Quatf  quat_nominal;    ///< 定义从机体坐标系到地球坐标系旋转的四元数
    Vector3f    vel;        ///< 地球坐标系下的 NED 速度，单位：米/秒
    Vector3f    pos;        ///< 地球坐标系下的 NED 位置，单位：米
    Vector3f    delta_ang_bias; ///< 角度增量偏差估计，单位：弧度
    Vector3f    delta_vel_bias; ///< 速度增量偏差估计，单位：米/秒
    Vector3f    mag_I;  ///< NED 地磁场，单位：高斯
    Vector3f    mag_B;  ///< 机体坐标系下的磁力计偏差估计，单位：高斯
    Vector2f    wind_vel;   ///< 地球坐标系下的水平风速，单位：米/秒
};

union fault_status_u {
    struct {
        bool bad_mag_x: 1;  ///< 0 - 如果磁力计 X 轴融合遇到数值计算错误则为 true
        bool bad_mag_y: 1;  ///< 1 - 如果磁力计 Y 轴融合遇到数值计算错误则为 true
        bool bad_mag_z: 1;  ///< 2 - 如果磁力计 Z 轴融合遇到数值计算错误则为 true
        bool bad_hdg: 1;    ///< 3 - 如果航向角融合遇到数值计算错误则为 true
        bool bad_mag_decl: 1;   ///< 4 - 如果磁偏角融合遇到数值计算错误则为 true
        bool bad_airspeed: 1;   ///< 5 - 如果空速融合遇到数值计算错误则为 true
        bool bad_sideslip: 1;   ///< 6 - 如果合成侧滑角约束融合遇到数值计算错误则为 true
        bool bad_optflow_X: 1;  ///< 7 - 如果光流 X 轴融合遇到数值计算错误则为 true
        bool bad_optflow_Y: 1;  ///< 8 - 如果光流 Y 轴融合遇到数值计算错误则为 true
        bool bad_vel_N: 1;  ///< 9 - 如果向北速度融合遇到数值计算错误则为 true
        bool bad_vel_E: 1;  ///< 10 - 如果向东速度融合遇到数值计算错误则为 true
        bool bad_vel_D: 1;  ///< 11 - 如果向下速度融合遇到数值计算错误则为 true
        bool bad_pos_N: 1;  ///< 12 - 如果向北位置融合遇到数值计算错误则为 true
        bool bad_pos_E: 1;  ///< 13 - 如果向东位置融合遇到数值计算错误则为 true
        bool bad_pos_D: 1;  ///< 14 - 如果向下位置融合遇到数值计算错误则为 true
        bool bad_acc_bias: 1;   ///< 15 - 如果检测到不良的速度增量偏差估计则为 true
        bool bad_acc_vertical: 1; ///< 16 - 如果检测到不良的垂直加速度计数据则为 true
        bool bad_acc_clipping: 1; ///< 17 - 如果速度增量数据包含限幅（非对称饱和）则为 true
    } flags;
    uint32_t value;

};

// 定义用于传达新息测试失败情况的结构体
union innovation_fault_status_u {
    struct {
        bool reject_hor_vel: 1;     ///< 0 - 如果拒绝了水平速度观测数据则为 true
        bool reject_ver_vel: 1;     ///< 1 - 如果拒绝了垂直速度观测数据则为 true
        bool reject_hor_pos: 1;     ///< 2 - 如果拒绝了水平位置观测数据则为 true
        bool reject_ver_pos: 1;     ///< 3 - 如果拒绝了垂直位置观测数据则为 true
        bool reject_mag_x: 1;       ///< 4 - 如果拒绝了 X 轴磁力计观测数据则为 true
        bool reject_mag_y: 1;       ///< 5 - 如果拒绝了 Y 轴磁力计观测数据则为 true
        bool reject_mag_z: 1;       ///< 6 - 如果拒绝了 Z 轴磁力计观测数据则为 true
        bool reject_yaw: 1;     ///< 7 - 如果拒绝了偏航角观测数据则为 true
        bool reject_airspeed: 1;    ///< 8 - 如果拒绝了空速观测数据则为 true
        bool reject_sideslip: 1;    ///< 9 - 如果拒绝了合成侧滑角观测数据则为 true
        bool reject_hagl: 1;        ///< 10 - 如果拒绝了离地高度观测数据则为 true
        bool reject_optflow_X: 1;   ///< 11 - 如果拒绝了 X 轴光流观测数据则为 true
        bool reject_optflow_Y: 1;   ///< 12 - 如果拒绝了 Y 轴光流观测数据则为 true
    } flags;
    uint16_t value;

};

// 发布各项 GPS 质量检查的状态
union gps_check_fail_status_u {
    struct {
        uint16_t fix    : 1; ///< 0 - 如果定位类型不足（没有 3D 解算）则为 true
        uint16_t nsats  : 1; ///< 1 - 如果使用的卫星数不足则为 true
        uint16_t pdop   : 1; ///< 2 - 如果位置精度衰减因子 (PDOP) 不足则为 true
        uint16_t hacc   : 1; ///< 3 - 如果报告的水平精度不足则为 true
        uint16_t vacc   : 1; ///< 4 - 如果报告的垂直精度不足则为 true
        uint16_t sacc   : 1; ///< 5 - 如果报告的速度精度不足则为 true
        uint16_t hdrift : 1; ///< 6 - 如果水平漂移过大（仅在地面静止时使用）则为 true
        uint16_t vdrift : 1; ///< 7 - 如果垂直漂移过大（仅在地面静止时使用）则为 true
        uint16_t hspeed : 1; ///< 8 - 如果水平速度过大（仅在地面静止时使用）则为 true
        uint16_t vspeed : 1; ///< 9 - 如果垂直速度误差过大则为 true
    } flags;
    uint16_t value;
};

// 包含滤波器控制状态的位掩码
union filter_control_status_u {
    struct {
        uint32_t tilt_align  : 1; ///< 0 - 如果滤波器倾角对齐完成则为 true
        uint32_t yaw_align   : 1; ///< 1 - 如果滤波器偏航角对齐完成则为 true
        uint32_t gps         : 1; ///< 2 - 如果计划进行 GPS 测量融合则为 true
        uint32_t opt_flow    : 1; ///< 3 - 如果计划进行光流测量融合则为 true
        uint32_t mag_hdg     : 1; ///< 4 - 如果计划进行简单的磁偏航航向融合则为 true
        uint32_t mag_3D      : 1; ///< 5 - 如果计划进行 3 轴磁力计测量融合则为 true
        uint32_t mag_dec     : 1; ///< 6 - 如果计划进行合成磁偏角测量融合则为 true
        uint32_t in_air      : 1; ///< 7 - 当飞行器在空中飞行时为 true
        uint32_t wind        : 1; ///< 8 - 当正在估计风速时为 true
        uint32_t baro_hgt    : 1; ///< 9 - 当融合气压高度作为主要高度参考时为 true
        uint32_t rng_hgt     : 1; ///< 10 - 当融合测距仪高度作为主要高度参考时为 true
        uint32_t gps_hgt     : 1; ///< 11 - 当融合 GPS 高度作为主要高度参考时为 true
        uint32_t ev_pos      : 1; ///< 12 - 当计划融合来自外部视觉的局部位置数据时为 true
        uint32_t ev_yaw      : 1; ///< 13 - 当计划融合来自外部视觉测量的偏航角数据时为 true
        uint32_t ev_hgt      : 1; ///< 14 - 当正在融合来自外部视觉测量的高度数据时为 true
        uint32_t fuse_beta   : 1; ///< 15 - 当正在融合合成侧滑角测量数据时为 true
        uint32_t mag_field_disturbed : 1; ///< 16 - 当磁场强度与预期不符时为 true
        uint32_t fixed_wing  : 1; ///< 17 - 当飞行器作为固定翼平台运行时为 true
        uint32_t mag_fault   : 1; ///< 18 - 当磁力计被宣告发生故障且不再使用时为 true
        uint32_t fuse_aspd   : 1; ///< 19 - 当正在融合空速测量数据时为 true
        uint32_t gnd_effect  : 1; ///< 20 - 当地面效应引起的静压上升保护处于激活状态时为 true
        uint32_t rng_stuck   : 1; ///< 21 - 当测距仪数据在 10 秒以上未能准备好，且新的测距值变化不足时为 true
        uint32_t gps_yaw     : 1; ///< 22 - 当计划融合来自 GPS 接收机的偏航角（而非地面航向）数据时为 true
        uint32_t mag_aligned_in_flight   : 1; ///< 23 - 当飞行中磁场对齐已完成时为 true
        uint32_t ev_vel      : 1; ///< 24 - 当计划融合来自外部视觉测量的局部坐标系速度数据时为 true
        uint32_t synthetic_mag_z : 1; ///< 25 - 当我们对磁力计 Z 分量使用合成测量值时为 true
        uint32_t vehicle_at_rest : 1; ///< 26 - 当飞行器处于静止状态时为 true
    } flags;
    uint32_t value;
};

 // 包含估计器求解状态的 Mavlink 位掩码
union ekf_solution_status {
    struct {
        uint16_t attitude           : 1; ///< 0 - 如果姿态估计良好则为 True
        uint16_t velocity_horiz     : 1; ///< 1 - 如果水平速度估计良好则为 True
        uint16_t velocity_vert      : 1; ///< 2 - 如果垂直速度估计良好则为 True
        uint16_t pos_horiz_rel      : 1; ///< 3 - 如果水平相对位置估计良好则为 True
        uint16_t pos_horiz_abs      : 1; ///< 4 - 如果水平绝对位置估计良好则为 True
        uint16_t pos_vert_abs       : 1; ///< 5 - 如果垂直绝对位置估计良好则为 True
        uint16_t pos_vert_agl       : 1; ///< 6 - 如果垂直位置（离地高度）估计良好则为 True
        uint16_t const_pos_mode     : 1; ///< 7 - 如果 EKF 处于恒定位置模式，并且不使用外部测量值（如 GPS 或光流）则为 True
        uint16_t pred_pos_horiz_rel : 1; ///< 8 - 如果 EKF 有足够数据进入可提供（相对）位置估计的模式则为 True
        uint16_t pred_pos_horiz_abs : 1; ///< 9 - 如果 EKF 有足够数据进入可提供（绝对）位置估计的模式则为 True
        uint16_t gps_glitch         : 1; ///< 10 - 如果 EKF 检测到 GPS 故障/干扰 (glitch) 则为 True
        uint16_t accel_error        : 1; ///< 11 - 如果 EKF 检测到不良加速度计数据则为 True
    } flags;
    uint16_t value;
};

union terrain_fusion_status_u {
    struct {
        bool range_finder: 1;   ///< 0 - 如果我们正在融合测距仪数据则为 true
        bool flow: 1;       ///< 1 - 如果我们正在融合光流数据则为 true
    } flags;
    uint8_t value;
};

// 定义用于传达信息事件的结构体
union information_event_status_u {
    struct {
        bool gps_checks_passed      : 1; ///< 0 - 当 GPS 质量检查通过时为 true
        bool reset_vel_to_gps       : 1; ///< 1 - 当速度状态重置为 GPS 测量值时为 true
        bool reset_vel_to_flow      : 1; ///< 2 - 当使用光流测量值重置速度状态时为 true
        bool reset_vel_to_vision    : 1; ///< 3 - 当速度状态重置为视觉系统测量值时为 true
        bool reset_vel_to_zero      : 1; ///< 4 - 当速度状态重置为零时为 true
        bool reset_pos_to_last_known    : 1; ///< 5 - 当位置状态重置为最后已知位置时为 true
        bool reset_pos_to_gps       : 1; ///< 6 - 当位置状态重置为 GPS 测量值时为 true
        bool reset_pos_to_vision    : 1; ///< 7 - 当位置状态重置为视觉系统测量值时为 true
        bool starting_gps_fusion    : 1; ///< 8 - 当滤波器开始使用 GPS 测量值来校正状态估计时为 true
        bool starting_vision_pos_fusion : 1; ///< 9 - 当滤波器开始使用视觉系统位置测量值来校正状态估计时为 true
        bool starting_vision_vel_fusion : 1; ///< 10 - 当滤波器开始使用视觉系统速度测量值来校正状态估计时为 true
        bool starting_vision_yaw_fusion : 1; ///< 11 - 当滤波器开始使用视觉系统偏航角测量值来校正状态估计时为 true
        bool yaw_aligned_to_imu_gps : 1; ///< 12 - 当滤波器将偏航角重置为由 IMU 和 GPS 数据推导出的估计值时为 true
    } flags;
    uint32_t value;
};

// 定义用于传达警告事件的结构体
union warning_event_status_u {
    struct {
        bool gps_quality_poor           : 1; ///< 0 - 当 GPS 未通过质量检查时为 true
        bool gps_fusion_timout          : 1; ///< 1 - 当一段显著时间内，GPS 数据未被用于校正状态估计时为 true
        bool gps_data_stopped           : 1; ///< 2 - 当 GPS 数据已停止一段显著时间时为 true
        bool gps_data_stopped_using_alternate   : 1; ///< 3 - 当 GPS 数据已停止一段显著时间，但滤波器能够使用其他数据源维持导航时为 true
        bool height_sensor_timeout      : 1; ///< 4 - 当一段显著时间内，高度传感器未被用于校正状态估计时为 true
        bool stopping_navigation        : 1; ///< 5 - 当滤波器数据不足以估计速度和位置，并回退到仅提供姿态、高度和高度率模式的操作时为 true
        bool invalid_accel_bias_cov_reset   : 1; ///< 6 - 当滤波器检测到不良加速度计偏差状态估计，并重置相应的协方差矩阵元素时为 true
        bool bad_yaw_using_gps_course       : 1; ///< 7 - 当滤波器检测到无效的偏航估计，并将偏航角重置为 GPS 地面航向时为 true
        bool stopping_mag_use           : 1; ///< 8 - 当滤波器检测到不良磁力计数据并停止进一步使用磁力计数据时为 true
        bool vision_data_stopped        : 1; ///< 9 - 当视觉系统数据已停止一段显著时间时为 true
        bool emergency_yaw_reset_mag_stopped    : 1; ///< 10 - 当滤波器检测到不良磁力计数据，已将偏航重置为其他数据源，并停止进一步使用磁力计数据时为 true
    } flags;
    uint32_t value;
};

}