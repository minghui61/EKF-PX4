#include "ekf_wrapper.h"
#include "EKF/ekf.h"
#include "matrix/math.hpp"
#include <stdlib.h>
#include <math.h>

static Ekf _ekf;

void nav_ekf_init(void) {
    _ekf.init(0); 
}

void nav_ekf_set_dummy_imu(uint64_t time_us) {
    float dt = 0.005f; // 200Hz 
    float time_sec = time_us / 1000000.0f;
    
    // --- 1. 虚拟磁力计 (20Hz) ---
    if (time_us % 50000 == 0) {
        magSample mag_sample = {};
        mag_sample.time_us = time_us;
        mag_sample.mag = matrix::Vector3f(0.2f, 0.0f, 0.4f);
        _ekf.setMagData(mag_sample);
    }

    // --- 2. 虚拟气压计 (20Hz) [解锁状态机的最关键一步！] ---
    if (time_us % 50000 == 0) {
        baroSample baro_sample = {};
        baro_sample.time_us = time_us;
        baro_sample.hgt = 0.0f; // 贴地高度
        _ekf.setBaroData(baro_sample);
    }

    // --- 3. 虚拟 IMU 数据 (200Hz) ---
    imuSample imu_sample = {};
    imu_sample.time_us = time_us;
    
    // 降低一点噪声，避免掩盖真实的运动
    float noise_gyr = ((rand() % 100) / 100.0f - 0.5f) * 0.001f;
    float noise_acc = ((rand() % 100) / 100.0f - 0.5f) * 0.01f;

    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float ax = 0.0f, ay = 0.0f, az = -9.81f; 

    // 剧烈机动：延迟到第 10 秒开始抬头，确保 EKF 已经完成了开机校准
    // 【机体绕 Y 轴（通常是右翼方向）以 0.5 rad/s（约 28.6°/s）的角速度旋转。持续 3 秒，总共应该抬头约 1.5rad（大概 86°）。】
    if (time_sec >= 10.0f && time_sec < 13.0f) {
        gy = 0.5f;  // 0.5 rad/s 的抬头角速度
    }

    // 计算积分
    imu_sample.delta_ang(0) = (gx + noise_gyr) * dt;
    imu_sample.delta_ang(1) = (gy + noise_gyr) * dt;
    imu_sample.delta_ang(2) = (gz + noise_gyr) * dt;
    
    imu_sample.delta_vel(0) = (ax + noise_acc) * dt;
    imu_sample.delta_vel(1) = (ay + noise_acc) * dt;
    imu_sample.delta_vel(2) = (az + noise_acc) * dt;
    
    imu_sample.delta_ang_dt = dt;
    imu_sample.delta_vel_dt = dt;

    _ekf.setIMUData(imu_sample);
}

void nav_ekf_update(float* roll, float* pitch, float* yaw) {
    _ekf.update();
    matrix::Quaternion<float> q = _ekf.getQuaternion();
    matrix::Eulerf euler(q);
    *roll  = euler.phi();
    *pitch = euler.theta();
    *yaw   = euler.psi();
}
