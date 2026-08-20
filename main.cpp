#include <stdio.h>
#include <stdint.h>
#include "ekf_wrapper.h"

int main() {
    printf("--- 开始 EKF 动态姿态解算测试 ---\n");
    nav_ekf_init();

    // 关键：时间戳从 1 秒开始，避开系统判定 0 为无效数据
    uint64_t time_us = 1000000; 
    
    // 模拟运行 20 秒 (20s / 0.005s = 4000 次循环)
    for(int i = 0; i <= 4000; i++) {
        time_us += 5000; 
        
        nav_ekf_set_dummy_imu(time_us);
        
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        nav_ekf_update(&roll, &pitch, &yaw);
        
        // 每 1 秒打印一次结果，观察姿态变化
        if (i % 200 == 0) {
            float time_sec = time_us / 1000000.0f;
            printf("Time: %5.1f s | Roll: %7.4f, Pitch: %7.4f, Yaw: %7.4f\n", 
                   time_sec, roll, pitch, yaw);
        }
    }
    
    printf("--- 测试运行结束 ---\n");
    return 0;
}
