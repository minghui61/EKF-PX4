#include "imu_down_sampler.hpp"

ImuDownSampler::ImuDownSampler(float target_dt_sec) : _target_dt{target_dt_sec} { reset(); }

// 累积 IMU 样本，直到达到目标时间步长
// 假设陀螺仪的 dt 接近加速度计的 dt
// 若达到目标时间步长则返回 true
/* 
在实际的嵌入式开发中，比如当你通过高速 SPI 接口去读取一颗像 LSM6DSV16X 这样的高性能六轴传感器时，
它的内部 FIFO 可能会以 1000Hz 甚至最高 8000Hz 的频率疯狂吐出数据。
哪怕你的主控是一颗运行频率极高的 Cortex-M7 级别芯片（比如 GD32H759），
如果让 EKF 算法在几千赫兹的频率下去做 15X15 的全状态协方差矩阵乘法和求逆，CPU 也会瞬间算力透支。
所以，我们需要将 8000Hz 的底层数据，“无损压缩”成 200Hz 左右的数据喂给 EKF。
*/
bool ImuDownSampler::update(const imuSample &imu_sample_new)
{
	if (_do_reset) {
		reset();
	}

	// 累积时间增量
	_imu_down_sampled.delta_ang_dt += imu_sample_new.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu_sample_new.delta_vel_dt;
	_imu_down_sampled.time_us = imu_sample_new.time_us;
	_imu_down_sampled.delta_vel_clipping[0] += imu_sample_new.delta_vel_clipping[0];
	_imu_down_sampled.delta_vel_clipping[1] += imu_sample_new.delta_vel_clipping[1];
	_imu_down_sampled.delta_vel_clipping[2] += imu_sample_new.delta_vel_clipping[2];

	// 使用四元数累积角增量数据
	// 该四元数表示从累积周期起点到终点的旋转
	const Quatf delta_q(AxisAnglef(imu_sample_new.delta_ang));
	_delta_angle_accumulated = _delta_angle_accumulated * delta_q;
	_delta_angle_accumulated.normalize();

	// 每次都将累积的增量速度数据旋转到更新后的旋转坐标系中
	const Dcmf delta_R(delta_q.inversed());
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel;

	// 在更新后的旋转坐标系中累积最新的增量速度数据
	// 假设有效采样时间位于上一帧与当前旋转坐标系之间的中点
	_imu_down_sampled.delta_vel += (imu_sample_new.delta_vel + delta_R * imu_sample_new.delta_vel) * 0.5f;

	// 检查滤波器预测步之间的目标时间间隔是否已被超过
	if (_imu_down_sampled.delta_ang_dt >= _target_dt - _imu_collection_time_adj) {
		// 累加 IMU 采样时间前移量，以满足平均 EKF 更新速率要求
		_imu_collection_time_adj += 0.01f * (_imu_down_sampled.delta_ang_dt - _target_dt);
		_imu_collection_time_adj = math::constrain(_imu_collection_time_adj, -0.5f * _target_dt,
							   0.5f * _target_dt);

		_imu_down_sampled.delta_ang = AxisAnglef(_delta_angle_accumulated);

		return true;

	} else {

		return false;
	}
}

void ImuDownSampler::reset()
{
	_imu_down_sampled.delta_ang.setZero();
	_imu_down_sampled.delta_vel.setZero();
	_imu_down_sampled.delta_ang_dt = 0.0f;
	_imu_down_sampled.delta_vel_dt = 0.0f;
	_imu_down_sampled.delta_vel_clipping[0] = false;
	_imu_down_sampled.delta_vel_clipping[1] = false;
	_imu_down_sampled.delta_vel_clipping[2] = false;
	_delta_angle_accumulated.setIdentity();
	_do_reset = false;
}
