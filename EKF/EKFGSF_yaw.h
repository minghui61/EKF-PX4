#pragma once

#include <geo/geo.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

#include "common.h"
#include "utils.hpp"

using matrix::AxisAnglef;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Matrix3f;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using matrix::wrap_pi;

static constexpr uint8_t N_MODELS_EKFGSF = 5;

// 所需数学常量
static constexpr float _m_2pi_inv = 0.159154943f;
static constexpr float _m_pi = 3.14159265f;
static constexpr float _m_pi2 = 1.57079632f;

using namespace estimator;

class EKFGSF_yaw
{
public:
    	EKFGSF_yaw();

    	// 更新滤波器状态 - 只要有新的 IMU 数据可用就应调用
	void update(const imuSample &imu_sample,
			bool run_EKF,  			// 当飞行或运动适合进行偏航估计时设为 true
			float airspeed,			// 用于离心加速度补偿的真实空速 - 不需要时设为 0
			const Vector3f &imu_gyro_bias); // 估计的陀螺仪偏置（rad/sec）

	void setVelocity(const Vector2f &velocity, // NE 速度测量值（m/s）
			float accuracy);	   // 速度测量的 1-sigma 精度（m/s）

	// 获取用于日志的解数据
	bool getLogData(float *yaw_composite,
			float *yaw_composite_variance,
			float yaw[N_MODELS_EKFGSF],
			float innov_VN[N_MODELS_EKFGSF],
			float innov_VE[N_MODELS_EKFGSF],
			float weight[N_MODELS_EKFGSF]) const;

    	// 获取偏航估计及其对应方差
    	// 若没有可用偏航估计，则返回 false
    	bool getYawData(float *yaw, float *yaw_variance) const;

private:

	// 参数 - 这些可以做成可调参数
	const float _gyro_noise{1.0e-1f}; 	// 协方差预测中使用的偏航率噪声（rad/sec）
	const float _accel_noise{2.0f};		// 协方差预测中使用的水平加速度噪声（m/sec**2）
	const float _tilt_gain{0.2f};		// 互补滤波中倾斜误差到陀螺仪修正的增益（1/sec）
	const float _gyro_bias_gain{0.04f};	// 互补滤波中陀螺仪修正积分项所应用的增益（1/sec）

	// N_MODELS_EKFGSF AHRS 互补滤波器组中使用的声明

	Vector3f _delta_ang{};	// IMU 增量角（rad）
	Vector3f _delta_vel{};	// IMU 增量速度（m/s）
	float _delta_ang_dt{};	// _delta_ang 的积分时间间隔（sec）
	float _delta_vel_dt{};	// _delta_vel 的积分时间间隔（sec）
	float _true_airspeed{};	// 用于离心加速度补偿的真实空速（m/s）

	struct _ahrs_ekf_gsf_struct{
		Dcmf R;			// 将向量从机体坐标系旋转到地球坐标系的矩阵
		Vector3f gyro_bias;	// 由四元数计算学习到并使用的陀螺仪偏置
		bool aligned;		// 当 AHRS 已完成对齐时为 true
		float vel_NE[2];	// 来自最近一次 GPS 测量的 NE 速度向量（m/s）
		bool fuse_gps;		// 该帧上是否应融合 GPS 为 true
		float accel_dt;		// 生成 _simple_accel_FR 数据时使用的时间步长（sec）
	} _ahrs_ekf_gsf[N_MODELS_EKFGSF]{};

	bool _ahrs_ekf_gsf_tilt_aligned{};	// 初始倾斜对齐已计算完成时为 true
	float _ahrs_accel_fusion_gain{};	// AHRS 计算中，来自加速度矢量倾斜误差到陀螺仪修正的增益
	Vector3f _ahrs_accel{};			// AHRS 计算中使用的低通滤波后机体坐标系比力矢量（m/s/s）
	float _ahrs_accel_norm{};		// AHRS 计算中使用的 _ahrs_accel 比力矢量长度（m/s/s）

	// 计算重力矢量失准到倾斜修正的增益，并供所有 AHRS 滤波器共用
	float ahrsCalcAccelGain() const;

	// 使用 IMU 和可选的真实空速数据更新指定 AHRS 旋转矩阵
	void ahrsPredict(const uint8_t model_index);

	// 使用 IMU 增量速度矢量对齐所有 AHRS 的横滚和俯仰姿态
	void ahrsAlignTilt();

	// 将所有 AHRS 的偏航姿态对齐到初始值
	void ahrsAlignYaw();

	// 在机体到地球坐标系旋转矩阵上，高效传播机体坐标系中的增量角
	Matrix3f ahrsPredictRotMat(const Matrix3f &R, const Vector3f &g);

	// N_MODELS_EKFGSF EKF 组中使用的声明

	struct _ekf_gsf_struct{
		matrix::Vector3f X; 				// Vel North (m/s),  Vel East (m/s), yaw (rad)s
		matrix::SquareMatrix<float, 3> P; 		// 协方差矩阵
		matrix::SquareMatrix<float, 2> S_inverse;	// 创新协方差矩阵逆
		float S_det_inverse; 				// 创新协方差矩阵行列式逆
		matrix::Vector2f innov; 			// 速度 N,E 创新量（m/s）
	} _ekf_gsf[N_MODELS_EKFGSF]{};

	bool _vel_data_updated{};	// 当速度数据已更新时为 true
	bool _run_ekf_gsf{};		// 当运行条件适合运行 GSF 和 EKF 模型并融合速度数据时为 true
	Vector2f _vel_NE{};        // NE 速度观测值（m/s）
	float _vel_accuracy{};     // 速度观测值的 1-sigma 精度（m/s）
	bool _ekf_gsf_vel_fuse_started{}; // 当 EKF 开始融合速度数据、预测与更新处理已激活时为 true

	// 初始化 GSF 和 EKF 滤波器的状态及协方差数据
	void initialiseEKFGSF();

	// 使用惯性数据预测指定 EKF 的状态和协方差
	void predictEKF(const uint8_t model_index);

	// 使用 NE 速度测量更新指定 EKF 的状态和协方差
	// 若更新失败则返回 false
	bool updateEKF(const uint8_t model_index);

	inline float sq(float x) const { return x * x; };

	// 用于组合各个 EKF 偏航估计的高斯和滤波器（GSF）所使用的声明

	matrix::Vector<float, N_MODELS_EKFGSF> _model_weights{};
	float _gsf_yaw{}; 		// 偏航估计值（rad）
	float _gsf_yaw_variance{}; 	// 偏航估计方差（rad^2）

	// 在假设高斯误差分布下，返回指定 EKF 状态估计的概率
	float gaussianDensity(const uint8_t model_index) const;
};
