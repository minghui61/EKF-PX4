#include "EKFGSF_yaw.h"
#include <cstdlib>

EKFGSF_yaw::EKFGSF_yaw()
{
	// 该标志在启动时必须为 false
	_ahrs_ekf_gsf_tilt_aligned = false;

	// 这些对象会在 initialise() 中被初始化后再供内部使用，
	// 但在此之前也可以用于日志输出
	memset(&_ahrs_ekf_gsf, 0, sizeof(_ahrs_ekf_gsf));
	memset(&_ekf_gsf, 0, sizeof(_ekf_gsf));
	_gsf_yaw = 0.0f;
	_ahrs_accel.zero();
}

void EKFGSF_yaw::update(const imuSample& imu_sample,
			bool run_EKF, 			// 在飞行或运动适合估计偏航时设为 true
			float airspeed, 			// 用于离心加速度补偿的真实空速；不需要时设为 0
			const Vector3f &imu_gyro_bias)  // 估计的陀螺仪速率偏置（rad/sec）
{
	// 复制到类变量中
	_delta_ang = imu_sample.delta_ang;
	_delta_vel = imu_sample.delta_vel;
	_delta_ang_dt = imu_sample.delta_ang_dt;
	_delta_vel_dt = imu_sample.delta_vel_dt;
	_run_ekf_gsf = run_EKF;
	_true_airspeed = airspeed;

	// 为减少振动影响，使用低通滤波器进行滤波，其时间常数为 AHRS 倾角修正时间常数的 1/10
	const float filter_coef = fminf(10.0f * _delta_vel_dt * _tilt_gain, 1.0f);
	const Vector3f accel = _delta_vel / fmaxf(_delta_vel_dt, 0.001f);
	_ahrs_accel = _ahrs_accel * (1.0f - filter_coef) + accel * filter_coef;

	// 首次初始化状态
	if (!_ahrs_ekf_gsf_tilt_aligned) {
		// 检查是否存在过大的加速度，以降低由机体运动导致的初始滚转/俯仰误差的可能性
		const float accel_norm_sq = accel.norm_squared();
		const float upper_accel_limit = CONSTANTS_ONE_G * 1.1f;
		const float lower_accel_limit = CONSTANTS_ONE_G * 0.9f;
		const bool ok_to_align = (accel_norm_sq > sq(lower_accel_limit)) && (accel_norm_sq < sq(upper_accel_limit));
		if (ok_to_align) {
			initialiseEKFGSF();
			ahrsAlignTilt();
			_ahrs_ekf_gsf_tilt_aligned = true;
		}
		return;
	}

	// 计算 AHRS 互补滤波模型所需的公共值
	_ahrs_accel_norm = _ahrs_accel.norm();

	// 每个模型的 AHRS 预测循环 - 该过程始终执行
	_ahrs_accel_fusion_gain = ahrsCalcAccelGain();
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		predictEKF(model_index);
	}

	// 只有在飞行状态下才运行 3 状态 EKF 模型，避免因操作员处理和 GPS 干扰导致估计被破坏
	if (_run_ekf_gsf && _vel_data_updated) {
		if (!_ekf_gsf_vel_fuse_started) {
			initialiseEKFGSF();
			ahrsAlignYaw();
			// 使用主滤波器中的陀螺仪偏置估计来初始化，
			// 因为绕重力方向可能存在较大的未校正速率陀螺偏置误差
			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
				_ahrs_ekf_gsf[model_index].gyro_bias = imu_gyro_bias;
			}
			_ekf_gsf_vel_fuse_started = true;
		} else {
			bool bad_update = false;
			for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
				// 后续测量作为直接状态观测值进行融合
				if (!updateEKF(model_index)) {
					bad_update = true;
				}
			}

			if (!bad_update) {
				float total_weight = 0.0f;
				// 假设正态分布，计算每个模型的权重
				const float min_weight = 1e-5f;
				uint8_t n_weight_clips = 0;
				for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
					_model_weights(model_index) = gaussianDensity(model_index) * _model_weights(model_index);
					if (_model_weights(model_index) < min_weight) {
						n_weight_clips++;
						_model_weights(model_index) = min_weight;
					}
					total_weight += _model_weights(model_index);
				}

				// 归一化权重函数
				if (n_weight_clips < N_MODELS_EKFGSF) {
					_model_weights /= total_weight;
				} else {
					// 由于创新方差过大，所有权重都塌缩，因此重置滤波器
					initialiseEKFGSF();
				}
			}
		}
	} else if (_ekf_gsf_vel_fuse_started && !_run_ekf_gsf) {
		// 等待再次飞行
		_ekf_gsf_vel_fuse_started = false;
	}

	// 计算各模型状态加权平均后的复合偏航向量。
	// 为避免角度绕圈问题，在求和前将偏航状态转换为长度等于权重的向量。
	Vector2f yaw_vector;
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		yaw_vector(0) += _model_weights(model_index) * cosf(_ekf_gsf[model_index].X(2));
		yaw_vector(1) += _model_weights(model_index) * sinf(_ekf_gsf[model_index].X(2));
	}
	_gsf_yaw = atan2f(yaw_vector(1),yaw_vector(0));

	// 根据各模型方差加权平均，计算复合偏航状态方差
	// 创新较大的模型权重更小
	_gsf_yaw_variance = 0.0f;
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index ++) {
		const float yaw_delta = wrap_pi(_ekf_gsf[model_index].X(2) - _gsf_yaw);
		_gsf_yaw_variance += _model_weights(model_index) * (_ekf_gsf[model_index].P(2,2) + yaw_delta * yaw_delta);
	}

	// 防止同一速度数据被重复使用
	_vel_data_updated = false;
}

void EKFGSF_yaw::ahrsPredict(const uint8_t model_index)
{
	// 使用简单互补滤波器为选定模型生成姿态解

	const Vector3f ang_rate = _delta_ang / fmaxf(_delta_ang_dt, 0.001f) - _ahrs_ekf_gsf[model_index].gyro_bias;

	const Dcmf R_to_body = _ahrs_ekf_gsf[model_index].R.transpose();
	const Vector3f gravity_direction_bf = R_to_body.col(2);

	// 使用加速度计数据进行角速率修正，并随着加速度大小偏离 1 g 而减小修正量（减少机体被抬起和移动时的漂移）。
	// 在固定翼飞行中，假设协调转弯且 X 轴向前，补偿离心加速度
	Vector3f tilt_correction;
	if (_ahrs_accel_fusion_gain > 0.0f) {

		Vector3f accel = _ahrs_accel;

		if (_true_airspeed > FLT_EPSILON) {
			// 假设 X 轴与空速方向对齐，计算体坐标系中的离心加速度
			// 使用机体角速率与机体坐标系空速向量的叉积
			const Vector3f centripetal_accel_bf = Vector3f(0.0f, _true_airspeed * ang_rate(2), - _true_airspeed * ang_rate(1));

			// 对测量到的加速度进行离心加速度修正
			accel -= centripetal_accel_bf;
		}

		tilt_correction = (gravity_direction_bf % accel) * _ahrs_accel_fusion_gain / _ahrs_accel_norm;

	}

	// 陀螺仪偏置估计
	constexpr float gyro_bias_limit = 0.05f;
	const float spinRate = ang_rate.length();
	if (spinRate < 0.175f) {
		_ahrs_ekf_gsf[model_index].gyro_bias -= tilt_correction * (_gyro_bias_gain * _delta_ang_dt);
		_ahrs_ekf_gsf[model_index].gyro_bias = matrix::constrain(_ahrs_ekf_gsf[model_index].gyro_bias, -gyro_bias_limit, gyro_bias_limit);
	}

	// 前一帧到当前帧的角增量
	const Vector3f delta_angle_corrected = _delta_ang + (tilt_correction - _ahrs_ekf_gsf[model_index].gyro_bias) * _delta_ang_dt;

	// 将角增量应用到旋转矩阵
	_ahrs_ekf_gsf[model_index].R = ahrsPredictRotMat(_ahrs_ekf_gsf[model_index].R, delta_angle_corrected);

}

void EKFGSF_yaw::ahrsAlignTilt()
{
	// 旋转矩阵直接由加速度测量值构造，并对所有模型都相同，因此只需计算一次。假设为：
	// 1) 偏航角为零 —— 在速度融合开始时，再为每个模型分别对齐偏航。
	// 2) 车辆不加速，因此所有测量到的加速度都来自重力。

	// 计算地球坐标系下向下轴单位向量旋转到机体坐标系中的表示
	const Vector3f down_in_bf = -_delta_vel.normalized();

	// 计算地球坐标系下北轴单位向量旋转到机体坐标系中的表示，且与 'down_in_bf' 正交
	const Vector3f i_vec_bf(1.0f,0.0f,0.0f);
	Vector3f north_in_bf = i_vec_bf - down_in_bf * (i_vec_bf.dot(down_in_bf));
	north_in_bf.normalize();

	// 计算地球坐标系下东轴单位向量旋转到机体坐标系中的表示，且与 'down_in_bf' 和 'north_in_bf' 正交
	const Vector3f east_in_bf = down_in_bf % north_in_bf;

	// 旋转矩阵中，地球坐标系到机体坐标系的每一列表示对应地球坐标系单位向量旋转到机体坐标系中的投影，
	// 例如 'north_in_bf' 会作为第一列。
	// 但我们需要的是机体坐标系到地球坐标系的旋转矩阵，因此将旋转到机体坐标系中的地球轴向量
	// 复制到对应的行中即可。
	Dcmf R;
	R.setRow(0, north_in_bf);
	R.setRow(1, east_in_bf);
	R.setRow(2, down_in_bf);

	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		_ahrs_ekf_gsf[model_index].R = R;
	}
}

void EKFGSF_yaw::ahrsAlignYaw()
{
	// 对每个模型对齐偏航角
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		Dcmf& R = _ahrs_ekf_gsf[model_index].R;
		const float yaw = wrap_pi(_ekf_gsf[model_index].X(2));
		R = updateYawInRotMat(yaw, R);

		_ahrs_ekf_gsf[model_index].aligned = true;
	}
}

void EKFGSF_yaw::predictEKF(const uint8_t model_index)
{
	// 使用 IMU 数据生成姿态参考
	ahrsPredict(model_index);

	// 仅在存在规律速度观测时才启动 EKF 部分算法
	if (!_ekf_gsf_vel_fuse_started) {
		return;
	}

	// 使用投影到水平面的方式计算偏航状态，以避免万向节锁
	const Dcmf& R = _ahrs_ekf_gsf[model_index].R;
	_ekf_gsf[model_index].X(2) = shouldUse321RotationSequence(R) ?
					getEuler321Yaw(R) :
					getEuler312Yaw(R);

	// 计算水平前-右坐标系中的增量速度
	const Vector3f del_vel_NED = _ahrs_ekf_gsf[model_index].R * _delta_vel;
	const float cos_yaw = cosf(_ekf_gsf[model_index].X(2));
	const float sin_yaw = sinf(_ekf_gsf[model_index].X(2));
	const float dvx =   del_vel_NED(0) * cos_yaw + del_vel_NED(1) * sin_yaw;
	const float dvy = - del_vel_NED(0) * sin_yaw + del_vel_NED(1) * cos_yaw;

	// 在地球坐标系中累积增量速度：
	_ekf_gsf[model_index].X(0) += del_vel_NED(0);
	_ekf_gsf[model_index].X(1) += del_vel_NED(1);

	// 预测协方差 - 方程由 EKF/python/gsf_ekf_yaw_estimator/main.py 生成

	// 为了提高可读性，复制局部短变量名
	const float &P00 = _ekf_gsf[model_index].P(0,0);
	const float &P01 = _ekf_gsf[model_index].P(0,1);
	const float &P02 = _ekf_gsf[model_index].P(0,2);
	const float &P11 = _ekf_gsf[model_index].P(1,1);
	const float &P12 = _ekf_gsf[model_index].P(1,2);
	const float &P22 = _ekf_gsf[model_index].P(2,2);
	const float &psi = _ekf_gsf[model_index].X(2);

	// 使用固定值作为增量速度和增量角度过程噪声方差
	const float dvxVar = sq(_accel_noise * _delta_vel_dt); // 前向增量速度方差 - (m/s)^2
	const float dvyVar = dvxVar; // 右向增量速度方差 - (m/s)^2
	const float dazVar = sq(_gyro_noise * _delta_ang_dt); // 偏航角增量方差 - rad^2

	// 来自 SymPy 脚本 src/lib/ecl/EKF/python/ekf_derivation/main.py 的优化自动生成代码
	const float S0 = cosf(psi);
	const float S1 = ecl::powf(S0, 2);
	const float S2 = sinf(psi);
	const float S3 = ecl::powf(S2, 2);
	const float S4 = S0*dvy + S2*dvx;
	const float S5 = P02 - P22*S4;
	const float S6 = S0*dvx - S2*dvy;
	const float S7 = S0*S2;
	const float S8 = P01 + S7*dvxVar - S7*dvyVar;
	const float S9 = P12 + P22*S6;

	_ekf_gsf[model_index].P(0,0) = P00 - P02*S4 + S1*dvxVar + S3*dvyVar - S4*S5;
	_ekf_gsf[model_index].P(0,1) = -P12*S4 + S5*S6 + S8;
	_ekf_gsf[model_index].P(1,1) = P11 + P12*S6 + S1*dvyVar + S3*dvxVar + S6*S9;
	_ekf_gsf[model_index].P(0,2) = S5;
	_ekf_gsf[model_index].P(1,2) = S9;
	_ekf_gsf[model_index].P(2,2) = P22 + dazVar;

	// 协方差矩阵对称，因此将上半部分复制到下半部分
	_ekf_gsf[model_index].P(1,0) = _ekf_gsf[model_index].P(0,1);
	_ekf_gsf[model_index].P(2,0) = _ekf_gsf[model_index].P(0,2);
	_ekf_gsf[model_index].P(2,1) = _ekf_gsf[model_index].P(1,2);

	// 限制方差
	const float min_var = 1e-6f;
	for (unsigned index = 0; index < 3; index++) {
		_ekf_gsf[model_index].P(index,index) = fmaxf(_ekf_gsf[model_index].P(index,index),min_var);
	}
}

// 使用速度测量更新指定模型索引下的 EKF 状态和协方差
bool EKFGSF_yaw::updateEKF(const uint8_t model_index)
{
	// 使用 GPS 提供的精度估计设定观测方差，并应用合理性最小值检查
	const float velObsVar = sq(fmaxf(_vel_accuracy, 0.01f));

	// 计算速度观测创新
	_ekf_gsf[model_index].innov(0) = _ekf_gsf[model_index].X(0) - _vel_NE(0);
	_ekf_gsf[model_index].innov(1) = _ekf_gsf[model_index].X(1) - _vel_NE(1);

	// 使用临时变量保存协方差元素，以减少自动生成代码表达式的冗长程度
	const float &P00 = _ekf_gsf[model_index].P(0,0);
	const float &P01 = _ekf_gsf[model_index].P(0,1);
	const float &P02 = _ekf_gsf[model_index].P(0,2);
	const float &P11 = _ekf_gsf[model_index].P(1,1);
	const float &P12 = _ekf_gsf[model_index].P(1,2);
	const float &P22 = _ekf_gsf[model_index].P(2,2);

	// 来自 SymPy 脚本 src/lib/ecl/EKF/python/ekf_derivation/main.py 的优化自动生成代码
	const float t0 = ecl::powf(P01, 2);
	const float t1 = -t0;
	const float t2 = P00*P11 + P00*velObsVar + P11*velObsVar + t1 + ecl::powf(velObsVar, 2);
	if (fabsf(t2) < 1e-6f) {
		return false;
	}
	const float t3 = 1.0F/t2;
	const float t4 = P11 + velObsVar;
	const float t5 = P01*t3;
	const float t6 = -t5;
	const float t7 = P00 + velObsVar;
	const float t8 = P00*t4 + t1;
	const float t9 = t5*velObsVar;
	const float t10 = P11*t7;
	const float t11 = t1 + t10;
	const float t12 = P01*P12;
	const float t13 = P02*t4;
	const float t14 = P01*P02;
	const float t15 = P12*t7;
	const float t16 = t0*velObsVar;
	const float t17 = ecl::powf(t2, -2);
	const float t18 = t4*velObsVar + t8;
	const float t19 = t17*t18;
	const float t20 = t17*(t16 + t7*t8);
	const float t21 = t0 - t10;
	const float t22 = t17*t21;
	const float t23 = t14 - t15;
	const float t24 = P01*t23;
	const float t25 = t12 - t13;
	const float t26 = t16 - t21*t4;
	const float t27 = t17*t26;
	const float t28 = t11 + t7*velObsVar;
	const float t30 = t17*t28;
	const float t31 = P01*t25;
	const float t32 = t23*t4 + t31;
	const float t33 = t17*t32;
	const float t35 = t24 + t25*t7;
	const float t36 = t17*t35;

	_ekf_gsf[model_index].S_det_inverse = t3;

	_ekf_gsf[model_index].S_inverse(0,0) = t3*t4;
	_ekf_gsf[model_index].S_inverse(0,1) = t6;
	_ekf_gsf[model_index].S_inverse(1,1) = t3*t7;
	_ekf_gsf[model_index].S_inverse(1,0) = _ekf_gsf[model_index].S_inverse(0,1);

	matrix::Matrix<float, 3, 2> K;
	K(0,0) = t3*t8;
	K(1,0) = t9;
	K(2,0) = t3*(-t12 + t13);
	K(0,1) = t9;
	K(1,1) = t11*t3;
	K(2,1) = t3*(-t14 + t15);

	_ekf_gsf[model_index].P(0,0) = P00 - t16*t19 - t20*t8;
	_ekf_gsf[model_index].P(0,1) = P01*(t18*t22 - t20*velObsVar + 1);
	_ekf_gsf[model_index].P(1,1) = P11 - t16*t30 + t22*t26;
	_ekf_gsf[model_index].P(0,2) = P02 + t19*t24 + t20*t25;
	_ekf_gsf[model_index].P(1,2) = P12 + t23*t27 + t30*t31;
	_ekf_gsf[model_index].P(2,2) = P22 - t23*t33 - t25*t36;
	_ekf_gsf[model_index].P(1,0) = _ekf_gsf[model_index].P(0,1);
	_ekf_gsf[model_index].P(2,0) = _ekf_gsf[model_index].P(0,2);
	_ekf_gsf[model_index].P(2,1) = _ekf_gsf[model_index].P(1,2);

	// 限制方差
	const float min_var = 1e-6f;
	for (unsigned index = 0; index < 3; index++) {
		_ekf_gsf[model_index].P(index,index) = fmaxf(_ekf_gsf[model_index].P(index,index),min_var);
	}

	// 检验比率 = transpose(innovation) * inverse(innovation variance) * innovation = [1x2] * [2,2] * [2,1] = [1,1]
	const float test_ratio = _ekf_gsf[model_index].innov * (_ekf_gsf[model_index].S_inverse * _ekf_gsf[model_index].innov);

	// 执行卡方创新一致性检验，并计算压缩比例因子，
	// 以限制创新量不超过 5σ
	// 如果检验比率大于 25（5σ），则缩短创新向量长度以将其限制在 5σ 内
	// 这可防止大测量尖峰干扰
	const float innov_comp_scale_factor = test_ratio > 25.f ? sqrtf(25.0f / test_ratio) : 1.f;

	// 修正状态向量并记录偏航角变化
	const float oldYaw = _ekf_gsf[model_index].X(2);

	_ekf_gsf[model_index].X -= (K * _ekf_gsf[model_index].innov) * innov_comp_scale_factor;

	const float yawDelta = _ekf_gsf[model_index].X(2) - oldYaw;

	// 将偏航角变化应用到 AHRS
	// 利用偏航旋转矩阵的稀疏性
	const float cosYaw = cosf(yawDelta);
	const float sinYaw = sinf(yawDelta);
	const float R_prev00 = _ahrs_ekf_gsf[model_index].R(0, 0);
	const float R_prev01 = _ahrs_ekf_gsf[model_index].R(0, 1);
	const float R_prev02 = _ahrs_ekf_gsf[model_index].R(0, 2);

	_ahrs_ekf_gsf[model_index].R(0, 0) = R_prev00 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 0) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(0, 1) = R_prev01 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 1) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(0, 2) = R_prev02 * cosYaw - _ahrs_ekf_gsf[model_index].R(1, 2) * sinYaw;
	_ahrs_ekf_gsf[model_index].R(1, 0) = R_prev00 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 0) * cosYaw;
	_ahrs_ekf_gsf[model_index].R(1, 1) = R_prev01 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 1) * cosYaw;
	_ahrs_ekf_gsf[model_index].R(1, 2) = R_prev02 * sinYaw + _ahrs_ekf_gsf[model_index].R(1, 2) * cosYaw;

	return true;
}

void EKFGSF_yaw::initialiseEKFGSF()
{
	_gsf_yaw = 0.0f;
	_ekf_gsf_vel_fuse_started = false;
	_gsf_yaw_variance = _m_pi2 * _m_pi2;
	_model_weights.setAll(1.0f / (float)N_MODELS_EKFGSF);  // 所有滤波器模型初始权重相同

	memset(&_ekf_gsf, 0, sizeof(_ekf_gsf));
	const float yaw_increment = 2.0f * _m_pi / (float)N_MODELS_EKFGSF;
	for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
		// 在 +-Pi 区间内均匀分布初始偏航估计
		_ekf_gsf[model_index].X(2) = -_m_pi + (0.5f * yaw_increment) + ((float)model_index * yaw_increment);

		// 采用上一测量值中的速度状态和对应方差
		_ekf_gsf[model_index].X(0) = _vel_NE(0);
		_ekf_gsf[model_index].X(1) = _vel_NE(1);
		_ekf_gsf[model_index].P(0,0) = sq(_vel_accuracy);
		_ekf_gsf[model_index].P(1,1) = _ekf_gsf[model_index].P(0,0);

		// 使用半个偏航间隔作为偏航不确定度
		_ekf_gsf[model_index].P(2,2) = sq(0.5f * yaw_increment);
	}
}

float EKFGSF_yaw::gaussianDensity(const uint8_t model_index) const
{
	// 计算 transpose(innovation) * inv(S) * innovation
	const float normDist = _ekf_gsf[model_index].innov.dot(_ekf_gsf[model_index].S_inverse * _ekf_gsf[model_index].innov);

	return _m_2pi_inv * sqrtf(_ekf_gsf[model_index].S_det_inverse) * expf(-0.5f * normDist);
}

bool EKFGSF_yaw::getLogData(float *yaw_composite, float *yaw_variance, float yaw[N_MODELS_EKFGSF], float innov_VN[N_MODELS_EKFGSF], float innov_VE[N_MODELS_EKFGSF], float weight[N_MODELS_EKFGSF]) const
{
	if (_ekf_gsf_vel_fuse_started) {
		*yaw_composite = _gsf_yaw;
		*yaw_variance = _gsf_yaw_variance;
		for (uint8_t model_index = 0; model_index < N_MODELS_EKFGSF; model_index++) {
			yaw[model_index] = _ekf_gsf[model_index].X(2);
			innov_VN[model_index] = _ekf_gsf[model_index].innov(0);
			innov_VE[model_index] = _ekf_gsf[model_index].innov(1);
			weight[model_index] = _model_weights(model_index);
		}
		return true;
	}
	return false;
}

float EKFGSF_yaw::ahrsCalcAccelGain() const
{
	// 使用连续函数计算加速度融合增益，该函数在 1g 处为 1，在最小/最大 g 值处为 0。
	// 当以固定翼飞机方式飞行时，允许更大的加速度，因为离心加速度修正会带来更高且更持续的 g 力。
	// 使用二次函数代替线性函数，以防止围绕 1g 的振动降低倾角修正效果。
	// 参见 https://www.desmos.com/calculator/dbqbxvnwfg

	float attenuation = 2.f;
	const bool centripetal_accel_compensation_enabled = (_true_airspeed > FLT_EPSILON);

	if (centripetal_accel_compensation_enabled
	    && _ahrs_accel_norm > CONSTANTS_ONE_G) {
		attenuation = 1.f;
	}

	const float delta_accel_g = (_ahrs_accel_norm - CONSTANTS_ONE_G) / CONSTANTS_ONE_G;
	return _tilt_gain * sq(1.f - math::min(attenuation * fabsf(delta_accel_g), 1.f));
}

Matrix3f EKFGSF_yaw::ahrsPredictRotMat(const Matrix3f &R, const Vector3f &g)
{
	Matrix3f ret = R;
	ret(0,0) += R(0,1) * g(2) - R(0,2) * g(1);
	ret(0,1) += R(0,2) * g(0) - R(0,0) * g(2);
	ret(0,2) += R(0,0) * g(1) - R(0,1) * g(0);
	ret(1,0) += R(1,1) * g(2) - R(1,2) * g(1);
	ret(1,1) += R(1,2) * g(0) - R(1,0) * g(2);
	ret(1,2) += R(1,0) * g(1) - R(1,1) * g(0);
	ret(2,0) += R(2,1) * g(2) - R(2,2) * g(1);
	ret(2,1) += R(2,2) * g(0) - R(2,0) * g(2);
	ret(2,2) += R(2,0) * g(1) - R(2,1) * g(0);

	// 重新归一化各行
	for (uint8_t r = 0; r < 3; r++) {
		const float rowLengthSq = ret.row(r).norm_squared();
		if (rowLengthSq > FLT_EPSILON) {
			// 利用行长度接近 1.0 的特点，使用线性近似来估计 inverse sqrt
			const float rowLengthInv = 1.5f - 0.5f * rowLengthSq;
			ret.row(r) *=  rowLengthInv;
		}
        }

	return ret;
}

bool EKFGSF_yaw::getYawData(float *yaw, float *yaw_variance) const
{
	if(_ekf_gsf_vel_fuse_started) {
		*yaw = _gsf_yaw;
		*yaw_variance = _gsf_yaw_variance;
		return true;
	}
	return false;
}

void EKFGSF_yaw::setVelocity(const Vector2f &velocity, float accuracy)
{
	_vel_NE = velocity;
	_vel_accuracy = accuracy;
	_vel_data_updated = true;
}
