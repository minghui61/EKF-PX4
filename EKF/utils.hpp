#include <matrix/math.hpp>

#pragma once

// 返回两个浮点数的平方，用于自动编码部分
static constexpr float sq(float var) { return var * var; }

// 将从坐标系 1 到坐标系 2 的 Tait-Bryan 312 旋转序列
// 转换为对应的、从坐标系 2 到坐标系 1 旋转的旋转矩阵
// rot312(0) - 第一次旋转是在 Z 轴上进行右手旋转（rad）
// rot312(1) - 第二次旋转是在 X 轴上进行右手旋转（rad）
// rot312(2) - 第三次旋转是在 Y 轴上进行右手旋转（rad）
// 参见 http://www.atacolorado.com/eulersequences.doc
matrix::Dcmf taitBryan312ToRotMat(const matrix::Vector3f &rot312);

// 使用 Kahan 求和算法计算 "sum_previous" 和 "input" 的和。
// 该函数要求调用方负责保留 "accumulator" 的副本，并在下一次迭代中传回该值。
// 参考: https://en.wikipedia.org/wiki/Kahan_summation_algorithm
float kahanSummation(float sum_previous, float input, float &accumulator);

// 计算四元数旋转对应的逆旋转矩阵
// 其结果与数学库中四元数到 Dcmf 算子生成的逆旋转相对应
matrix::Dcmf quatToInverseRotMat(const matrix::Quatf &quat);

// 当滚转倾斜大于俯仰倾斜时，应使用 3-2-1 Tait-Bryan（偏航-俯仰-滚转）旋转序列；
// 当俯仰倾斜大于滚转倾斜时，应使用 3-1-2 旋转序列，以避免万向锁。
bool shouldUse321RotationSequence(const matrix::Dcmf& R);

float getEuler321Yaw(const matrix::Quatf& q);
float getEuler321Yaw(const matrix::Dcmf& R);

float getEuler312Yaw(const matrix::Quatf& q);
float getEuler312Yaw(const matrix::Dcmf& R);

matrix::Dcmf updateEuler321YawInRotMat(float yaw, const matrix::Dcmf& rot_in);
matrix::Dcmf updateEuler312YawInRotMat(float yaw, const matrix::Dcmf& rot_in);

// Checks which euler rotation sequence to use and update yaw in rotation matrix
matrix::Dcmf updateYawInRotMat(float yaw, const matrix::Dcmf& rot_in);

namespace ecl{
	inline float powf(float x, int exp)
	{
		float ret;
		if (exp > 0) {
			ret = x;
			for (int count = 1; count < exp; count++) {
				ret *= x;
			}
			return ret;
		} else if (exp < 0) {
			return 1.0f / ecl::powf(x, -exp);
		}
		return 1.0f;
	}
}
