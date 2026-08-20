/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file Functions.hpp
 *
 * 经常复用的一些简单数学函数集合
 */

#pragma once

#include "Limits.hpp"

#include <px4_platform_common/defines.h>
#include <matrix/matrix/math.hpp>

namespace math
{

// 类型安全的符号函数，将零视为正数
template<typename T>
int signNoZero(T val)
{
	return (T(0) <= val) - (val < T(0));
}

/**
 * 基于布尔值的符号函数
 *
 * @param[in] positive 用于判断符号的真假值
 * @return 当 positive 为 true 返回 1，为 false 返回 -1
 */
inline int signFromBool(bool positive)
{
	return positive ? 1 : -1;
}

template<typename T>
T sq(T val)
{
	return val * val;
}

/*
 * 指数曲线（expo）函数的实现。
 * 本质上是线性函数与三次函数之间的线性组合。
 * @param value 输入值，范围为 [-1, 1]
 * @param e 取值范围 [0, 1]，用于设置线性与三次形状的比例
 * 		0 - 纯线性函数
 * 		1 - 纯三次函数
 * @return 函数输出值
 */
template<typename T>
const T expo(const T &value, const T &e)
{
	T x = constrain(value, (T) - 1, (T) 1);
	T ec = constrain(e, (T) 0, (T) 1);
	return (1 - ec) * x + ec * x * x * x;
}

/*
 * SuperExpo 函数实现。
 * 采用 1/(1-x) 形式进一步直观地塑造遥控输入曲线。
 * 相比其他实现进行了增强以保持输出尺度在 [-1,1] 之间。
 * @param value 输入值，范围为 [-1, 1]
 * @param e 取值范围 [0, 1]，用于设置线性与三次形状的比例（参见 expo）
 * @param g 取值范围 [0, 1)，用于设置 SuperExpo 的形状
 * 		0 - 退化为普通 expo 函数
 * 		0.99 - 曲线弯曲非常强烈，几乎在最大摇杆输入前保持为零
 * @return 函数输出值
 */
template<typename T>
const T superexpo(const T &value, const T &e, const T &g)
{
	T x = constrain(value, (T) - 1, (T) 1);
	T gc = constrain(g, (T) 0, (T) 0.99);
	return expo(x, e) * (1 - gc) / (1 - fabsf(x) * gc);
}

/*
 * Deadzone（死区）函数，在死区之外保持线性并连续
 * 1                ------
 *                /
 *             --
 *           /
 * -1 ------
 *        -1 -dz +dz 1
 * @param value 输入值，范围为 [-1, 1]
 * @param dz 取值范围 [0, 1)，表示死区在整个区间的比例
 * 		0 - 无死区，线性映射 [-1, 1]
 * 		0.5 - 死区为区间的一半，即 [-0.5, 0.5]
 * 		0.99 - 几乎整个区间是死区
 */
template<typename T>
const T deadzone(const T &value, const T &dz)
{
	T x = constrain(value, (T) - 1, (T) 1);
	T dzc = constrain(dz, (T) 0, (T) 0.99);
	// Rescale the input such that we get a piecewise linear function that will be continuous with applied deadzone
	T out = (x - matrix::sign(x) * dzc) / (1 - dzc);
	// apply the deadzone (values zero around the middle)
	return out * (fabsf(x) > dzc);
}

template<typename T>
const T expo_deadzone(const T &value, const T &e, const T &dz)
{
	return expo(deadzone(value, dz), e);
}


/*
 * 分段函数：常数、线性、常数，由两个转折点参数定义
 * y_high          -------
 *                /
 *               /
 *              /
 * y_low -------
 *         x_low   x_high
 */
template<typename T>
const T gradual(const T &value, const T &x_low, const T &x_high, const T &y_low, const T &y_high)
{
	if (value <= x_low) {
		return y_low;

	} else if (value > x_high) {
		return y_high;

	} else {
		/* linear function between the two points */
		T a = (y_high - y_low) / (x_high - x_low);
		T b = y_low - a * x_low;
		return  a * value + b;
	}
}

/*
 * 分段函数：常数、线性、线性、常数，由三个转折点参数定义
 *  y_high               -------
 *                      /
 *                    /
 *  y_middle        /
 *                /
 *               /
 *              /
 * y_low -------
 *         x_low x_middle x_high
 */
template<typename T>
const T gradual3(const T &value,
		 const T &x_low, const T &x_middle, const T &x_high,
		 const T &y_low, const T &y_middle, const T &y_high)
{
	if (value < x_middle) {
		return gradual(value, x_low, x_middle, y_low, y_middle);

	} else {
		return gradual(value, x_middle, x_high, y_middle, y_high);
	}
}

/*
 * 平方根与线性混合函数，在交点 (1,1) 处连接
 *                     /
 *      线性          /
 *                   /
 * 1                /
 *                /
 *      平方根     |
 *              |
 * 0     -------
 *             0    1
 */
template<typename T>
const T sqrt_linear(const T &value)
{
	if (value < static_cast<T>(0)) {
		return static_cast<T>(0);

	} else if (value < static_cast<T>(1)) {
		return sqrtf(value);

	} else {
		return value;
	}
}

/*
 * 在两点 a 和 b 之间进行线性插值。
 * s=0 返回 a
 * s=1 返回 b
 * s 可以为任意值。
 */
template<typename T>
const T lerp(const T &a, const T &b, const T &s)
{
	return (static_cast<T>(1) - s) * a + s * b;
}

template<typename T>
constexpr T negate(T value)
{
	static_assert(sizeof(T) > 2, "implement for T");
	return -value;
}

template<>
constexpr int16_t negate<int16_t>(int16_t value)
{
	if (value == INT16_MAX) {
		return INT16_MIN;

	} else if (value == INT16_MIN) {
		return INT16_MAX;
	}

	return -value;
}

/*
 * 计算汉明重量（Hamming weight），即统计给定整数中被置位的比特数量。
 */

template<typename T>
int countSetBits(T n)
{
	int count = 0;

	while (n) {
		count += n & 1;
		n >>= 1;
	}

	return count;
}

inline bool isFinite(const float &value)
{
	return PX4_ISFINITE(value);
}

inline bool isFinite(const matrix::Vector3f &value)
{
	return PX4_ISFINITE(value(0)) && PX4_ISFINITE(value(1)) && PX4_ISFINITE(value(2));
}

} /* namespace math */
