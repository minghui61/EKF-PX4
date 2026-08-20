#pragma once

#include "math.hpp"

#if defined (__PX4_NUTTX) || defined (__PX4_QURT)
#include <px4_defines.h>
#endif

namespace matrix
{

template<typename Type>
bool is_finite(Type x)
{
#if defined (__PX4_NUTTX)
	return PX4_ISFINITE(x);
#elif defined (__PX4_QURT)
	return __builtin_isfinite(x);
#else
	return std::isfinite(x);
#endif
}

/**
 * 比较两个浮点数是否相等
 *
 * NAN 被视为等同于 NAN 和 -NAN
 * INFINITY 被视为等同于 INFINITY，但不等于 -INFINITY
 *
 * @param x 右侧的比较值
 * @param y 左侧的比较值
 * @param eps 判断相等的数值容差
 * @return 如果两个值被视为相等则返回 true，否则返回 false
 */
template<typename Type>
bool isEqualF(const Type x, const Type y, const Type eps = Type(1e-4f))
{
	return (matrix::fabs(x - y) <= eps)
	       || (isnan(x) && isnan(y))
	       || (isinf(x) && isinf(y) && isnan(x - y));
}

namespace detail
{

template<typename Floating>
Floating wrap_floating(Floating x, Floating low, Floating high)
{
	// already in range
	if (low <= x && x < high) {
		return x;
	}

	const auto range = high - low;
	const auto inv_range = Floating(1) / range; // should evaluate at compile time, multiplies below at runtime
	const auto num_wraps = floor((x - low) * inv_range);
	return x - range * num_wraps;
}

}  // namespace detail

/**
 * 将单精度浮点值包裹到区间 [low, high)
 *
 * @param x 可能超出范围的输入值
 * @param low 允许范围的下界
 * @param high 允许范围的上界
 * @return 包裹后位于范围内的值
 */
inline float wrap(float x, float low, float high)
{
	return matrix::detail::wrap_floating(x, low, high);
}

/**
 * 将双精度浮点值包裹到区间 [low, high)
 *
 * @param x 可能超出范围的输入值
 * @param low 允许范围的下界
 * @param high 允许范围的上界
 * @return 包裹后位于范围内的值
 */
inline double wrap(double x, double low, double high)
{
	return matrix::detail::wrap_floating(x, low, high);
}

/**
 * 将整数值包裹到区间 [low, high)
 *
 * @param x 可能超出范围的输入值
 * @param low 允许范围的下界
 * @param high 允许范围的上界
 * @return 包裹后位于范围内的值
 */
template<typename Integer>
Integer wrap(Integer x, Integer low, Integer high)
{
	const auto range = high - low;

	if (x < low) {
		x += range * ((low - x) / range + 1);
	}

	return low + (x - low) % range;
}

/**
 * 将值包裹到区间 [-π, π)
 */
template<typename Type>
Type wrap_pi(Type x)
{
	return wrap(x, Type(-M_PI), Type(M_PI));
}

/**
 * 将值包裹到区间 [0, 2π)
 */
template<typename Type>
Type wrap_2pi(Type x)
{
	return wrap(x, Type(0), Type(M_TWOPI));
}

/**
 * 对先前按 [low, high) 包裹过的值进行展开（unwrap）
 *
 * @param[in] last_x 上一次未包裹的值
 * @param[in] new_x 当前处于区间内的新值
 * @param low 包裹区间的下界
 * @param high 包裹区间的上界
 * @return 展开后的新值
 */
template<typename Type>
Type unwrap(const Type last_x, const Type new_x, const Type low, const Type high)
{
	return last_x + wrap(new_x - last_x, low, high);
}

/**
 * 对按 [-π, π) 包裹的角度值进行展开（unwrap）
 *
 * @param[in] last_angle 上一次未包裹的角度 [rad]
 * @param[in] new_angle 新角度，范围 [-π, π] [rad]
 * @return 展开后的角度 [rad]
 */
template<typename Type>
Type unwrap_pi(const Type last_angle, const Type new_angle)
{
	return unwrap(last_angle, new_angle, Type(-M_PI), Type(M_PI));
}

/**
 * 类型安全的符号（sign/signum）函数
 *
 * @param[in] val 要取符号的数值
 * @return 如果 val < 0 则返回 -1，val == 0 则返回 0，val > 0 则返回 1
 */
template<typename T>
int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

} // namespace matrix
