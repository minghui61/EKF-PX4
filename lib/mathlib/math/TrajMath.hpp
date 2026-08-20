/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file TrajMath.hpp
 *
 * 用于轨迹生成的一组函数
 */

#pragma once

namespace math
{

namespace trajectory
{

/*
 * 计算在给定期望速度、剩余距离、最大加速度和最大跃度（jerk）下，路径上允许的最大速度。
 * 假设加速度为恒定型，且存在 2*accel/jerk 的延迟（从相反方向最大加速度达到目标加速度所需时间）。
 * 需解的方程：vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)
 *
 * @param jerk 最大跃度
 * @param accel 最大加速度
 * @param braking_distance 距离目标点的制动距离
 * @param final_speed 到达制动距离时车辆仍保持的速度
 *
 * @return 最大允许速度
 */
inline float computeMaxSpeedFromDistance(const float jerk, const float accel, const float braking_distance,
		const float final_speed)
{
	auto sqr = [](float f) {return f * f;};
	float b =  4.0f * sqr(accel) / jerk;
	float c = - 2.0f * accel * braking_distance - sqr(final_speed);
	float max_speed = 0.5f * (-b + sqrtf(sqr(b) - 4.0f * c));

	// don't slow down more than the end speed, even if the conservative accel ramp time requests it
	return max(max_speed, final_speed);
}

/* Compute the maximum tangential speed in a circle defined by two line segments of length "d"
 * forming a V shape, opened by an angle "alpha". The circle is tangent to the end of the
 * two segments as shown below:
 *      \\
 *      | \ d
 *      /  \
 *  __='___a\
 *      d
 *  @param alpha 两条线段之间的夹角
 *  @param accel 最大侧向加速度
 *  @param d 两条线段的长度
 *
 *  @return 最大切向速度
 */
inline float computeMaxSpeedInWaypoint(const float alpha, const float accel, const float d)
{
	float tan_alpha = tanf(alpha / 2.0f);
	float max_speed_in_turn = sqrtf(accel * d * tan_alpha);

	return max_speed_in_turn;
}

/*
 * 计算给定最大加速度、最大跃度和最大延迟加速度情况下的制动距离。
 * 假设加速度为恒定型，且存在 accel_delay_max/jerk 的延迟（从相反方向最大加速度达到目标加速度所需时间）。
 * 需解的方程：vel_final^2 = vel_initial^2 - 2*accel*(x - vel_initial*2*accel/jerk)
 *
 * @param velocity 初始速度
 * @param jerk 最大跃度
 * @param accel 制动过程中的目标最大加速度
 * @param accel_delay_max 描述上述延迟的加速度值
 *
 * @return 制动距离
 */
inline float computeBrakingDistanceFromVelocity(const float velocity, const float jerk, const float accel,
		const float accel_delay_max)
{
	return velocity * (velocity / (2.0f * accel) + accel_delay_max / jerk);
}

} /* namespace traj */
} /* namespace math */
