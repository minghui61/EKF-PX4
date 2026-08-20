/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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
 * @file AlphaFilter.hpp
 *
 * @brief 一阶 "alpha" IIR 数字滤波器，也称为泄露积分器或遗忘均值。
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <float.h>
#include <mathlib/math/Functions.hpp>

using namespace math;

template <typename T>
class AlphaFilter
{
public:
	AlphaFilter() = default;
	explicit AlphaFilter(float alpha) : _alpha(alpha) {}

	~AlphaFilter() = default;

	/**
	 * 为时间抽象设置滤波器参数
	 *
	 * 两个参数必须使用相同的单位。
	 *
	 * @param sample_interval 两个样本之间的时间间隔
	 * @param time_constant 滤波器时间常数，用于确定收敛速度
	 */
	void setParameters(float sample_interval, float time_constant)
	{
		const float denominator = time_constant + sample_interval;

		if (denominator > FLT_EPSILON) {
			setAlpha(sample_interval / denominator);
		}
	}

	bool setCutoffFreq(float sample_freq, float cutoff_freq)
	{
		if ((sample_freq <= 0.f) || (cutoff_freq <= 0.f) || (cutoff_freq >= sample_freq / 2.f)
		    || !isFinite(sample_freq) || !isFinite(cutoff_freq)) {

			// 参数无效
			return false;
		}

		setParameters(1.f / sample_freq, 1.f / (2.f * M_PI_F * cutoff_freq));
		_cutoff_freq = cutoff_freq;
		return true;
	}

	/**
	 * 直接设置滤波参数 alpha（不使用时间抽象）
	 *
	 * @param alpha [0,1] 表示对前一状态的权重。值越大表示时间常数越长。
	 */
	void setAlpha(float alpha) { _alpha = alpha; }

	/**
	 * 将滤波器状态设置为初始值
	 *
	 * @param sample 新的初始值
	 */
	void reset(const T &sample) { _filter_state = sample; }

	/**
	 * 向滤波器添加新的原始值
	 *
	 * @return 返回滤波后的结果
	 */
	const T &update(const T &sample)
	{
		_filter_state = updateCalculation(sample);
		return _filter_state;
	}

	const T &getState() const { return _filter_state; }
	float getCutoffFreq() const { return _cutoff_freq; }

protected:
	T updateCalculation(const T &sample) { return (1.f - _alpha) * _filter_state + _alpha * sample; }

	float _cutoff_freq{0.f};
	float _alpha{0.f};
	T _filter_state{};
};
