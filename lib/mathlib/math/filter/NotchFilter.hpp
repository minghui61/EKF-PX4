/****************************************************************************
 *
 *   Copyright (C) 2019-2021 PX4 Development Team. All rights reserved.
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

/*
 * @file NotchFilter.hpp
 *
 * @brief Notch（陷波）滤波器的实现。
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 * @author Samuel Garcin <samuel.garcin@wecorpindustries.com>
 */

#pragma once

#include <mathlib/math/Functions.hpp>
#include <cmath>
#include <float.h>
#include <matrix/math.hpp>

namespace math
{

template<typename T>
class NotchFilter
{
public:
	NotchFilter() = default;
	~NotchFilter() = default;

	bool setParameters(float sample_freq, float notch_freq, float bandwidth);

	/**
	 * Add a new raw value to the filter using the Direct Form I
	 *
	 * @return retrieve the filtered result
	 */
	inline T apply(const T &sample)
	{
		if (!_initialized) {
			reset(sample);
			_initialized = true;
		}

		return applyInternal(sample);
	}

	// Filter array of samples in place using the direct form I
	inline void applyArray(T samples[], int num_samples)
	{
		if (!_initialized) {
			reset(samples[0]);
			_initialized = true;
		}

		for (int n = 0; n < num_samples; n++) {
			samples[n] = applyInternal(samples[n]);
		}
	}

	float getNotchFreq() const { return _notch_freq; }
	float getBandwidth() const { return _bandwidth; }

	// 仅在单元测试中使用
	void getCoefficients(float a[3], float b[3]) const
	{
		a[0] = 1.f;
		a[1] = _a1;
		a[2] = _a2;
		b[0] = _b0;
		b[1] = _b1;
		b[2] = _b2;
	}

	float getMagnitudeResponse(float frequency) const
	{
		float w = 2.f * M_PI_F * frequency / _sample_freq;

		float numerator = _b0 * _b0 + _b1 * _b1 + _b2 * _b2
				  + 2.f * (_b0 * _b1 + _b1 * _b2) * cosf(w) + 2.f * _b0 * _b2 * cosf(2.f * w);

		float denominator = 1.f + _a1 * _a1 + _a2 * _a2 + 2.f * (_a1 + _a1 * _a2) * cosf(w) + 2.f * _a2 * cosf(2.f * w);

		return sqrtf(numerator / denominator);
	}

	/**
	 * 绕过滤波器的更新以直接设置不同的滤波器系数。
	 * 注意：如果使用此方法更改滤波器系数，则保存在滤波器上的滤波频率和品质因数将失去其物理意义。
	 * 该方法用于创建特定滤波器的克隆。
	 */
	void setCoefficients(float a[2], float b[3])
	{
		_a1 = a[0];
		_a2 = a[1];
		_b0 = b[0];
		_b1 = b[1];
		_b2 = b[2];
	}

	bool initialized() const { return _initialized; }

	void reset() { _initialized = false; }

	void reset(const T &sample)
	{
		const T input = isFinite(sample) ? sample : T{};

		_delay_element_1 = _delay_element_2 = input;
		_delay_element_output_1 = _delay_element_output_2 = input * (_b0 + _b1 + _b2) / (1 + _a1 + _a2);

		if (!isFinite(_delay_element_1) || !isFinite(_delay_element_2)) {
			_delay_element_output_1 = _delay_element_output_2 = {};
		}

		_initialized = true;
	}

	void disable()
	{
		// no filtering
		_notch_freq = 0.f;
		_bandwidth = 0.f;
		_sample_freq = 0.f;

		_b0 = 1.f;
		_b1 = 0.f;
		_b2 = 0.f;

		_a1 = 0.f;
		_a2 = 0.f;

		_initialized = false;
	}

protected:

	/**
	 * Add a new raw value to the filter using the Direct Form I
	 *
	 * @return retrieve the filtered result
	 */
	inline T applyInternal(const T &sample)
	{
		// 直接一阶（Direct Form I）实现
		T output = _b0 * sample + _b1 * _delay_element_1 + _b2 * _delay_element_2 - _a1 * _delay_element_output_1 - _a2 *
			   _delay_element_output_2;

		// 输入移位
		_delay_element_2 = _delay_element_1;
		_delay_element_1 = sample;

		// 输出移位
		_delay_element_output_2 = _delay_element_output_1;
		_delay_element_output_1 = output;

		return output;
	}

	T _delay_element_1{};
	T _delay_element_2{};
	T _delay_element_output_1{};
	T _delay_element_output_2{};

	// All the coefficients are normalized by a0, so a0 becomes 1 here
	float _a1{0.f};
	float _a2{0.f};

	float _b0{1.f};
	float _b1{0.f};
	float _b2{0.f};

	float _notch_freq{};
	float _bandwidth{};
	float _sample_freq{};

	bool _initialized{false};
};

/**
 * 通过设置参数和系数初始化滤波器。
 * 使用 Direct Form I 方法，可以在保持滤波器历史（状态）的同时，动态更新滤波频率、刷新率和品质因数。
 */
template<typename T>
bool NotchFilter<T>::setParameters(float sample_freq, float notch_freq, float bandwidth)
{
	if ((sample_freq <= 0.f) || (notch_freq <= 0.f) || (bandwidth <= 0.f) || (notch_freq >= sample_freq / 2)
	    || !isFinite(sample_freq) || !isFinite(notch_freq) || !isFinite(bandwidth)) {

		// 参数无效
		disable();
		return false;
	}

	const float freq_min = sample_freq * 0.001f;

	const float notch_freq_new = math::max(notch_freq, freq_min);
	const float bandwidth_new = math::max(bandwidth, freq_min);

	const float sample_freq_diff = fabsf(sample_freq - _sample_freq);
	const float bandwidth_diff = fabsf(bandwidth_new - _bandwidth);

	const bool sample_freq_change = (sample_freq_diff > FLT_EPSILON);
	const bool bandwidth_change = (bandwidth_diff > FLT_EPSILON);

	if (!sample_freq_change && !bandwidth_change) {
		const float notch_freq_diff = fabsf(notch_freq_new - _notch_freq);

		if (notch_freq_diff > FLT_EPSILON) {
			// only notch frequency has changed
			_notch_freq = notch_freq_new;

			const float beta = -cosf(2.f * M_PI_F * _notch_freq / _sample_freq);

			_b1 = 2.f * beta * _b0;
			_a1 = _b1;

			if (notch_freq_diff > _bandwidth) {
				// force reset
				_initialized = false;
			}

			if (!isFinite(_b1)) {
				disable();
				return false;
			}

			return true;

		} else {
			// no change, do nothing
			return true;
		}
	}

	_sample_freq = sample_freq;
	_notch_freq = notch_freq_new;
	_bandwidth = bandwidth_new;

	const float alpha = tanf(M_PI_F * _bandwidth / _sample_freq);
	const float beta = -cosf(2.f * M_PI_F * _notch_freq / _sample_freq);
	const float a0_inv = 1.f / (alpha + 1.f);

	_b0 = a0_inv;
	_b1 = 2.f * beta * a0_inv;
	_b2 = a0_inv;

	_a1 = _b1;
	_a2 = (1.f - alpha) * a0_inv;

	if (!isFinite(_b0) || !isFinite(_b1) || !isFinite(_b2) || !isFinite(_a2)) {
		disable();
		return false;
	}

	// force reset if bandwidth or sample frequency changed by more than 1%
	if (bandwidth_diff > 0.01f * _bandwidth) {
		_initialized = false;

	} else if (sample_freq_diff > 0.01f * _sample_freq) {
		_initialized = false;
	}

	return true;
}

} // namespace math
