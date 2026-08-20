/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file WelfordMean.hpp
 *
 * 使用 Welford 的在线算法计算均值和方差
 */

#pragma once

namespace math
{

template <typename Type, size_t N>
class WelfordMean
{
public:
	// 对于新样本，更新计数、均值和 M2（用于方差计算）。
	void update(const matrix::Vector<Type, N> &new_value)
	{
		_count++;

		// mean 累积整个数据集的均值
		const matrix::Vector<Type, N> delta{new_value - _mean};
		_mean += delta / _count;

		// M2 累积关于均值的平方距离
		// count 记录到目前为止观察到的样本数量
		_M2 += delta.emult(new_value - _mean);

		// 防止浮点精度误差导致出现负方差
		_M2 = matrix::max(_M2, {});
	}

	bool valid() const { return _count > 2; }
	unsigned count() const { return _count; }

	void reset()
	{
		_count = 0;
		_mean = {};
		_M2 = {};
	}

	// Retrieve the mean, variance and sample variance
	matrix::Vector<Type, N> mean() const { return _mean; }
	matrix::Vector<Type, N> variance() const { return _M2 / _count; }
	matrix::Vector<Type, N> sample_variance() const { return _M2 / (_count - 1); }
private:
	matrix::Vector<Type, N> _mean{};
	matrix::Vector<Type, N> _M2{};
	unsigned _count{0};
};

} // namespace math
