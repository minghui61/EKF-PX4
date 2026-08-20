/****************************************************************************
 *
 *   Copyright (c) 2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file sensor_range_finder.cpp
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */

#include "sensor_range_finder.hpp"

namespace estimator
{
namespace sensor
{

void SensorRangeFinder::runChecks(const uint64_t current_time_us, const Dcmf &R_to_earth)
{
	updateSensorToEarthRotation(R_to_earth);
	updateValidity(current_time_us);
}

void SensorRangeFinder::updateSensorToEarthRotation(const Dcmf &R_to_earth)
{
	// 计算传感器坐标系到地球坐标系旋转矩阵的 (2,2) 分量
	// 这是超声/激光测距和光流数据使用所必需的
	_cos_tilt_rng_to_earth = R_to_earth(2, 0) * _sin_pitch_offset + R_to_earth(2, 2) * _cos_pitch_offset;
}

void SensorRangeFinder::updateValidity(uint64_t current_time_us)
{
	updateDtDataLpf(current_time_us);

	if (isSampleOutOfDate(current_time_us) || !isDataContinuous()) {
		_is_sample_valid = false;
		_is_regularly_sending_data = false;
		return;
	}

	_is_regularly_sending_data = true;

	// 只有在已从缓冲区获取新数据后，才运行检查
	if (_is_sample_ready) {
		_is_sample_valid = false;

		if (_sample.quality == 0) {
			_time_bad_quality_us = current_time_us;

		} else if (current_time_us - _time_bad_quality_us > _quality_hyst_us) {
			// 一段时间内未收到低质量数据

			if (isTiltOk() && isDataInRange()) {
				updateStuckCheck();

				if (!_is_stuck) {
					_is_sample_valid = true;
					_time_last_valid_us = _sample.time_us;
				}
			}
		}
	}
}

void SensorRangeFinder::updateDtDataLpf(uint64_t current_time_us)
{
	// 使用 2 秒时间常数计算样本间到达时间的一阶 IIR 低通滤波值。
	float alpha = 0.5f * _dt_update;
	_dt_data_lpf = _dt_data_lpf * (1.0f - alpha) + alpha * (current_time_us - _sample.time_us);

	// 对滤波器状态施加尖峰保护。
	_dt_data_lpf = fminf(_dt_data_lpf, 4e6f);
}

inline bool SensorRangeFinder::isSampleOutOfDate(uint64_t current_time_us) const
{
	return (current_time_us - _sample.time_us) > 2 * RNG_MAX_INTERVAL;
}

inline bool SensorRangeFinder::isDataInRange() const
{
	return (_sample.rng >= _rng_valid_min_val) && (_sample.rng <= _rng_valid_max_val);
}

void SensorRangeFinder::updateStuckCheck()
{
	// 当距离有效值在一段时间内不可用时，检查是否出现“卡住”的测距值
	// 这用于处理部分激光雷达传感器常见的故障模式
	if (((_sample.time_us - _time_last_valid_us) > (uint64_t)10e6)) {

		// 需要对测距值的方差进行检查，以判断是否出现“卡住”测量
		if (_stuck_max_val - _stuck_min_val > _stuck_threshold) {
			_stuck_min_val = 0.0f;
			_stuck_max_val = 0.0f;
			_is_stuck = false;

		} else {
			if (_sample.rng > _stuck_max_val) {
				_stuck_max_val = _sample.rng;
			}

			if (_stuck_min_val < 0.1f || _sample.rng < _stuck_min_val) {
				_stuck_min_val = _sample.rng;
			}

			_is_stuck = true;
		}
	}
}

} // namespace sensor
} // namespace estimator
