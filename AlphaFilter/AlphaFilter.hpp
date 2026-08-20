#pragma once
template <typename T>
class AlphaFilter {
public:
    AlphaFilter() = default;
    AlphaFilter(float alpha) : _alpha(alpha) {} // 修复大括号初始化报错

    ~AlphaFilter() = default;

    void reset(const T &val) { _x = val; }
    void update(const T &input, float tau, float dt) {
        float alpha = (tau > 0.0f) ? (dt / (dt + tau)) : 1.0f;
        update(input, alpha);
    }
    void update(const T &input, float alpha) {
        _x = _x * (1.0f - alpha) + input * alpha;
    }
    void update(const T &input) {
        _x = _x * (1.0f - _alpha) + input * _alpha;
    }
    void setParameters(float sample_interval, float time_constant) {
        _alpha = (time_constant > 0.0f) ? (sample_interval / (sample_interval + time_constant)) : 1.0f;
    }
    const T &getState() const { return _x; }

private:
    T _x{};
    float _alpha{0.0f};
};
