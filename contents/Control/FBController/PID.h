#ifndef PID_h
#define PID_h

#include <iostream>
#include <cmath>
#include <array>
#include "./../../MyStdFunctions.h"
#include "IFBController.h"

namespace myStd
{
template <typename T>
class PID : FBController
{
public:
    // Parameters //
    enum class Mode
    {
        pPID = 0, // 位置型PID
        sPID,     // 速度型PID
        PI_D,     // 微分先行型PID
        I_PD      // 比例微分先行型PID
    };

    struct gain_t
    {
        T Kp, Ki, Kd;
    };

    struct param_t
    {
        Mode mode;
        gain_t gain;
        bool need_saturation;
        T output_min, output_max;
    };

    // Constructors //
    PID() = default;
    PID(T kp, T ki, T kd)
    {
        _param.gain.Kp = kp;
        _param.gain.Ki = ki;
        _param.gain.Kd = kd;
    }
    PID(param_t param) : _param(param) {}

    // Public Functions //
    void reset();

    inline void setParam(const param_t param) { _param = param; }
    inline void setGain(const gain_t gain) { _param.gain = gain; }
    inline void setMode(const Mode mode) { _param.mode = mode; }
    inline void setSaturation(T min_v, T max_v);
    inline void setSaturationAbs(T max_v);

    inline void update(T target, T now_val, T dt);
    inline T getControlVal() { return output; };

private:
    param_t _param;
    inline T calculate_pPID(T target, T now_val, T dt);
    inline T calculate_sPID(T target, T now_val, T dt);
    inline T calculate_PI_D(T target, T now_val, T dt);
    inline T calculate_I_PD(T target, T now_val, T dt);

    // リセットするやつ
    std::array<T, 3> diff; // 0: 現在, 1: 過去, 2: 大過去
    T prev_val, prev_target;
    T integral;
    T output;
};

template <typename T>
void PID<T>::reset()
{
    diff.fill(0.0);
    prev_val = prev_target = 0.0;
    integral = 0.0;
    output = 0.0;
}

template <typename T>
inline void PID<T>::setSaturation(T min_v, T max_v)
{
    _param.need_saturation = true;
    _param.output_min = min_v;
    _param.output_max = max_v;
}

template <typename T>
inline void PID<T>::setSaturationAbs(T max_v)
{
    _param.need_saturation = true;
    _param.output_min = -max_v;
    _param.output_max = max_v;
}

template <typename T>
inline void PID<T>::update(T target, T now_val, T dt)
{
    diff[0] = target - now_val;                   // 最新の偏差
    integral += (diff[0] + diff[1]) * (dt / 2.0); // 積分

    switch (_param.mode)
    {
    case Mode::pPID:
        output = calculate_pPID(target, now_val, dt);
    case Mode::sPID:
        output = calculate_sPID(target, now_val, dt);
    case Mode::PI_D:
        output = calculate_PI_D(target, now_val, dt);
    case Mode::I_PD:
        output = calculate_I_PD(target, now_val, dt);
    }

    // 次回ループのために今回の値を前回の値にする
    diff[2] = diff[1];
    diff[1] = diff[0];
    prev_target = target;
    prev_val = now_val;

    // ガード処理
    if (_param.need_saturation)
        output = (_param.need_saturation) ? guard(output, _param.output_min, _param.output_max) : output;
}

// 位置型PID
template <typename T>
inline T PID<T>::calculate_pPID(T target, T now_val, T dt)
{
    T p = _param.gain.Kp * diff[0];
    T i = _param.gain.Ki * integral;
    T d = _param.gain.Kd * ((diff[0] - diff[1]) / dt);
    return p + i + d;
}

// 速度型PID
template <typename T>
inline T PID<T>::calculate_sPID(T target, T now_val, T dt)
{
    T p = _param.gain.Kp * diff[0] - diff[1];
    T i = _param.gain.Ki * diff[0] * dt;
    T d = _param.gain.Kd * (diff[0] - 2 * diff[1] + diff[2]) / dt;
    return prev_val + p + i + d;
}

// 微分先行型PID
template <typename T>
inline T PID<T>::calculate_PI_D(T target, T now_val, T dt)
{
    T p = _param.gain.Kp * diff[0];
    T i = _param.gain.Ki * integral;
    T d = -_param.gain.Kd * ((now_val - prev_val) / dt);
    return p + i + d;
}

// 比例微分先行型PID
template <typename T>
inline T PID<T>::calculate_I_PD(T target, T now_val, T dt)
{
    T p = -_param.gain.Kp * now_val;
    T i = _param.gain.Ki * integral;
    T d = -_param.gain.Kd * ((now_val - prev_val) / dt);
    return p + i + d;
}

} // namespace myStd

#endif // PID_h
