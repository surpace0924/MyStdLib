#ifndef PurePursuitControl_h
#define PurePursuitControl_h

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include "./../MyStdFunctions.h"
#include "./../Vector/Vector.h"
#include "./FBController/FBController.h"

namespace myStd
{
template <typename T, typename T_fbc>
class PurePursuitControl
{
public:
    // Parameters //
    enum class Mode
    {
        diff = 0, // 2DoF（差動二輪型）
        omni      // 3DoF（全方位移動型）
    };

    struct param_t
    {
        Mode mode;
        T_fbc fbc_linear;
        T_fbc fbc_angular;
    };

    // Constructors //
    PurePursuitControl() = default;
    PurePursuitControl(std::vector<Pose2D<T>> path) { set(path); }

    // Public Functions //
    inline void set(const std::vector<Pose2D<T>> path);
    inline void setParam(const PurePursuitControl::param_t param) { _param = param; };
    inline void setMode(const PurePursuitControl::Mode mode) { _param.mode = mode; };
    inline void setController(T_fbc fbc_linear, T_fbc fbc_angular);
    inline void push_back(std::vector<Pose2D<T>> path);
    inline void push_back(Pose2D<T> path) { _path.push_back(path); }
    inline void update(int idx, myStd::Pose2D<T> now_pose, T dt);
    inline Pose2D<T> getControlVal() { return output; }

private:
    param_t _param;
    Pose2D<T> output;
    std::vector<Pose2D<T>> _path; // 通過点のリスト

}; // namespace myStd

template <typename T, typename T_fbc>
inline void PurePursuitControl<T, T_fbc>::set(std::vector<Pose2D<T>> path)
{
    _path.clear();
    push_back(path);
}

template <typename T, typename T_fbc>
inline void PurePursuitControl<T, T_fbc>::push_back(std::vector<Pose2D<T>> path)
{
    for (auto p : path)
        _path.push_back(p);
}

template <typename T, typename T_fbc>
inline void PurePursuitControl<T, T_fbc>::setController(T_fbc fbc_linear, T_fbc fbc_angular)
{
    _param.fbc_linear = fbc_linear;
    _param.fbc_angular = fbc_angular;
}

template <typename T, typename T_fbc>
inline void PurePursuitControl<T, T_fbc>::update(int idx, myStd::Pose2D<T> now_pose, T dt)
{
    T distance = Pose2D<T>::getDistance(_path[idx], now_pose);
    _param.fbc_linear.update(0, distance, dt);
    output.x = _param.fbc_linear.getControlVal();

    T angle = Pose2D<T>::getAngle(_path[idx], now_pose);
    _param.fbc_angular.update(0, angle, dt);
    output.theta = _param.fbc_angular.getControlVal();
}

} // namespace myStd
#endif // PurePursuitControl_h
