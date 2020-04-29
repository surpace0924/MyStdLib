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
    enum class Drive
    {
        diff = 0, // 2DoF（差動二輪型）
        omni      // 3DoF（全方位移動型）
    };

    T hz = 1; // 制御周期[Hz]

    // Constructors //
    PurePursuitControl() = default;
    PurePursuitControl(std::vector<Pose2D<T>> path)
    {
        set(path);
    }

    // Public Functions //
    void set(std::vector<Pose2D<T>> path)
    {
        _path.clear();
        push_back(path);
    }

    void push_back(std::vector<Pose2D<T>> path)
    {
        for (auto p : path)
            _path.push_back(p);
    }

    void push_back(Pose2D<T> path)
    {
        _path.push_back(path);
    }

    void setController(T_fbc fbc_linear, T_fbc fbc_angular)
    {
        _fbc_linear = fbc_linear;
        _fbc_angular = fbc_angular;
    }

    void update(int idx, myStd::Pose2D<T> now_pose, T dt)
    {
        T distance = Pose2D<T>::getDistance(_path[idx], now_pose);
        _fbc_linear.update(0, distance, dt);
        output.x = _fbc_linear.getControlVal();

        T angle = Pose2D<T>::getAngle(_path[idx], now_pose);
        _fbc_angular.update(0, angle, dt);
        output.theta = _fbc_angular.getControlVal();
    }

    Pose2D<T> getControlVal() { return output; }

private:
    T_fbc _fbc_linear;
    T_fbc _fbc_angular;
    Pose2D<T> output;
    std::vector<Pose2D<T>> _path; // 通過点のリスト

}; // namespace myStd
} // namespace myStd
#endif // PurePursuitControl_h
