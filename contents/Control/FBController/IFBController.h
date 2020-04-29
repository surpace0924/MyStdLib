#ifndef IFBController_h
#define IFBController_h

#include <iostream>
#include <cmath>
#include <array>
#include "./../../MyStdFunctions.h"

namespace myStd
{
class FBController
{
public:
    void reset();

    template <class... Args>
    void setParam(Args... args);

    template <class... Args>
    void setGain(Args... args);

    template <class... Args>
    void setMode(Args... args);

    template <class... Args>
    void setSaturation(Args... args);

    void update(double target, double now_val, double dt);

    template <typename T>
    T getControlVal();
};
} // namespace myStd

#endif // IFBController_h