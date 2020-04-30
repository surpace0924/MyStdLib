#ifndef Pose2D_h
#define Pose2D_h

#include <iostream>
#include <cmath>
#include <string>
#include "./../MyStdFunctions.h"
#include "Vector2.h"

namespace myStd
{
template <typename T>
class Pose2D
{
public:
    T x = 0;
    T y = 0;
    T theta = 0;

    // Constructors //
    constexpr Pose2D() = default;

    constexpr Pose2D(T _x, T _y, T _theta) : x(_x), y(_y), theta(_theta) {}

    constexpr Pose2D(const Vector2<T> &v, T _theta)
    {
        x = v.x;
        y = v.y;
        theta = _theta;
    }

    constexpr Pose2D(T _x, T _y) : x(_x), y(_y) {}

    constexpr Pose2D(const Vector2<T> &v)
    {
        x = v.x;
        y = v.y;
    }

    // Public Functions //
    bool equals(const Pose2D &v)
    {
        return *this == v;
    }

    void set(T _x, T _y, T _theta)
    {
        x = _x;
        y = _y;
        theta = _theta;
    }

    void setByPolar(T r, T angle, T robot_theta)
    {
        x = r * std::cos(angle);
        y = r * std::sin(angle);
        theta = robot_theta;
    }

    // 原点中心に回転
    void rotate(T angle)
    {
        Vector2<T> p(0, 0);
        rotate(p, angle);
    }

    // 指定座標中心(rot_x, rot_y)に回転
    void rotate(T rot_x, T rot_y, T angle)
    {
        Vector2<T> p(rot_x, rot_y);
        rotate(p, angle);
    }

    // 座標oを中心にangleだけ回転
    void rotate(Vector2<T> o, T angle)
    {
        Vector2<T> p(x - o.x, y - o.y);
        p.rotate(angle);
        p.x += o.x;
        p.y += o.y;
        x = p.x;
        y = p.y;
    }

    std::string toString()
    {
        return '(' + std::to_string(x) + ", " + std::to_string(y) + ')';
    }

    T length() const
    {
        return magnitude();
    }

    T magnitude() const
    {
        return std::sqrt(sqrMagnitude());
    }

    constexpr T sqrLength() const
    {
        return sqrMagnitude();
    }

    constexpr T sqrMagnitude() const
    {
        return x * x + y * y;
    }

    // Static functuons //
    static T getDot(Pose2D a, Pose2D b)
    {
        return (a.x * b.x + a.y * b.y);
    }

    static T getAngle(Pose2D a, Pose2D b)
    {
        return std::atan2(b.y - a.y, b.x - a.x);
    }

    static T getDistance(Pose2D a, Pose2D b)
    {
        Pose2D v = (b - a);
        return v.magnitude();
    }

    static Pose2D leap(Pose2D a, Pose2D b, T t)
    {
        t = guard(t, 0.0, 1.0);
        Pose2D v = a;
        v.x += (b.x - a.x) * t;
        v.y += (b.y - a.y) * t;
        v.theta += (b.theta - a.theta) * t;
        return v;
    }

    // Operators //
    constexpr Pose2D operator+() const
    {
        return *this;
    }

    constexpr Pose2D operator-() const
    {
        return {-x, -y, -theta};
    }

    constexpr Pose2D operator+(const Pose2D &v) const
    {
        return {x + v.x, y + v.y, theta + v.theta};
    }

    constexpr Pose2D operator-(const Pose2D &v) const
    {
        return {x - v.x, y - v.y, theta - v.theta};
    }

    constexpr Pose2D operator*(T s) const
    {
        return {x * s, y * s, theta * s};
    }

    constexpr Pose2D operator/(T s) const
    {
        return {x / s, y / s, theta / s};
    }

    Pose2D &operator+=(const Pose2D &v)
    {
        x += v.x;
        y += v.y;
        theta += v.theta;
        return *this;
    }

    Pose2D &operator-=(const Pose2D &v)
    {
        x -= v.x;
        y -= v.y;
        theta -= v.theta;
        return *this;
    }

    Pose2D &operator*=(T s)
    {
        x *= s;
        y *= s;
        theta *= s;
        return *this;
    }

    Pose2D &operator/=(T s)
    {
        x /= s;
        y /= s;
        theta /= s;
        return *this;
    }

    bool operator==(const Pose2D &v) const
    {
        return ((x == v.x) && (y == v.y) && (theta == v.y));
    }

    bool operator!=(const Pose2D &v) const
    {
        return !((x == v.x) && (y == v.y) && (theta == v.y));
    }

private:
};

template <typename Char, typename T>
inline std::basic_ostream<Char> &operator<<(std::basic_ostream<Char> &os, const Pose2D<T> &v)
{
    return os << Char('(') << v.x << Char(',') << Char(' ') << v.y << Char(',') << Char(' ') << v.theta << Char(')');
}

template <typename Char, typename T>
inline std::basic_istream<Char> &operator>>(std::basic_istream<Char> &is, Pose2D<T> &v)
{
    Char unused;
    return is >> unused >> v.x >> unused >> v.y >> unused >> v.theta >> unused;
}
} // namespace myStd
#endif // Pose2D_h
