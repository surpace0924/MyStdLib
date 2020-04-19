#ifndef Pose2D_h
#define Pose2D_h

#include <iostream>
#include <cmath>
#include <string>
#include "./../MyStdFunctions.h"
#include "Vector2.h"

namespace myStd
{
class Pose2D
{
public:
    double x = 0;
    double y = 0;
    double theta = 0;

    // Constructors //
    Pose2D() = default;

    constexpr Pose2D(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}

    constexpr Pose2D(const Vector2 &v, double _theta)
    {
        x = v.x;
        y = v.y;
        theta = _theta;
    }

    constexpr Pose2D(double _x, double _y) : x(_x), y(_y) {}

    constexpr Pose2D(const Vector2 &v)
    {
        x = v.x;
        y = v.y;
    }

    // Public Functions //
    bool equals(const Pose2D &v)
    {
        return *this == v;
    }

    void set(double _x, double _y, double _theta)
    {
        x = _x;
        y = _y;
        theta = _theta;
    }

    void setByPolar(double r, double angle, double robot_theta)
    {
        x = r * std::cos(angle);
        y = r * std::sin(angle);
        theta = robot_theta;
    }

    // 原点中心に回転
    void rotate(double angle)
    {
        Vector2 p(0, 0);
        rotate(p, angle);
    }

    // 指定座標中心(rot_x, rot_y)に回転
    void rotate(double rot_x, double rot_y, double angle)
    {
        Vector2 p(rot_x, rot_y);
        rotate(p, angle);
    }

    // 座標oを中心にangleだけ回転
    void rotate(Vector2 o, double angle)
    {
        Vector2 p(x - o.x, y - o.y);
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

    double length() const
    {
        return magnitude();
    }

    double magnitude() const
    {
        return std::sqrt(sqrMagnitude());
    }

    constexpr double sqrLength() const
    {
        return sqrMagnitude();
    }

    constexpr double sqrMagnitude() const
    {
        return x * x + y * y;
    }

    // Static functuons //
    static double getDot(Pose2D a, Pose2D b)
    {
        return (a.x * b.x + a.y * b.y);
    }

    static double getAngle(Pose2D a, Pose2D b)
    {
        return std::atan2(b.y - a.y, b.x - a.x);
    }

    static double getDistance(Pose2D a, Pose2D b)
    {
        Pose2D v = (b - a);
        return v.magnitude();
    }

    static Pose2D leap(Pose2D a, Pose2D b, double t)
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

    constexpr Pose2D operator*(double s) const
    {
        return {x * s, y * s, theta * s};
    }

    constexpr Pose2D operator/(double s) const
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

    Pose2D &operator*=(double s)
    {
        x *= s;
        y *= s;
        theta *= s;
        return *this;
    }

    Pose2D &operator/=(double s)
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

template <class Char>
inline std::basic_ostream<Char> &operator<<(std::basic_ostream<Char> &os, const Pose2D &v)
{
    return os << Char('(') << v.x << Char(',') << Char(' ') << v.y << Char(',') << Char(' ') << v.theta << Char(')');
}

template <class Char>
inline std::basic_istream<Char> &operator>>(std::basic_istream<Char> &is, Pose2D &v)
{
    Char unused;
    return is >> unused >> v.x >> unused >> v.y >> unused >> v.theta >> unused;
}
} // namespace myStd
#endif // Pose2D_h
