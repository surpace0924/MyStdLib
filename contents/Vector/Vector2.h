#ifndef Vector2_h
#define Vector2_h

#include <iostream>
#include <cmath>
#include <string>
#include "./../MyStdFunctions.h"

class Vector2
{
public:
    double x = 0;
    double y = 0;

    // Constructors //
    Vector2() = default;
    constexpr Vector2(double _x, double _y) : x(_x), y(_y) {}

    // Public Functions //
    bool equals(const Vector2 &v)
    {
        return *this == v;
    }

    void normalize()
    {
        *this /= length();
    }

    void set(double _x, double _y)
    {
        x = _x;
        y = _y;
    }

    void setByPolar(double r, double angle)
    {
        x = r * std::cos(angle);
        y = r * std::sin(angle);
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

    Vector2 normalized() const
    {
        return *this / length();
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
    static double getDot(Vector2 a, Vector2 b)
    {
        return (a.x * b.x + a.y * b.y);
    }

    static double getAngle(Vector2 a, Vector2 b)
    {
        return std::atan2(b.y - a.y, b.x - a.x);
    }

    static double getDistance(Vector2 a, Vector2 b)
    {
        Vector2 v = (b - a);
        return v.magnitude();
    }

    static Vector2 leap(Vector2 a, Vector2 b, double t)
    {
        t = guard(t, 0.0, 1.0);
        Vector2 v = a;
        v.x += (b.x - a.x) * t;
        v.y += (b.y - a.y) * t;
        return v;
    }

    // Operators //
    constexpr Vector2 operator+() const
    {
        return *this;
    }

    constexpr Vector2 operator-() const
    {
        return {-x, -y};
    }

    constexpr Vector2 operator+(const Vector2 &v) const
    {
        return {x + v.x, y + v.y};
    }

    constexpr Vector2 operator-(const Vector2 &v) const
    {
        return {x - v.x, y - v.y};
    }

    constexpr Vector2 operator*(double s) const
    {
        return {x * s, y * s};
    }

    constexpr Vector2 operator/(double s) const
    {
        return {x / s, y / s};
    }

    Vector2 &operator+=(const Vector2 &v)
    {
        x += v.x;
        y += v.y;
        return *this;
    }

    Vector2 &operator-=(const Vector2 &v)
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Vector2 &operator*=(double s)
    {
        x *= s;
        y *= s;
        return *this;
    }

    Vector2 &operator/=(double s)
    {
        x /= s;
        y /= s;
        return *this;
    }

    bool operator==(const Vector2 &v) const
    {
        return (x == v.x && (y == v.y));
    }

    bool operator!=(const Vector2 &v) const
    {
        return !(x == v.x && (y == v.y));
    }

private:
};

template <class Char>
inline std::basic_ostream<Char> &operator<<(std::basic_ostream<Char> &os, const Vector2 &v)
{
    return os << Char('(') << v.x << Char(',') << Char(' ') << v.y << Char(')');
}

template <class Char>
inline std::basic_istream<Char> &operator>>(std::basic_istream<Char> &is, Vector2 &v)
{
    Char unused;
    return is >> unused >> v.x >> unused >> v.y >> unused;
}

#endif // Vector2_h
