#ifndef Vector2_h
#define Vector2_h

#include <iostream>
#include <cmath>
#include <string>
#include "./../MyStdFunctions.h"

namespace myStd
{
template <typename T>
class Vector2
{
public:
    T x = 0;
    T y = 0;

    // Constructors //
    constexpr Vector2() = default;
    constexpr Vector2(T _x, T _y) : x(_x), y(_y) {}

    // Public Functions //
    bool equals(const Vector2 &v)
    {
        return *this == v;
    }

    void normalize()
    {
        *this /= length();
    }

    void set(T _x, T _y)
    {
        x = _x;
        y = _y;
    }

    void setByPolar(T r, T angle)
    {
        x = r * std::cos(angle);
        y = r * std::sin(angle);
    }

    // 原点中心に回転
    void rotate(T angle)
    {
        Vector2 p(0, 0);
        rotate(p, angle);
    }

    // 指定座標中心(rot_x, rot_y)に回転
    void rotate(T rot_x, T rot_y, T angle)
    {
        Vector2 p(rot_x, rot_y);
        rotate(p, angle);
    }

    // 座標oを中心にangleだけ回転
    void rotate(Vector2 o, T angle)
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

    T length() const
    {
        return magnitude();
    }

    T magnitude() const
    {
        return std::sqrt(sqrMagnitude());
    }

    Vector2 normalized() const
    {
        return *this / length();
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
    static T getDot(Vector2 a, Vector2 b)
    {
        return (a.x * b.x + a.y * b.y);
    }

    static T getAngle(Vector2 a, Vector2 b)
    {
        return std::atan2(b.y - a.y, b.x - a.x);
    }

    static T getDistance(Vector2 a, Vector2 b)
    {
        Vector2 v = (b - a);
        return v.magnitude();
    }

    static Vector2 leap(Vector2 a, Vector2 b, T t)
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

    constexpr Vector2 operator*(T s) const
    {
        return {x * s, y * s};
    }

    constexpr Vector2 operator/(T s) const
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

    Vector2 &operator*=(T s)
    {
        x *= s;
        y *= s;
        return *this;
    }

    Vector2 &operator/=(T s)
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

template <typename Char, typename T>
inline std::basic_ostream<Char> &operator<<(std::basic_ostream<Char> &os, const Vector2<T> &v)
{
    return os << Char('(') << v.x << Char(',') << Char(' ') << v.y << Char(')');
}

template <typename Char, typename T>
inline std::basic_istream<Char> &operator>>(std::basic_istream<Char> &is, Vector2<T> &v)
{
    Char unused;
    return is >> unused >> v.x >> unused >> v.y >> unused;
}
} // namespace myStd
#endif // Vector2_h
