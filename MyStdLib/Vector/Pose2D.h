/**
 * @file Pose2D.h
 * @brief 2次元の座標を扱う
**/
#ifndef Pose2D_h
#define Pose2D_h

#include <iostream>
#include <cmath>
#include <string>
#include "./../MyStdFunctions.h"
#include "Vector2.h"

namespace myStd
{
    /**
     * @brief 2次元の座標を扱う
    **/
    template <typename T>
    class Pose2D
    {
    public:
        T x = 0;     /**< 2次元直交座標におけるx成分 */
        T y = 0;     /**< 2次元直交座標におけるy成分 */
        T theta = 0; /**< 2次元直交座標における角度（向き）成分 [rad] */

        /**
         * @brief コンストラクタ
         */
        constexpr Pose2D() = default;

        /**
         * @brief コンストラクタ 直交座標(_x, _y, _theta)で初期化
         */
        constexpr Pose2D(T _x, T _y, T _theta) : x(_x), y(_y), theta(_theta) {}

        /**
         * @brief コンストラクタ Vector2と角度の数値で初期化
         */
        constexpr Pose2D(const Vector2<T> &v, T _theta)
        {
            x = v.x;
            y = v.y;
            theta = _theta;
        }

        /**
         * @brief コンストラクタ 直交座標(_x, _y)で初期化（角度はゼロ）
         */
        constexpr Pose2D(T _x, T _y) : x(_x), y(_y) {}

        /**
         * @brief コンストラクタ Vector2で初期化（角度はゼロ）
         */
        constexpr Pose2D(const Vector2<T> &v)
        {
            x = v.x;
            y = v.y;
        }

        /**
         * @brief 指定されたベクトルがこのベクトルと等しい場合にtrueを返す
         * @param v: 指定するベクトル
         */
        bool equals(const Pose2D &v)
        {
            return *this == v;
        }

        /**
         * @brief 直交座標形式でこのベクトルを設定
         * @param _x: 指定するベクトル
         * @param _y: 指定するベクトル
         * @param _theta: 指定するベクトル
         */
        void set(T _x, T _y, T _theta)
        {
            x = _x;
            y = _y;
            theta = _theta;
        }

        /**
         * @brief 極座標形式でこのベクトルを設定
         * @param r: 原点からの距離
         * @param angle: 原点との角度
         * @param robot_theta: ロボットの座標
         */
        void setByPolar(T r, T angle, T robot_theta)
        {
            x = r * std::cos(angle);
            y = r * std::sin(angle);
            theta = robot_theta;
        }

        /**
         * @brief このベクトルを原点中心にangle[rad]回転
         * @param angle: 回転させる角度[rad]
         */
        void rotate(T angle)
        {
            x = x * cos(angle) - y * sin(angle);
            y = x * sin(angle) + y * cos(angle);
        }

        /**
         * @brief 指定座標中心(rot_x, rot_y)に回転
         * @param rot_x: 回転中心のx座標
         * @param rot_y: 回転中心のy座標
         * @param angle: 回転させる角度[rad]
         */
        void rotate(T rot_x, T rot_y, T angle)
        {
            Vector2<T> p(rot_x, rot_y);
            rotate(p, angle);
        }

        /**
         * @brief 座標oを中心にangleだけ回転
         * @param o: 回転中心の座標
         * @param angle: 回転させる角度[rad]
         */
        void rotate(Vector2<T> o, T angle)
        {
            Vector2<T> p(x - o.x, y - o.y);
            p.rotate(angle);
            p.x += o.x;
            p.y += o.y;
            x = p.x;
            y = p.y;
        }

        /**
         * @brief このベクターをフォーマットした文字列を返す
         * @return フォーマットした文字列
         */
        std::string toString()
        {
            return '(' + std::to_string(x) + ", " + std::to_string(y) + ')';
        }

        /**
         * @brief このベクトルの長さを返す
         * @return このベクトルの長さ
         */
        T length() const
        {
            return magnitude();
        }

        /**
         * @brief このベクトルの長さを返す
         * @return このベクトルの長さ
         */
        T magnitude() const
        {
            return std::sqrt(sqrMagnitude());
        }

        /**
         * @brief このベクトルの長さの2乘を返す
         * @return このベクトルの長さ2乘
         */
        constexpr T sqrLength() const
        {
            return sqrMagnitude();
        }

        /**
         * @brief このベクトルの長さの2乘を返す
         * @return このベクトルの長さ2乘
         */
        constexpr T sqrMagnitude() const
        {
            return x * x + y * y;
        }

        /**
         * @brief 2つのベクトルの内積を返す
         * @param a: 1つ目のベクトル
         * @param b: 2つ目のベクトル
         * @return 2つのベクトルの内積
         */
        static T getDot(Pose2D a, Pose2D b)
        {
            return (a.x * b.x + a.y * b.y);
        }

        /**
         * @brief 2つのベクトルのなす角を弧度法で返す
         * @param a: 1つ目のベクトル
         * @param b: 2つ目のベクトル
         * @return 2つのベクトルのなす角[rad]
         */
        static T getAngle(Pose2D a, Pose2D b)
        {
            return std::atan2(b.y - a.y, b.x - a.x);
        }

        /**
         * @brief 2つのベクトルの距離を返す
         * @param a: 1つ目のベクトル
         * @param b: 2つ目のベクトル
         * @return 2つのベクトルの距離を返す
         */
        static T getDistance(Pose2D a, Pose2D b)
        {
            Pose2D v = (b - a);
            return v.magnitude();
        }

        /**
         * @brief ベクトルaとbの間をtで線形補間
         * @param a: 1つ目のベクトル
         * @param b: 2つ目のベクトル
         * @param t: 媒介変数
         * @return 補間点
         */
        static Pose2D leap(Pose2D a, Pose2D b, T t)
        {
            t = guard(t, 0.0, 1.0);
            Pose2D v = a;
            v.x += (b.x - a.x) * t;
            v.y += (b.y - a.y) * t;
            v.theta += (b.theta - a.theta) * t;
            return v;
        }

        /**
         * @brief 全ての要素にスカラ加算
         */
        constexpr Pose2D operator+() const
        {
            return *this;
        }

        /**
         * @brief 全ての要素にスカラ減算
         */
        constexpr Pose2D operator-() const
        {
            return {-x, -y, -theta};
        }

        /**
         * @brief ベクトルの要素同士の和
         */
        constexpr Pose2D operator+(const Pose2D &v) const
        {
            return {x + v.x, y + v.y, theta + v.theta};
        }

        /**
         * @brief ベクトルの要素同士の差
         */
        constexpr Pose2D operator-(const Pose2D &v) const
        {
            return {x - v.x, y - v.y, theta - v.theta};
        }

        /**
         * @brief 全ての要素にスカラ乗算
         * @attention ベクトル同士の乗算は未定義，内積の計算はgetDot()を使用
         */
        constexpr Pose2D operator*(T s) const
        {
            return {x * s, y * s, theta * s};
        }

        /**
         * @brief 全ての要素にスカラ除算
         * @attention ベクトル同士の除算は未定義
         */
        constexpr Pose2D operator/(T s) const
        {
            return {x / s, y / s, theta / s};
        }

        /**
         * @brief ベクトルの要素同士の和を代入（スカラとの和の場合は全ての要素に対して加算）
         */
        Pose2D &operator+=(const Pose2D &v)
        {
            x += v.x;
            y += v.y;
            theta += v.theta;
            return *this;
        }

        /**
         * @brief ベクトルの要素同士の差を代入（スカラとの和の場合は全ての要素に対して減算）
         */
        Pose2D &operator-=(const Pose2D &v)
        {
            x -= v.x;
            y -= v.y;
            theta -= v.theta;
            return *this;
        }

        /**
         * @brief 全ての要素に対してスカラ乗算して代入（ベクトル同士の乗算は未定義）
         */
        Pose2D &operator*=(T s)
        {
            x *= s;
            y *= s;
            theta *= s;
            return *this;
        }

        /**
         * @brief 全ての要素に対してスカラ除算して代入（ベクトル同士の除算は未定義）
         */
        Pose2D &operator/=(T s)
        {
            x /= s;
            y /= s;
            theta /= s;
            return *this;
        }

        /**
         * @brief 2つのベクトルが等しい場合にtrueを返す
         */
        bool operator==(const Pose2D &v) const
        {
            return ((x == v.x) && (y == v.y) && (theta == v.y));
        }

        /**
         * @brief 2つのベクトルが等しい場合にfalseを返す
         */
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
