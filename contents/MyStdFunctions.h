#ifndef MyStdFunctions_h
#define MyStdFunctions_h

#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <string>

constexpr double PI = 3.1415926535897932384626433832795;
constexpr double HALF_PI = PI / 2.0;
constexpr double TWO_PI = PI * 2.0;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double EULER = 2.718281828459045235360287471352;

constexpr double GRAVITY = 9.807;
constexpr double Nm2gfm = (1 / GRAVITY);
constexpr double gfm2Nm = GRAVITY;

constexpr double mNm2gfcm = (Nm2gfm * 100);
constexpr double gfcm2mNm = (gfm2Nm / 100);

//#define min(a,b) ((a)<(b)?(a):(b))
using std::min;

//#define max(a,b) ((a)>(b)?(a):(b))
using std::max;

//#define abs(x) ((x)>0?(x):-(x))
using std::abs;

//#define round(x)     ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
using std::round;

// std::pow(x, 2)
template <typename T>
static inline T sq(T x)
{
    return (x) * (x);
}

template <typename T>
static inline T constrain(T x, T min, T max)
{
    return ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)));
}

template <typename T>
static inline T guard(T x, T min, T max)
{
    return constrain<T>(x, min, max);
}

template <typename T>
static inline double radians(T deg)
{
    return (deg * DEG_TO_RAD);
}

template <typename T>
static inline double degrees(T rad)
{
    return (rad * RAD_TO_DEG);
}

template <typename T>
static inline uint8_t lowByte(T word)
{
    return (word & 0xff);
}

template <typename T>
static inline uint8_t highByte(T word)
{
    return (word >> 8);
}

template <typename T>
static inline T bitRead(T value, int bit)
{
    return ((value >> bit) & 0x01);
}

template <typename T>
static inline T bitSet(T value, int bit)
{
    return (value |= (1UL << bit));
}

template <typename T>
static inline T bitClear(T value, int bit)
{
    return (value &= ~(1UL << bit));
}

template <typename T>
static inline T bitWrite(T value, int bit, int bitvalue)
{
    return (bitvalue ? bitSet(value, bit) : bitClear(value, bit));
}

template <typename T>
static inline T bitShift(int bit)
{
    return (1UL << bit);
}

template <typename T>
static inline int signOf(T x)
{
    return (x > 0 ? 1 : x < 0 ? -1 : 0);
}

template <typename T>
static inline T map(T x, T in_min, T in_max, T out_min, T out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline double leap(double a, double b, double t)
{
    t = guard(t, 0.0, 1.0);
    return (a + (b - a) * t);
}

static inline double leapUnclamped(double a, double b, double t)
{
    return (a + (b - a) * t);
}

static inline double normalizeAnglePositive(double angle)
{
    return fmod(fmod(angle, 2.0 * PI) + 2.0 * PI, 2.0 * PI);
}

static inline double normalizeAngle(double angle)
{
    double a = normalizeAnglePositive(angle);
    if (a > PI)
        a -= 2.0 * PI;
    return a;
}

static inline double shortestAngularDistance(double from, double to)
{
    return normalizeAngle(to - from);
}

static inline double normalizeAbs90deg(double angle)
{
    return fmod(fmod(angle + HALF_PI, PI) + PI, PI) - HALF_PI;
}

template <typename T>
void dataShiftToLast(T new_data, T *data_array, int num)
{
    for (int i = num - 2; i >= 0; i--)
        data_array[i + 1] = data_array[i];
    data_array[0] = new_data;
}

#endif // MyStdFunctions_h
