# Vector2

## Constructors
| 名前 | 機能 |
| ---- | ---- |
| Vector2() | ゼロベクトルで初期化 |
| Vector2(double _x, double _y) | 直交座標で初期化 |


## Public Functions
| 名前 | 機能 |
| ---- | ---- |
| bool equals(const Vector2 &v) | 指定されたベクトルがこのベクトルと等しい場合にtrueを返す |
| void normalize() | このベクトルの大きさを1にする |
| void rotate(double angle) | このベクトルを原点中心にangle[rad]回転 |
| void rotate(double rot_x, double rot_y, double angle) | 指定座標中心(rot_x, rot_y)に回転 |
| void rotate(Vector2 o, double angle) | 座標oを中心にangleだけ回転 |
| void set(double _x, double _y) | 直交座標形式でこのベクトルを設定 |
| void setByPolar(double r, double angle) | 極座標形式でこのベクトルを設定 |
| std::string toString() | このベクターをフォーマットした文字列を返す |
| double length() const | このベクトルの長さを返す |
| double magnitude() const | このベクトルの長さを返す |
| Vector2 normalized() const | 大きさが1のこのベクトルを返す |
| constexpr double lengthSquare() const | このベクトルの長さの2乗を返す |
| constexpr double lengthMagnitude() const | このベクトルの長さの2乗を返す |

## Static Functions
| 名前 | 機能 |
| ---- | ---- |
| static double getDot(Vector2 a, Vector2 b) | 2つのベクトルの内積を返す |
| static double getAngle(Vector2 a, Vector2 b) | 2つのベクトルのなす角を弧度法で返す |
| static double getDistance(Vector2 a, Vector2 b) | 二つのベクトルの距離を返す |
| static Vector2 leap(Vector2 a, Vector2 b, double t) | ベクトルaとbの間をtで線形補間 |

## Operators
| 名前 | 機能 |
| ---- | ---- |
| + | ベクトルの要素同士の和（スカラとの和の場合は全ての要素に対して加算） |
| - | ベクトルの要素同士の差（スカラとの和の場合は全ての要素に対して減算） |
| * | 全ての要素に対してスカラ乗算（ベクトル同士の乗算は未定義，内積の計算はgetDot()を使用） |
| / | 全ての要素に対してスカラ除算（ベクトル同士の除算は未定義） |
| += | ベクトルの要素同士の和を代入（スカラとの和の場合は全ての要素に対して加算） |
| -= | ベクトルの要素同士の差を代入（スカラとの和の場合は全ての要素に対して減算） |
| *= | 全ての要素に対してスカラ乗算して代入（ベクトル同士の乗算は未定義） |
| /= | 全ての要素に対してスカラ除算して代入（ベクトル同士の除算は未定義） |
| == | 2つのベクトルが等しい場合にtrueを返す |
| != | 2つのベクトルが等しい場合にfalseを返す |
| << | 出力ストリーム |
| >> | 入力ストリーム |