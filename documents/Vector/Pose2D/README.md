# Pose2D

## Parameters
| 名前 | 機能 |
| ---- | ---- |
| x | 2次元直交座標におけるx成分 |
| y | 2次元直交座標におけるy成分 |
| theta | 2次元直交座標における角度（向き）成分 [rad] |


## Constructors
| 名前 | 機能 |
| ---- | ---- |
| Pose2D() | ゼロベクトルで初期化 |
| Pose2D(double _x, double _y, double _theta) | 直交座標(_x, _y, _theta)で初期化 |
| Pose2D(const Pose2D &v, double _theta) | Vector2と角度の数値で初期化 |
| Pose2D(double _x, double _y) | 直交座標(_x, _y)で初期化（角度はゼロ） |
| Pose2D(const Pose2D &v) | Vector2で初期化（角度はゼロ） |


## Public Functions
| 名前 | 機能 |
| ---- | ---- |
| bool equals(const Pose2D &v) | 指定されたベクトルがこのベクトルと等しい場合にtrueを返す |
| void rotate(double angle) | このベクトルを原点中心にangle[rad]回転 |
| void rotate(double rot_x, double rot_y, double angle) | 指定座標中心(rot_x, rot_y)に回転 |
| void rotate(Vector2 o, double angle) | 座標oを中心にangleだけ回転 |
| void set(double _x, double _y, double _theta) | 直交座標形式でこのベクトルを設定 |
| void setByPolar(double r, double angle, double robot_theta) | 極座標形式でこのベクトルを設定 |
| std::string toString() | このベクターをフォーマットした文字列を返す |
| double length() const | このベクトルの長さを返す |
| double magnitude() const | このベクトルの長さを返す |
| constexpr double lengthSquare() const | このベクトルの長さの2乗を返す |
| constexpr double lengthMagnitude() const | このベクトルの長さの2乗を返す |

## Static Functions
| 名前 | 機能 |
| ---- | ---- |
| static double getDot(Vector2 a, Vector2 b) | 2つのベクトルの内積を返す |
| static double getAngle(Vector2 a, Vector2 b) | 2つのベクトルのなす角を弧度法で返す |
| static double getDistance(Vector2 a, Vector2 b) | 二つのベクトルの距離を返す |
| static Pose2D leap(Pose2D a, Pose2D b, double t) | ベクトルaとbの間をtで線形補間 |


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
