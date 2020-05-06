/**
 * @file PurePursuitControl.h
 * @brief PurePursuit制御（単純追従制御）
**/
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
    /**
     * @brief PurePursuit制御（単純追従制御）
    **/
    template <typename T, typename T_fbc>
    class PurePursuitControl
    {
    public:
        /**
         * @brief モードリスト
         */
        enum class Mode
        {
            diff = 0, /**< 2DoF（差動二輪型） */
            omni      /**< 3DoF（全方位移動型） */
        };

        /**
         * @brief パラメータ構造体
         */
        struct param_t
        {
            Mode mode;         /**< モード */
            T_fbc fbc_linear;  /**< 並進用のフィードバックコントローラ */
            T_fbc fbc_angular; /**< 回転用のフィードバックコントローラ */
        };

        /**
         * @brief コンストラクタ
         */
        PurePursuitControl() = default;

        /**
         * @brief コンストラクタ 経路データで初期化
         * @param path: 経路データ
         */
        PurePursuitControl(std::vector<Pose2D<T>> path) { set(path); }

        /**
         * @brief 経路データの設定
         * @param path: 経路データ
         */
        inline void setPath(const std::vector<Pose2D<T>> path);

        /**
         * @brief パラメータの設定
         * @param param: パラメータ構造体
         */
        inline void setParam(const PurePursuitControl::param_t param) { _param = param; };

        /**
         * @brief モードの設定
         * @param mode: モードリスト
         */
        inline void setMode(const PurePursuitControl::Mode mode) { _param.mode = mode; };

        /**
         * @brief 追従用フィードバックコントローラの設定
         * @param fbc_linear: 並進用のフィードバックコントローラ
         * @param fbc_angular: 回転用のフィードバックコントローラ
         */
        inline void setController(T_fbc fbc_linear, T_fbc fbc_angular);

        /**
         * @brief 経路データを末尾に追加
         * @param path: 経路データ
         */
        inline void push_back(std::vector<Pose2D<T>> path);

        /**
         * @brief 経路データの末尾に座標を追加
         * @param pose: 座標
         */
        inline void push_back(Pose2D<T> pose) { _path.push_back(pose); }

        /**
         * @brief 値の更新
         * @param target: 経路データのインデックス
         * @param now_val: 現在値
         * @param dt: 前回この関数をコールしてからの経過時間
         */
        inline void update(int idx, myStd::Pose2D<T> now_pose, T dt);

        /**
         * @brief 制御量（計算結果）の取得
         * @return 制御量（計算結果）
         * @attention update()を呼び出さないと値は更新されない
         */
        inline Pose2D<T> getControlVal() { return output; }

    private:
        param_t _param;
        Pose2D<T> output;
        std::vector<Pose2D<T>> _path; // 通過点のリスト

    }; // namespace myStd

    template <typename T, typename T_fbc>
    inline void PurePursuitControl<T, T_fbc>::setPath(std::vector<Pose2D<T>> path)
    {
        _path.clear();
        push_back(path);
    }

    template <typename T, typename T_fbc>
    inline void PurePursuitControl<T, T_fbc>::push_back(std::vector<Pose2D<T>> path)
    {
        for (auto p : path)
            _path.push_back(p);
    }

    template <typename T, typename T_fbc>
    inline void PurePursuitControl<T, T_fbc>::setController(T_fbc fbc_linear, T_fbc fbc_angular)
    {
        _param.fbc_linear = fbc_linear;
        _param.fbc_angular = fbc_angular;
    }

    template <typename T, typename T_fbc>
    inline void PurePursuitControl<T, T_fbc>::update(int idx, myStd::Pose2D<T> now_pose, T dt)
    {
        T distance = Pose2D<T>::getDistance(_path[idx], now_pose);
        _param.fbc_linear.update(0, distance, dt);
        output.x = _param.fbc_linear.getControlVal();

        T angle = Pose2D<T>::getAngle(_path[idx], now_pose);
        _param.fbc_angular.update(0, angle, dt);
        output.theta = _param.fbc_angular.getControlVal();
    }

} // namespace myStd
#endif // PurePursuitControl_h
