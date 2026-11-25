#ifndef __PYRO_WL_FORWARD_KF_H__
#define __PYRO_WL_FORWARD_KF_H__

#include "pyro_algo_lkf.h"

namespace pyro
{

struct ForwardStateEstimate
{
    float x;
    float v;
    float a;
    float imu_acc_bias;
    float slip_score;
};

class wl_forward_kf_t
{
  public:
    // 状态 [x,v,a,IMU 加速度零偏]；量测 [a_imu,v_wheel]，沿用 KF_ok 实测模型。
    // States [x,v,a,IMU acceleration bias]; measurements [a_imu,v_wheel], as in KF_ok.
    using filter_t = lkf_t<4, 2>;

    wl_forward_kf_t() = default;
    wl_forward_kf_t(const wl_forward_kf_t &) = delete;
    wl_forward_kf_t &operator=(const wl_forward_kf_t &) = delete;

    void init(); // 固定存储，重复初始化无操作 / Fixed storage; repeated init is a no-op.
    ForwardStateEstimate reset(float x, float v_wheel, float a_imu,
                               bool wheel_valid = true);
    ForwardStateEstimate update(float a_imu, float v_wheel,
                                bool wheel_valid = true);
    const filter_t &get_filter() const { return _kf; }
    const filter_t::measurement_noise_t &get_measurement_noise() const
    {
        return R_;
    }

  private:
    // 预测后调整本拍 R；进入 correct 前完成，不使用校正后的状态。
    // Adjust this cycle's R after prediction, before correction, using prior state.
    void _adjust_wheel_r(const filter_t::measurement_t &z);
    // 不可信轮速下冻结零偏增益行；状态与 Joseph 协方差使用同一个 K。
    // Freeze bias gain for untrusted wheels; state/Joseph updates use the same K.
    status_t _freeze_bias_update(filter_t::correction_t &correction) const;
    ForwardStateEstimate _get_estimate() const;

    filter_t _kf{};
    filter_t::transition_t F_{};        // 状态转移矩阵 / State transition.
    filter_t::observation_t H_{};       // 观测矩阵 / Observation matrix.
    filter_t::covariance_t Q_d_{};      // 每拍过程噪声协方差 / Process covariance per step.
    filter_t::measurement_noise_t R_{}; // 量测噪声协方差 / Measurement covariance.
    float _previous_wheel_v = 0.0f;
    float _wheel_accel = 0.0f;
    float _slip_score = 0.0f;
    bool _wheel_valid = false;
    bool _wheel_was_valid = false;
};

} // namespace pyro
#endif // __PYRO_WL_FORWARD_KF_H__
