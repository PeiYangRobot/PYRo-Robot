#include "pyro_wl_forward_kf.h"
#include "wl_config.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

void wl_forward_kf_t::init()
{
    if (_kf.is_initialized())
        return;

    const float t = FORWARD_KF_TS;
    const float half_t2 = 0.5f * t * t;
    F_ = {
        1.0f, t, half_t2, 0.0f,
        0.0f, 1.0f, t, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };
    H_ = {
        0.0f, 0.0f, 1.0f, 1.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
    };
    filter_t::options_t options;
    options.covariance_update = filter_t::covariance_update_t::JOSEPH;
    options.solver            = filter_t::solver_t::INVERSE;
    options.symmetrize        = true;
    filter_t::covariance_t P0{};
    const float g_a[4] = {half_t2, t, 1.0f, 0.0f};
    for (uint8_t row = 0; row < 4; ++row)
    {
        options.min_variance[row] = FORWARD_KF_MIN_VARIANCE[row];
        P0(row, row) = FORWARD_KF_P_INIT[row];
        for (uint8_t col = 0; col < 4; ++col)
            Q_d_(row, col) = FORWARD_KF_ACCEL_VARIANCE * g_a[row] * g_a[col];
    }
    Q_d_(3, 3) = FORWARD_KF_BIAS_VARIANCE;
    R_(0, 0) = FORWARD_KF_R_IMU;
    R_(1, 1) = FORWARD_KF_R_WHEEL;
    _kf.init({}, P0, options);
    // 与旧 init 的零速 reset 一致，首个有效轮速仍参与微分。
    // Match the old zero-speed reset: differentiate the first valid wheel sample.
    _wheel_valid = _wheel_was_valid = true;
}

ForwardStateEstimate wl_forward_kf_t::reset(float x, float v_wheel, float a_imu,
                                           bool wheel_valid)
{
    init();
    _wheel_valid = wheel_valid && matrix::is_finite(v_wheel);
    _wheel_was_valid = _wheel_valid;
    _previous_wheel_v = _wheel_valid ? v_wheel : 0.0f;
    _wheel_accel = 0.0f;
    _slip_score = 0.0f;
    const filter_t::state_t x0 = {x, _previous_wheel_v, a_imu, 0.0f};
    filter_t::covariance_t P0{};
    for (uint8_t i = 0; i < 4; ++i)
        P0(i, i) = FORWARD_KF_P_INIT[i];
    R_(1, 1) = FORWARD_KF_R_WHEEL;
    _kf.reset(x0, P0);
    return _get_estimate();
}

ForwardStateEstimate wl_forward_kf_t::update(float a_imu, float v_wheel,
                                            bool wheel_valid)
{
    init();
    _wheel_valid = wheel_valid && matrix::is_finite(v_wheel);
    if (_wheel_valid && _wheel_was_valid)
        _wheel_accel = FORWARD_KF_WHEEL_DIFF_A * _wheel_accel +
                      FORWARD_KF_WHEEL_DIFF_B * (v_wheel - _previous_wheel_v);
    else
        _wheel_accel = 0.0f;
    if (_wheel_valid)
        _previous_wheel_v = v_wheel;
    _wheel_was_valid = _wheel_valid;

    const filter_t::measurement_t z = {a_imu, _wheel_valid ? v_wheel : 0.0f};
    if (_kf.predict(F_, Q_d_) != PYRO_OK)
        return _get_estimate();
    _adjust_wheel_r(z);
    // 仅屏蔽失效编码器；保留合法零速和 IMU 量测，用策略限制零偏学习。
    // Mask only invalid encoders; retain valid zero speed/IMU, and gate bias learning.
    const filter_t::measurement_mask_t mask = {true, _wheel_valid};
    _kf.correct(z, H_, R_, &mask,
                [this](filter_t::correction_t &correction)
                { return _freeze_bias_update(correction); });
    return _get_estimate();
}

void wl_forward_kf_t::_adjust_wheel_r(const filter_t::measurement_t &z)
{
    float slip_target = 0.0f;
    if (_wheel_valid)
    {
        const auto &x_pred = _kf.get_state();
        const auto &P_pred = _kf.get_covariance();
        const float r_v    = z[1] - x_pred[1];
        // 单条轮速量测的 NIS 使用基础 R，不使用本拍放大值或整体 NIS。
        // Wheel-only NIS uses baseline R, not this cycle's inflated R or joint NIS.
        const float nis = r_v * r_v / (P_pred(1, 1) + FORWARD_KF_R_WHEEL);
        // 预测零偏避免将本拍轮速校正反馈到滑移判断。
        // Prior bias avoids feeding the current wheel correction back into gating.
        const float a_imu_corrected = z[0] - x_pred[3];
        const float r_wi = _wheel_accel - a_imu_corrected;
        const float g_v = std::clamp(
            (nis - FORWARD_KF_SLIP_NIS_LOW) /
                (FORWARD_KF_SLIP_NIS_HIGH - FORWARD_KF_SLIP_NIS_LOW),
            0.0f, 1.0f);
        const float g_a = std::clamp(
            (std::fabs(r_wi) - FORWARD_KF_SLIP_ACCEL_LOW) /
                (FORWARD_KF_SLIP_ACCEL_HIGH - FORWARD_KF_SLIP_ACCEL_LOW),
            0.0f, 1.0f);
        slip_target = 0.5f * (g_v + g_a);
    }
    const float tau = slip_target > _slip_score
                          ? FORWARD_KF_SLIP_RISE_TAU
                          : FORWARD_KF_SLIP_FALL_TAU;
    const float alpha = FORWARD_KF_TS / (tau + FORWARD_KF_TS);
    _slip_score += alpha * (slip_target - _slip_score);
    R_(1, 1) = FORWARD_KF_R_WHEEL *
        (1.0f + _slip_score * (FORWARD_KF_WHEEL_R_MULTIPLIER - 1.0f));
}

status_t wl_forward_kf_t::_freeze_bias_update(filter_t::correction_t &correction) const
{
    if (!_wheel_valid || _slip_score >= FORWARD_KF_BIAS_FREEZE_SCORE)
    {
        // IMU 单独只能观测 a+bias；冻结增益行，不能只在校正后改回零偏状态。
        // IMU alone observes a+bias; freeze the gain row, not just the corrected state.
        correction.K(3, 0) = 0.0f;
        correction.K(3, 1) = 0.0f;
    }
    return PYRO_OK;
}

ForwardStateEstimate wl_forward_kf_t::_get_estimate() const
{
    const auto &x = _kf.get_state();
    return {x[0], x[1], x[2], x[3], _slip_score};
}

} // namespace pyro
