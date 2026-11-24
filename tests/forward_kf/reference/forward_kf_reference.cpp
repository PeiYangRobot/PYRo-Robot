// Frozen KF_ok 19ea989 (tested forward filter); test-only, never linked into firmware.
#include "forward_kf_reference.h"
#include "forward_kf_config.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace pyro
{

void wl_forward_kf_reference_t::init()
{
    if (_kf.xhat_data != nullptr)
        return;

    Kalman_Filter_Init(&_kf, 4, 0, 2);
    _kf.UseAutoAdjustment = 0;
    _kf.UseJosephForm = 1;
    _kf.UserData = this;
    // Runs after xminus/Pminus, before the gain and measurement correction.
    _kf.User_Func2_f = _adjust_wheel_r;
    // Modify the gain before state/Joseph covariance correction.
    _kf.User_Func3_f = _freeze_bias_update;

    const float t = FORWARD_KF_TS;
    const float half_t2 = 0.5f * t * t;
    const float f_init[16] = {
        1.0f, t, half_t2, 0.0f,
        0.0f, 1.0f, t, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };
    const float h_init[8] = {
        0.0f, 0.0f, 1.0f, 1.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
    };
    std::memcpy(_kf.F_data, f_init, sizeof(f_init));
    std::memcpy(_kf.H_data, h_init, sizeof(h_init));
    const float g_a[4] = {half_t2, t, 1.0f, 0.0f};
    for (uint8_t row = 0; row < 4; ++row)
    {
        _kf.StateMinVariance[row] = FORWARD_KF_MIN_VARIANCE[row];
        for (uint8_t col = 0; col < 4; ++col)
            _kf.Q_data[row * 4 + col] =
                FORWARD_KF_ACCEL_VARIANCE * g_a[row] * g_a[col];
    }
    _kf.Q_data[15] = FORWARD_KF_BIAS_VARIANCE;
    _kf.R_data[0] = FORWARD_KF_R_IMU;
    _kf.R_data[3] = FORWARD_KF_R_WHEEL;
    reset(0.0f, 0.0f, 0.0f);
}

ForwardStateReference wl_forward_kf_reference_t::reset(float x, float v_wheel, float a_imu,
                                           bool wheel_valid)
{
    _wheel_valid = wheel_valid && std::isfinite(v_wheel);
    _wheel_was_valid = _wheel_valid;
    _previous_wheel_v = _wheel_valid ? v_wheel : 0.0f;
    _wheel_accel = 0.0f;
    _slip_score = 0.0f;
    _kf.xhat_data[0] = x;
    _kf.xhat_data[1] = _previous_wheel_v;
    _kf.xhat_data[2] = a_imu;
    _kf.xhat_data[3] = 0.0f;
    std::memcpy(_kf.xhatminus_data, _kf.xhat_data, 4 * sizeof(float));
    std::memcpy(_kf.FilteredValue, _kf.xhat_data, 4 * sizeof(float));
    std::memset(_kf.P_data, 0, 16 * sizeof(float));
    std::memset(_kf.MeasuredVector, 0, 2 * sizeof(float));
    for (uint8_t i = 0; i < 4; ++i)
        _kf.P_data[i * 4 + i] = FORWARD_KF_P_INIT[i];
    std::memcpy(_kf.Pminus_data, _kf.P_data, 16 * sizeof(float));
    _kf.H_data[5] = _wheel_valid ? 1.0f : 0.0f;
    _kf.R_data[3] = FORWARD_KF_R_WHEEL;
    _kf.MeasurementUpdateFailed = 0;
    _kf.MatStatus = ARM_MATH_SUCCESS;
    return _get_estimate();
}

ForwardStateReference wl_forward_kf_reference_t::update(float a_imu, float v_wheel,
                                            bool wheel_valid)
{
    _wheel_valid = wheel_valid && std::isfinite(v_wheel);
    if (_wheel_valid && _wheel_was_valid)
        _wheel_accel = FORWARD_KF_WHEEL_DIFF_A * _wheel_accel +
                      FORWARD_KF_WHEEL_DIFF_B * (v_wheel - _previous_wheel_v);
    else
        _wheel_accel = 0.0f;
    if (_wheel_valid)
        _previous_wheel_v = v_wheel;
    _wheel_was_valid = _wheel_valid;

    _kf.MeasuredVector[0] = a_imu;
    _kf.MeasuredVector[1] = _wheel_valid ? v_wheel : 0.0f;
    // Disable only a truly invalid encoder row; valid zero speed is retained.
    _kf.H_data[5] = _wheel_valid ? 1.0f : 0.0f;
    Kalman_Filter_Update(&_kf);
    return _get_estimate();
}

void wl_forward_kf_reference_t::_adjust_wheel_r(KalmanFilter_t *kf)
{
    auto &filter = *static_cast<wl_forward_kf_reference_t *>(kf->UserData);
    float slip_target = 0.0f;
    if (filter._wheel_valid)
    {
        const float r_v = kf->z_data[1] - kf->xhatminus_data[1];
        const float nis = r_v * r_v / (kf->Pminus_data[5] + FORWARD_KF_R_WHEEL);
        // Prior bias avoids feeding the current wheel correction back into gating.
        const float a_imu_corrected = kf->z_data[0] - kf->xhatminus_data[3];
        const float r_wi = filter._wheel_accel - a_imu_corrected;
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
    const float tau = slip_target > filter._slip_score
                          ? FORWARD_KF_SLIP_RISE_TAU
                          : FORWARD_KF_SLIP_FALL_TAU;
    const float alpha = FORWARD_KF_TS / (tau + FORWARD_KF_TS);
    filter._slip_score += alpha * (slip_target - filter._slip_score);
    kf->R_data[3] = FORWARD_KF_R_WHEEL *
        (1.0f + filter._slip_score * (FORWARD_KF_WHEEL_R_MULTIPLIER - 1.0f));
}

void wl_forward_kf_reference_t::_freeze_bias_update(KalmanFilter_t *kf)
{
    const auto &filter = *static_cast<wl_forward_kf_reference_t *>(kf->UserData);
    if (!filter._wheel_valid || filter._slip_score >= FORWARD_KF_BIAS_FREEZE_SCORE)
    {
        // Without trusted wheel speed, IMU observes only a + bias.
        // Joseph correction uses this same gain, retaining bias uncertainty.
        kf->K_data[6] = 0.0f;
        kf->K_data[7] = 0.0f;
    }
}

ForwardStateReference wl_forward_kf_reference_t::_get_estimate() const
{
    return {_kf.FilteredValue[0], _kf.FilteredValue[1], _kf.FilteredValue[2],
            _kf.FilteredValue[3], _slip_score};
}

} // namespace pyro
