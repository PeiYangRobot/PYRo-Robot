// Frozen KF_ok 19ea989 (tested forward filter); test-only, never linked into firmware.
#ifndef FORWARD_KF_REFERENCE_H
#define FORWARD_KF_REFERENCE_H

#include "pyro_core_def.h"
#include "kalman_filter.h"

namespace pyro
{

struct ForwardStateReference
{
    float x;
    float v;
    float a;
    float imu_acc_bias;
    float slip_score;
};

class wl_forward_kf_reference_t
{
  public:
    wl_forward_kf_reference_t() = default;
    wl_forward_kf_reference_t(const wl_forward_kf_reference_t &) = delete;
    wl_forward_kf_reference_t &operator=(const wl_forward_kf_reference_t &) = delete;

    void init(); // allocate once through the existing KF library
    ForwardStateReference reset(float x, float v_wheel, float a_imu,
                               bool wheel_valid = true);
    ForwardStateReference update(float a_imu, float v_wheel,
                                bool wheel_valid = true);
    const KalmanFilter_t &get_filter() const { return _kf; }

  private:
    static void _adjust_wheel_r(KalmanFilter_t *kf);
    static void _freeze_bias_update(KalmanFilter_t *kf);
    ForwardStateReference _get_estimate() const;

    KalmanFilter_t _kf{};
    float _previous_wheel_v = 0.0f;
    float _wheel_accel = 0.0f;
    float _slip_score = 0.0f;
    bool _wheel_valid = false;
    bool _wheel_was_valid = false;
};

} // namespace pyro
#endif // FORWARD_KF_REFERENCE_H
