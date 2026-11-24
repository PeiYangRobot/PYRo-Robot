// Replay identical inputs against frozen KF_ok 19ea989 and the generic-KF adaptation.
#ifdef FORWARD_KF_REFERENCE
#include "reference/forward_kf_reference.h"
#else
#include "pyro_wl_forward_kf.h"
#endif
#include "wl_config.h"
#include <cmath>
#include <cstdio>
#include <limits>

int main(int argc, char **argv)
{
    if (argc != 3)
        return 1;
    FILE *output = std::fopen(argv[1], "wb");
    if (!output)
        return 1;
#ifdef FORWARD_KF_REFERENCE
    pyro::wl_forward_kf_reference_t filter;
#else
    pyro::wl_forward_kf_t filter;
#endif
    filter.init();
    float true_v = 0.0f;
    const bool stress = argv[2][0] == 's';
    for (int i = 0; i < 30000; ++i)
    {
        const float t = i * pyro::FORWARD_KF_TS;
        float acceleration = i < 10000 && !stress ? 0.0f : 0.7f * std::sin(2.2f * t);
        true_v += acceleration * pyro::FORWARD_KF_TS;
        float imu = acceleration + (i >= 4000 || stress ? 0.35f : 0.0f);
        float wheel = true_v;
        bool valid = true;
        if (i == 4000 && !stress)
            filter.reset(0.0f, 0.0f, imu);
        if (stress)
        {
            wheel += 0.003f * std::sin(17.0f * t);
            if (i % 2500 < 30)
                wheel += 4.0f;
            const int phase = i % 4000;
            if (phase >= 1000 && phase < 1300)
                wheel = std::numeric_limits<float>::quiet_NaN();
            if (phase >= 1500 && phase < 1600)
            {
                wheel = 1000.0f;
                valid = false;
            }
            if (phase >= 1700 && phase < 1750)
                wheel = std::numeric_limits<float>::infinity();
            if (phase >= 2100 && phase < 2140)
            {
                imu = 8.0f;
            }
            if (i % 6000 == 0)
                filter.reset(0.1f, wheel, imu, valid);
        }
        // Repeated init must not change a running filter or its slip history.
        if (i % 257 == 0)
            filter.init();
#ifdef FORWARD_KF_REFERENCE
        const float prior_bias = filter.get_filter().xhat_data[3];
        const float prior_bias_variance = filter.get_filter().P_data[15];
#else
        const float prior_bias = filter.get_filter().get_state()[3];
        const float prior_bias_variance = filter.get_filter().get_covariance()(3, 3);
#endif
        const auto state = filter.update(imu, wheel, valid);
        const float fields[5] = {state.x, state.v, state.a, state.imu_acc_bias, state.slip_score};
        if (std::fwrite(fields, sizeof(float), 5, output) != 5)
            return 2;
        const auto &kf = filter.get_filter();
#ifdef FORWARD_KF_REFERENCE
        const float *P = kf.P_data;
        const float *K = kf.K_data;
        const float *R = kf.R_data;
        if (kf.MeasurementUpdateFailed || kf.MatStatus != ARM_MATH_SUCCESS)
            return 3;
#else
        const float *P = kf.get_covariance().data();
        const float *K = kf.get_gain().data();
        const float *R = filter.get_measurement_noise().data();
        const std::size_t count = valid && pyro::matrix::is_finite(wheel) ? 2 : 1;
        if (kf.get_diagnostics().status != pyro::PYRO_OK || kf.get_diagnostics().measurement_count != count)
            return 3;
#endif
        const bool freeze_bias = !valid || !std::isfinite(wheel) ||
                                 state.slip_score >= pyro::FORWARD_KF_BIAS_FREEZE_SCORE;
        if (freeze_bias && (K[6] != 0.0f || K[7] != 0.0f ||
                            state.imu_acc_bias != prior_bias || P[15] < prior_bias_variance))
            return 4;
        if (std::fwrite(P, sizeof(float), 16, output) != 16 ||
            std::fwrite(K, sizeof(float), 8, output) != 8 ||
            std::fwrite(R, sizeof(float), 4, output) != 4)
            return 2;
    }
    return std::fclose(output) != 0;
}
