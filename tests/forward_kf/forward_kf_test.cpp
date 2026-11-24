#include "pyro_wl_forward_kf.h"
#include "wl_config.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace pyro;

static void check(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

static void check_covariance(const wl_forward_kf_t::filter_t &kf)
{
    const auto &x = kf.get_state();
    const auto &P = kf.get_covariance();
    double lower[16]{};
    for (int row = 0; row < 4; ++row)
    {
        check(matrix::is_finite(x[row]), "finite state");
        check(P(row, row) >= kf.get_options().min_variance[row],
              "variance floor");
        for (int col = 0; col < 4; ++col)
            check(P(row, col) == P(col, row),
                  "symmetric P");
        for (int col = 0; col <= row; ++col)
        {
            double value = P(row, col);
            for (int i = 0; i < col; ++i)
                value -= lower[row * 4 + i] * lower[col * 4 + i];
            if (row == col)
            {
                check(value > 0.0, "positive definite covariance");
                lower[row * 4 + col] = std::sqrt(value);
            }
            else
                lower[row * 4 + col] = value / lower[col * 4 + col];
        }
    }
}

// Independent double-precision batch KF reference; no CMSIS calls.
struct ReferenceKF
{
    double x[4]{};
    double p[16]{};
    double f[16]{};
    double q[16]{};

    explicit ReferenceKF(const wl_forward_kf_t::filter_t &kf)
    {
        for (int i = 0; i < 4; ++i)
            x[i] = kf.get_state()[i];
        for (int i = 0; i < 16; ++i)
        {
            p[i] = kf.get_covariance()[i];
        }
        const double t = FORWARD_KF_TS;
        const double half_t2 = float(0.5f * FORWARD_KF_TS * FORWARD_KF_TS);
        const double model[16] = {1,t,half_t2,0, 0,1,t,0, 0,0,1,0, 0,0,0,1};
        std::copy_n(model, 16, f);
        const float g_a[4] = {float(half_t2), FORWARD_KF_TS, 1.0f, 0.0f};
        for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 4; ++col)
                q[row * 4 + col] = FORWARD_KF_ACCEL_VARIANCE * g_a[row] * g_a[col];
        q[15] = FORWARD_KF_BIAS_VARIANCE;
    }

    void update(const double z[2], const float *h, const float *r, bool freeze_bias)
    {
        double xm[4]{}, pm[16]{}, gain[8]{}, aug[2][4]{};
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
                xm[i] += f[i * 4 + j] * x[j];
            for (int j = 0; j < 4; ++j)
            {
                pm[i * 4 + j] = q[i * 4 + j];
                for (int a = 0; a < 4; ++a)
                    for (int b = 0; b < 4; ++b)
                        pm[i * 4 + j] +=
                            f[i * 4 + a] * p[a * 4 + b] * f[j * 4 + b];
            }
        }
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
            {
                aug[i][j]     = r[i * 2 + j];
                aug[i][j + 2] = i == j ? 1.0 : 0.0;
                for (int a = 0; a < 4; ++a)
                    for (int b = 0; b < 4; ++b)
                        aug[i][j] +=
                            h[i * 4 + a] * pm[a * 4 + b] * h[j * 4 + b];
            }
        for (int c = 0; c < 2; ++c)
        {
            int pivot = c;
            for (int row = c + 1; row < 2; ++row)
                if (std::abs(aug[row][c]) > std::abs(aug[pivot][c]))
                    pivot = row;
            for (int j = 0; j < 4; ++j)
                std::swap(aug[c][j], aug[pivot][j]);
            const double diagonal = aug[c][c];
            check(std::abs(diagonal) > 1.0e-15, "reference innovation inverse");
            for (int j = 0; j < 4; ++j)
                aug[c][j] /= diagonal;
            for (int row = 0; row < 2; ++row)
                if (row != c)
                {
                    const double scale = aug[row][c];
                    for (int j = 0; j < 4; ++j)
                        aug[row][j] -= scale * aug[c][j];
                }
        }
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 2; ++j)
                for (int a = 0; a < 4; ++a)
                    for (int b = 0; b < 2; ++b)
                        gain[i * 2 + j] +=
                            pm[i * 4 + a] * h[b * 4 + a] * aug[b][j + 2];
        if (freeze_bias)
            gain[6] = gain[7] = 0.0;
        double residual[2]{}, transform[16]{};
        for (int j = 0; j < 2; ++j)
        {
            residual[j] = z[j];
            for (int a = 0; a < 4; ++a)
                residual[j] -= h[j * 4 + a] * xm[a];
        }
        for (int i = 0; i < 4; ++i)
        {
            x[i] = xm[i];
            for (int j = 0; j < 2; ++j)
                x[i] += gain[i * 2 + j] * residual[j];
            for (int j = 0; j < 4; ++j)
            {
                transform[i * 4 + j] = i == j ? 1.0 : 0.0;
                for (int a = 0; a < 2; ++a)
                    transform[i * 4 + j] -= gain[i * 2 + a] * h[a * 4 + j];
            }
        }
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
            {
                double value = 0.0;
                for (int a = 0; a < 4; ++a)
                    for (int b = 0; b < 4; ++b)
                        value += transform[i * 4 + a] * pm[a * 4 + b] *
                                 transform[j * 4 + b];
                for (int a = 0; a < 2; ++a)
                    for (int b = 0; b < 2; ++b)
                        value +=
                            gain[i * 2 + a] * r[a * 2 + b] * gain[j * 2 + b];
                p[i * 4 + j] = value;
            }
        for (int i = 0; i < 4; ++i)
            p[i * 4 + i] =
                std::max(p[i * 4 + i], double(FORWARD_KF_MIN_VARIANCE[i]));
    }
};

int main()
{
    wl_forward_kf_t filter;
    filter.init();
    const auto &kf = filter.get_filter();
    using filter_t = wl_forward_kf_t::filter_t;
    static_assert(filter_t::state_t::rows == 4 && filter_t::measurement_t::rows == 2);
    check(kf.get_options().covariance_update == filter_t::covariance_update_t::JOSEPH &&
          kf.get_options().solver == filter_t::solver_t::INVERSE, "KF modes");
    const auto state_before_init = kf.get_state();
    const auto P_before_init = kf.get_covariance();
    filter.init();
    check(kf.get_state() == state_before_init && kf.get_covariance() == P_before_init,
          "repeated init retains state and covariance");

    for (int i = 0; i < 20000; ++i)
    {
        const auto state = filter.update(0.0f, 0.0f);
        check(state.x == 0.0f && state.v == 0.0f && state.a == 0.0f &&
                  state.imu_acc_bias == 0.0f && state.slip_score == 0.0f,
              "legitimate zero measurements are retained");
        check_covariance(kf);
    }
    std::puts("PASS: stationary zero measurements and covariance, 20000 steps");

    filter.reset(0.0f, 0.0f, 0.4f);
    ForwardStateEstimate state{};
    for (int i = 0; i < 15000; ++i)
        state = filter.update(0.4f, 0.0f);
    check(std::abs(state.imu_acc_bias - 0.4f) < 0.003f &&
              std::abs(state.a) < 0.003f && std::abs(state.v) < 0.003f &&
              state.slip_score < 0.001f,
          "static bias separation");
    std::puts("PASS: IMU bias convergence");

    // Match KF_ok: neither slip nor wheel loss can train bias from IMU alone.
    const float learned_bias = state.imu_acc_bias;
    const float learned_variance = kf.get_covariance()(3, 3);
    state = filter.update(0.4f, 3.0f);
    check(state.slip_score >= FORWARD_KF_BIAS_FREEZE_SCORE &&
              state.imu_acc_bias == learned_bias && kf.get_gain()(3, 0) == 0.0f &&
              kf.get_gain()(3, 1) == 0.0f && kf.get_covariance()(3, 3) >= learned_variance,
          "slip preserves learned bias and Joseph bias uncertainty");
    for (int i = 0; i < 200; ++i)
    {
        state = filter.update(1.2f, 3.0f, false);
        check(state.imu_acc_bias == learned_bias && kf.get_gain()(3, 0) == 0.0f &&
                  kf.get_gain()(3, 1) == 0.0f,
              "offline wheel cannot train bias from IMU alone");
        check_covariance(kf);
    }
    check(state.a > 0.7f, "IMU still corrects acceleration while bias is frozen");
    std::puts("PASS: bias freeze during slip and encoder outage");

    filter.reset(0.0f, 0.0f, 0.35f);
    ReferenceKF reference(kf);
    const float H[8] = {0,0,1,1, 0,1,0,0};
    double true_x = 0.0, true_v = 0.0, max_difference = 0.0;
    for (int i = 1; i <= 30000; ++i)
    {
        const double t = i * double(FORWARD_KF_TS);
        const float acceleration =
            float(0.7 * std::sin(2.0 * 3.141592653589793 * 0.35 * t));
        true_x += true_v * FORWARD_KF_TS +
                  0.5 * acceleration * FORWARD_KF_TS * FORWARD_KF_TS;
        true_v += acceleration * FORWARD_KF_TS;
        const float imu   = acceleration + 0.35f;
        const float wheel = float(true_v);
        state             = filter.update(imu, wheel);
        const double z[2] = {imu, wheel};
        reference.update(z, H, filter.get_measurement_noise().data(),
                         state.slip_score >= FORWARD_KF_BIAS_FREEZE_SCORE);
        for (int j = 0; j < 4; ++j)
            max_difference = std::max(
                max_difference, std::abs(kf.get_state()[j] - reference.x[j]));
        check_covariance(kf);
        check(kf.get_diagnostics().status == PYRO_OK &&
              kf.get_diagnostics().math_status == ARM_MATH_SUCCESS,
              "CMSIS update success");
    }
    check(max_difference < 0.002 && std::abs(state.v - true_v) < 0.02 &&
              std::abs(state.x - true_x) < 0.04 &&
              std::abs(state.imu_acc_bias - 0.35f) < 0.02,
          "moving trajectory and independent double KF reference");
    std::printf("PASS: moving trajectory and double KF, max difference %.6g, "
                "30000 steps\n",
                max_difference);

    filter.reset(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 2000; ++i)
        filter.update(0.0f, 0.0f);
    const float prior_gain = kf.get_gain()(1, 1);
    state                  = filter.update(0.0f, 3.0f);
    check(state.slip_score > 0.0f && filter.get_measurement_noise()(1, 1) > FORWARD_KF_R_WHEEL &&
              kf.get_gain()(1, 1) < prior_gain,
          "wheel slip lowers current wheel gain");
    const float slip_step_v     = state.v;
    const float slip_step_score = state.slip_score;
    const float expected_score = FORWARD_KF_TS / (FORWARD_KF_SLIP_RISE_TAU + FORWARD_KF_TS);
    const float expected_multiplier = 1.0f + expected_score * (FORWARD_KF_WHEEL_R_MULTIPLIER - 1.0f);
    check(std::abs(slip_step_score - expected_score) < 1.0e-6f &&
              std::abs(filter.get_measurement_noise()(1, 1) / FORWARD_KF_R_WHEEL - expected_multiplier) < 1.0e-4f &&
              std::abs(slip_step_v) < 0.01f * 3.0f * prior_gain,
          "one-tick slip correction below one percent of normal wheel correction");

    // Return to true zero wheel speed after the single-tick pulse. The wheel
    // differentiator retains some evidence, then score must recover slowly.
    float peak_score     = slip_step_score;
    float previous_score = slip_step_score;
    for (int i = 0; i < 100; ++i)
    {
        previous_score = state.slip_score;
        state          = filter.update(0.0f, 0.0f);
        peak_score     = std::max(peak_score, state.slip_score);
        check(state.slip_score >= 0.0f && state.slip_score <= 1.0f,
              "bounded asymmetric slip score");
        check_covariance(kf);
    }
    check(state.slip_score > 0.0f && state.slip_score < peak_score &&
              std::abs(state.slip_score - previous_score * (30.0f / 31.0f)) <
                  1.0e-6f &&
              filter.get_measurement_noise()(1, 1) > FORWARD_KF_R_WHEEL,
          "slip pulse retains a 30 ms recovery tail");
    check(state.slip_score < FORWARD_KF_BIAS_FREEZE_SCORE &&
              (kf.get_gain()(3, 0) != 0.0f || kf.get_gain()(3, 1) != 0.0f),
          "bias learning resumes after wheel confidence recovers");
    std::printf("PASS: one-tick slip, 2 ms rise and 30 ms recovery, first "
                "score %.4f, tail %.4f\n",
                slip_step_score, state.slip_score);
    // KF_ok has no model confidence gate: wheel/IMU disagreement still adapts R.
    filter.reset(0.0f, 0.0f, 0.0f);
    state = filter.update(8.0f, 3.0f);
    check(state.slip_score > 0.0f && filter.get_measurement_noise()(1, 1) > FORWARD_KF_R_WHEEL &&
              state.imu_acc_bias == 0.0f && state.a > 0.0f,
          "model-free slip weighting retains IMU fusion and freezes bias");
    std::printf("PASS: model-free slip adaptation, step v %.4f, "
                "score %.4f\n",
                slip_step_v, slip_step_score);

    filter.reset(0.0f, 1.0f, 0.0f);
    state = filter.update(0.0f, std::numeric_limits<float>::quiet_NaN());
        check(matrix::is_finite(state.v) && state.v == 1.0f &&
              kf.get_diagnostics().measurement_count == 1 && kf.get_gain()(1, 1) == 0.0f,
          "invalid wheel disables only wheel measurement");
    state = filter.update(0.0f, 1000.0f, false);
    check(state.v == 1.0f, "offline wheel value is ignored");
    state = filter.update(0.0f, 1.0f, true);
    check(kf.get_diagnostics().measurement_count == 2 && state.slip_score == 0.0f,
          "wheel recovery has no derivative impulse");
    filter.reset(0.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f, false);
    check(kf.get_state()[1] == 0.0f, "invalid wheel at reset");
    std::puts("PASS: encoder invalidation and recovery");

    // Deliberately singular scalar innovation; exercise the real CMSIS failure path.
    lkf_t<1, 1> singular;
    lkf_t<1, 1>::options_t inverse_options;
    inverse_options.solver = lkf_t<1, 1>::solver_t::INVERSE;
    singular.init({2}, {0}, inverse_options);
    singular.predict({1}, {0});
    singular.correct({100}, {1}, {0});
    check(singular.get_diagnostics().math_status == ARM_MATH_SINGULAR &&
              singular.get_state()[0] == 2.0f && singular.get_covariance()[0] == 0.0f,
          "inverse failure retains prediction");
    singular.predict({1}, {0});
    check(singular.correct({100}, {1}, {1}) == PYRO_OK &&
              singular.get_diagnostics().math_status == ARM_MATH_SUCCESS,
          "inverse failure clears next cycle");
    std::puts("PASS: innovation inverse failure and next-cycle recovery");

    // More measurements than states used to overflow the shared scratch arrays.
    lkf_t<1, 3> wide;
    wide.init({0}, {1});
    wide.predict({1}, {0});
    wide.correct({1, 1, 1}, {1, 1, 1}, {1,0,0, 0,1,0, 0,0,1});
    check(std::abs(wide.get_state()[0] - 0.75f) < 1.0e-5f &&
              std::abs(wide.get_covariance()[0] - 0.25f) < 1.0e-5f,
          "scratch supports zSize > xhatSize");
    std::puts("PASS: scratch storage for more measurements than states");

    std::puts("All forward KF checks passed.");
    return 0;
}
