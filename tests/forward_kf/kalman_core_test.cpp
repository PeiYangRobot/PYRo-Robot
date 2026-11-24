#include "pyro_algo_kf.h"
#include "pyro_algo_lkf.h"
#include "pyro_algo_ekf.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <new>

using namespace pyro;
static std::size_t allocations = 0;
void *operator new(std::size_t size)
{
    ++allocations;
    if (void *value = std::malloc(size))
        return value;
    throw std::bad_alloc();
}
void *operator new[](std::size_t size)
{
    return ::operator new(size);
}
void operator delete(void *value) noexcept
{
    std::free(value);
}
void operator delete[](void *value) noexcept
{
    std::free(value);
}
void operator delete(void *value, std::size_t) noexcept
{
    std::free(value);
}
void operator delete[](void *value, std::size_t) noexcept
{
    std::free(value);
}

static void check(bool passed, const char *message)
{
    if (!passed)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

// Independent double-precision batch KF for two states, three correlated
// measurements and explicit control. Uses Gaussian elimination, not CMSIS.
struct Reference
{
    double x[2] = {0.2, -0.1};
    double p[4] = {1, 0.1, 0.1, 0.5};
    void step(const float *f, const float *q, const float *b, float u,
              const float *z, const float *h, const float *r)
    {
        double xp[2]{}, pp[4]{}, cross[6]{}, a[3][6]{}, gain[6]{};
        for (int i = 0; i < 2; ++i)
        {
            xp[i] = b[i] * u;
            for (int j = 0; j < 2; ++j)
                xp[i] += f[i * 2 + j] * x[j];
            for (int j = 0; j < 2; ++j)
            {
                pp[i * 2 + j] = q[i * 2 + j];
                for (int k = 0; k < 2; ++k)
                    for (int l = 0; l < 2; ++l)
                        pp[i * 2 + j] +=
                            f[i * 2 + k] * p[k * 2 + l] * f[j * 2 + l];
            }
        }
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 2; ++k)
                    cross[i * 3 + j] += pp[i * 2 + k] * h[j * 2 + k];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            {
                a[i][j]     = r[i * 3 + j];
                a[i][j + 3] = i == j;
                for (int k = 0; k < 2; ++k)
                    a[i][j] += h[i * 2 + k] * cross[k * 3 + j];
            }
        for (int i = 0; i < 3; ++i)
        {
            int pivot = i;
            for (int row = i + 1; row < 3; ++row)
                if (std::fabs(a[row][i]) > std::fabs(a[pivot][i]))
                    pivot = row;
            for (int col = 0; col < 6; ++col)
                std::swap(a[i][col], a[pivot][col]);
            const double scale = a[i][i];
            for (int col = 0; col < 6; ++col)
                a[i][col] /= scale;
            for (int row = 0; row < 3; ++row)
                if (row != i)
                {
                    const double coefficient = a[row][i];
                    for (int col = 0; col < 6; ++col)
                        a[row][col] -= coefficient * a[i][col];
                }
        }
        double residual[3]{}, transform[4]{};
        for (int j = 0; j < 3; ++j)
        {
            residual[j] = z[j];
            for (int k = 0; k < 2; ++k)
                residual[j] -= h[j * 2 + k] * xp[k];
        }
        for (int i = 0; i < 2; ++i)
        {
            x[i] = xp[i];
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 3; ++k)
                    gain[i * 3 + j] += cross[i * 3 + k] * a[k][j + 3];
                x[i] += gain[i * 3 + j] * residual[j];
            }
            for (int j = 0; j < 2; ++j)
            {
                transform[i * 2 + j] = i == j;
                for (int k = 0; k < 3; ++k)
                    transform[i * 2 + j] -= gain[i * 3 + k] * h[k * 2 + j];
            }
        }
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
            {
                p[i * 2 + j] = 0;
                for (int k = 0; k < 2; ++k)
                    for (int l = 0; l < 2; ++l)
                        p[i * 2 + j] += transform[i * 2 + k] * pp[k * 2 + l] *
                                        transform[j * 2 + l];
                for (int k = 0; k < 3; ++k)
                    for (int l = 0; l < 3; ++l)
                        p[i * 2 + j] +=
                            gain[i * 3 + k] * r[k * 3 + l] * gain[j * 3 + l];
            }
    }
};

int main()
{
    using filter_t = lkf_t<2, 3, 1>;
    filter_t filter;
    lkf_t<2, 3, 1, true> checked_filter;
    static_assert(!filter_t::check_numerics);
    static_assert(!kf_t<1, 1>::check_numerics);
    static_assert(!ekf_t<1, 1>::check_numerics);
    static_assert(decltype(checked_filter)::check_numerics);
    filter_t::state_t x             = {0.2f, -0.1f};
    filter_t::covariance_t p        = {1, 0.1f, 0.1f, 0.5f};
    filter_t::transition_t f        = {1, 0.001f, 0, 1};
    filter_t::covariance_t q        = {1e-7f, 1e-6f, 1e-6f, 1e-3f};
    filter_t::control_matrix_t b    = {0.0000005f, 0.001f};
    filter_t::observation_t h       = {1, 0, 0, 1, 1, 1};
    filter_t::measurement_noise_t r = {0.3f,   0.02f,  -0.01f, 0.02f, 0.1f,
                                       0.015f, -0.01f, 0.015f, 0.2f};
    check(filter.predict(f, q) == PYRO_NOT_FOUND, "uninitialized filter");
    const auto allocation_start = allocations;
    check(filter.init(x, p) == PYRO_OK, "init");
    check(checked_filter.init(x, p) == PYRO_OK, "checked init");
    check(filter.init(x, p) == PYRO_ALREADY_INIT, "repeated init");
    Reference reference;
    for (int i = 0; i < 10000; ++i)
    {
        f[1]                      = 0.0008f + (i % 5) * 0.0001f;
        h[4]                      = 1.0f + 0.1f * std::sin(i * 0.002f);
        r[8]                      = 0.2f + (i % 3) * 0.02f;
        q[3]                      = 0.001f + (i % 2) * 0.0001f;
        filter_t::control_t u     = {0.3f * std::cos(i * 0.001f)};
        filter_t::measurement_t z = {std::sin(i * 0.001f), std::cos(i * 0.001f),
                                     0.4f};
        check(filter.predict(f, q, b, u) == PYRO_OK,
              "time-varying control prediction");
        check(filter.correct(z, h, r) == PYRO_OK, "correlated correction");
        check(checked_filter.predict(f, q, b, u) == PYRO_OK &&
                  checked_filter.correct(z, h, r) == PYRO_OK &&
                  checked_filter.get_state() == filter.get_state() &&
                  checked_filter.get_covariance() == filter.get_covariance(),
              "checked and unchecked paths are identical for valid models");
        reference.step(f.data(), q.data(), b.data(), u[0], z.data(), h.data(),
                       r.data());
        const auto &cov = filter.get_covariance();
        for (int j = 0; j < 2; ++j)
            check(std::fabs(filter.get_state()[j] - reference.x[j]) < 1e-4,
                  "double state reference");
        for (int j = 0; j < 4; ++j)
            check(std::fabs(cov[j] - reference.p[j]) < 1e-5,
                  "double covariance reference");
        check(cov[1] == cov[2] && cov[0] > 0 &&
                  cov[0] * cov[3] > cov[1] * cov[2],
              "symmetric positive P");
    }
    check(filter.reset(x, p) == PYRO_OK, "reset");
    check(allocations == allocation_start, "no new/new[] in core lifecycle");

    // Nonadjacent rows must select the full correlated R submatrix.
    using wide_t = lkf_t<1, 3, 0, true>;
    wide_t wide;
    wide.init({2}, {1});
    wide_t::measurement_mask_t mask        = {true, false, true};
    wide_t::measurement_noise_t correlated = {1, 0, 0.5f, 0, 0, 0, 0.5f, 0, 1};
    check(wide.correct({0, 1000, 0}, {1, 1, 1}, correlated, &mask) == PYRO_OK,
          "masked correlated rows");
    check(std::fabs(wide.get_state()[0] - 6.0f / 7.0f) < 1e-6f,
          "valid zero correction with correlated R");
    check(wide.get_gain()[1] == 0 &&
              wide.get_diagnostics().measurement_count == 2,
          "full-order diagnostic gain");
    const auto prior = wide.get_state();
    mask.fill(false);
    check(wide.correct({0, 0, 0}, {1, 1, 1}, correlated, &mask) ==
                  PYRO_WARNING &&
              wide.get_state() == prior,
          "all missing keeps prediction");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    check(wide.correct({nan, nan, nan}, {1, 1, 1}, correlated) == PYRO_WARNING,
          "NaN samples are missing");
    check(wide.predict({nan}, {0}) == PYRO_PARAM_ERROR &&
              wide.get_state() == prior,
          "bad F holds state");
    check(wide.predict({1}, {-1}) == PYRO_PARAM_ERROR,
          "negative process variance rejected");
    wide.reset({2}, {1});
    check(wide.predict({std::numeric_limits<float>::max()}, {0}) ==
                  PYRO_ERROR &&
              wide.get_diagnostics().result ==
                  wide_t::result_t::NON_FINITE_RESULT &&
              wide.get_state()[0] == 2,
          "finite-input prediction overflow does not commit");
    check(wide.reset({2}, {1}) == PYRO_OK, "recover reset");
    wide_t::measurement_mask_t only = {true, false, false};
    check(wide.correct({0, nan, nan}, {1, nan, nan}, correlated, &only) ==
              PYRO_OK,
          "masked bad rows ignored");
    check(std::fabs(wide.get_state()[0] - 1) < 1e-6f,
          "valid row survives missing rows");
    check(wide.reset({2}, {0}) == PYRO_OK, "zero P allowed");
    check(wide.predict({1}, {0}) == PYRO_OK, "prediction before singular S");
    check(wide.correct({0, 0, 0}, {1, 1, 1}, {}) == PYRO_ERROR &&
              wide.get_diagnostics().math_status ==
                  ARM_MATH_DECOMPOSITION_FAILURE &&
              wide.get_state()[0] == 2,
          "failed solve keeps prediction and reason");
    check(wide.correct({0, 0, 0}, {1, 1, 1}, {1, 0, 0, 0, 1, 0, 0, 0, 1}) ==
              PYRO_OK,
          "solve recovery");

    // EKF residual is z-h(x), not z-H*x; gating stays inside one correction.
    ekf_t<1, 1, true> nonlinear;
    nonlinear.init({2}, {1});
    nonlinear.predict({2}, {1}, {0});
    int policy_calls = 0;
    check(nonlinear.correct(
              {5}, {4}, {4}, {1}, nullptr,
              [&](auto &correction)
              {
                  ++policy_calls;
                  check(nonlinear.get_state()[0] == 2 &&
                            std::fabs(correction.diagnostics.nis - 1.0f / 17) <
                                1e-6f,
                        "policy sees NIS before any state commit");
                  return PYRO_OK;
              }) == PYRO_OK &&
              std::fabs(nonlinear.get_state()[0] - (2.0f + 4.0f / 17)) < 1e-6f,
          "nonlinear residual");
    check(policy_calls == 1, "one policy call per correction");
    const auto before_rejection = nonlinear.get_state();
    check(nonlinear.correct({100}, {4}, {4}, {1}, nullptr, [](auto &)
                            { return PYRO_WARNING; }) == PYRO_WARNING &&
              nonlinear.get_state() == before_rejection,
          "gating holds prior");
    check(nonlinear.correct({5}, {4}, {4}, {1}, nullptr,
                            [nan](auto &correction)
                            {
                                correction.K[0] = nan;
                                return PYRO_OK;
                            }) == PYRO_PARAM_ERROR &&
              nonlinear.get_state() == before_rejection,
          "bad custom gain does not commit");
    check(nonlinear.predict({nan}, {1}, {0}) == PYRO_PARAM_ERROR &&
              nonlinear.get_state() == before_rejection,
          "bad nonlinear prediction does not commit");
    check(nonlinear.correct({5}, {4}, {4}, {1}, nullptr,
                            [](auto &) { return PYRO_ERROR; }) == PYRO_ERROR &&
              nonlinear.get_diagnostics().result ==
                  decltype(nonlinear)::result_t::POLICY_FAILURE &&
              nonlinear.get_state() == before_rejection,
          "policy failure reports without commit");
    nonlinear.correct({5}, {4}, {4}, {1});
    check(nonlinear.get_diagnostics().result ==
              decltype(nonlinear)::result_t::CORRECTED,
          "correction recovers after a failed policy");
    const auto before_copy = nonlinear.get_state();
    auto copy              = nonlinear;
    copy.reset({0}, {1});
    check(nonlinear.get_state() == before_copy, "copied objects do not alias");
    lkf_t<1, 1> bounded;
    lkf_t<1, 1>::options_t options;
    options.min_variance[0] = 0.1f;
    bounded.init({0}, {0}, options);
    check(bounded.get_covariance()[0] == 0.1f,
          "variance floor at initialization");
    bounded.predict({0.1f}, {0});
    check(bounded.get_covariance()[0] == 0.1f,
          "variance floor also applies to prediction without measurements");

    // Core owns only numerical propagation/fusion; any model supplies x_pred/y.
    kf_t<1, 1, true> core;
    core.init({2}, {1});
    check(core.predict({7}, {3}, {2}) == PYRO_OK && core.get_state()[0] == 7 &&
              core.get_covariance()[0] == 11,
          "generic prediction commits external x rather than F*x");
    check(core.correct({1}, {2}, {3}) == PYRO_OK &&
              std::fabs(core.get_state()[0] - (7 + 22.0f / 47)) < 1e-6f,
          "generic correction fuses supplied innovation");
    core.reset({0}, {1});
    check(core.correct({2}, {1}, {1}, nullptr,
                       [](auto &correction)
                       {
                           correction.K[0] *= 0.5f;
                           return PYRO_OK;
                       }) == PYRO_OK &&
              std::fabs(core.get_state()[0] - 0.5f) < 1e-6f &&
              std::fabs(core.get_covariance()[0] - 0.625f) < 1e-6f,
          "modified gain recomputes delta and Joseph covariance");
    core.reset({0}, {1});
    check(core.correct({2}, {1}, {1}, nullptr,
                       [](auto &correction)
                       {
                           correction.dx[0]  = 0.1f;
                           correction.use_dx = true;
                           return PYRO_OK;
                       }) == PYRO_OK &&
              std::fabs(core.get_state()[0] - 0.1f) < 1e-6f &&
              std::fabs(core.get_covariance()[0] - 0.5f) < 1e-6f,
          "external delta override retains K-based covariance");
    bool called = false;
    check(core.correct({nan}, {1}, {1}, nullptr,
                       [&](auto &)
                       {
                           called = true;
                           return PYRO_OK;
                       }) == PYRO_PARAM_ERROR &&
              !called,
          "invalid residual fails before invoking policy");
    decltype(core)::measurement_mask_t missing{false};
    check(core.correct({nan}, {nan}, {nan}, &missing,
                       [&](auto &)
                       {
                           called = true;
                           return PYRO_OK;
                       }) == PYRO_WARNING &&
              !called,
          "missing rows bypass policy and invalid model data");
    core.reset({0}, {0});
    check(core.correct({1}, {1}, {0}, nullptr,
                       [&](auto &)
                       {
                           called = true;
                           return PYRO_OK;
                       }) == PYRO_ERROR &&
              !called,
          "failed solve does not invoke policy");
    ekf_t<1, 1> square;
    square.init({2}, {1});
    check(square.predict({4}, {4}, {0}) == PYRO_OK &&
              square.get_state()[0] == 4 && square.get_covariance()[0] == 16,
          "independent EKF supports nonlinear state model");
    check(square.correct({17}, {16}, {8}, {1}) == PYRO_OK &&
              std::fabs(square.get_state()[0] - (4 + 128.0f / 1025)) < 1e-6f,
          "independent EKF uses nonlinear observation and Jacobian");

    kf_t<1, 1> fast_core;
    check(fast_core.predict({0}, {1}, {0}) == PYRO_NOT_FOUND,
          "unchecked core retains initialization checks");
    fast_core.init({0}, {1});
    // Intentionally violate the numeric input contract to verify scans are absent.
    check(fast_core.predict({nan}, {1}, {0}) == PYRO_OK &&
              !matrix::is_finite(fast_core.get_state()[0]),
          "unchecked prediction does not scan numeric values");
    fast_core.reset({0}, {0});
    check(fast_core.correct({1}, {1}, {0}) == PYRO_ERROR &&
              fast_core.get_diagnostics().math_status ==
                  ARM_MATH_DECOMPOSITION_FAILURE &&
              fast_core.get_state()[0] == 0,
          "unchecked core retains solve errors and commit semantics");
    fast_core.reset({2}, {1});
    check(fast_core.correct({nan}, {nan}, {nan}, &missing) == PYRO_WARNING &&
              fast_core.get_state()[0] == 2,
          "unchecked missing rows use explicit mask");
    ekf_t<1, 2> fast_ekf;
    fast_ekf.init({2}, {1});
    decltype(fast_ekf)::measurement_mask_t active{true, false};
    check(fast_ekf.correct({0, nan}, {2, nan}, {1, nan}, {1, nan, nan, nan},
                           &active) == PYRO_OK &&
              std::fabs(fast_ekf.get_state()[0] - 1) < 1e-6f,
          "unchecked EKF excludes explicitly masked invalid rows");
    check(allocations == allocation_start,
          "policies and nonlinear adapters allocate no heap memory");
    std::puts("PASS: generic KF, dynamic matrices/control, correlated/missing "
              "rows, EKF, failures and storage");
}
