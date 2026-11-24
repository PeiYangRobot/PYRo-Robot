#include "pyro_algo_matrix.h"
// Verify that including Matrix first does not pollute core constants.
#include "pyro_core_def.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <type_traits>
#include <utility>
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

template <typename A, typename B, typename = void>
struct can_multiply : std::false_type
{
};
template <typename A, typename B>
struct can_multiply<
    A, B, std::void_t<decltype(std::declval<A>() * std::declval<B>())>>
    : std::true_type
{
};
template <typename A, typename B, typename = void>
struct can_add : std::false_type
{
};
template <typename A, typename B>
struct can_add<A, B,
               std::void_t<decltype(std::declval<A>() + std::declval<B>())>>
    : std::true_type
{
};

static void check(bool passed, const char *message)
{
    if (!passed)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

template <std::size_t Rows, std::size_t Cols>
static void near(const matrix_t<Rows, Cols> &actual,
                 const matrix_t<Rows, Cols> &expected, const char *message)
{
    for (std::size_t i = 0; i < actual.size(); ++i)
        check(matrix::is_finite(actual[i]) &&
                  std::fabs(actual[i] - expected[i]) < 1.0e-5f,
              message);
}

static void arithmetic()
{
    static_assert(sizeof(matrix_t<2, 3>) == sizeof(std::array<float, 6>));
    static_assert(std::is_trivially_copyable_v<matrix_t<2, 3>>);
    static_assert(std::is_same_v<vector_t<2>, matrix_t<2, 1>>);
    const matrix_t<2, 3> A{1, 2, 3, 4, 5, 6};
    const matrix_t<3, 2> B{7, 8, 9, 10, 11, 12};
    matrix_t<2, 2> product{};
    check(matrix::multiply(matrix::view<2, 3>(A), matrix::view<3, 2>(B),
                           matrix::view<2, 2>(product)) == ARM_MATH_SUCCESS,
          "rectangular product");
    near(product, matrix_t<2, 2>{58, 64, 139, 154}, "product reference");
    matrix_t<3, 2> transposed{};
    check(matrix::transpose(matrix::view<2, 3>(A),
                            matrix::view<3, 2>(transposed)) == ARM_MATH_SUCCESS,
          "rectangular transpose");
    near(transposed, matrix_t<3, 2>{1, 4, 2, 5, 3, 6}, "transpose reference");
    matrix_t<2, 3> sum = A;
    check(matrix::add(matrix::view<2, 3>(A), matrix::view<2, 3>(sum),
                      matrix::view<2, 3>(sum)) == ARM_MATH_SUCCESS,
          "sum may alias second input");
    near(sum, matrix_t<2, 3>{2, 4, 6, 8, 10, 12}, "sum reference");
    check(matrix::subtract(matrix::view<2, 3>(sum), matrix::view<2, 3>(A),
                           matrix::view<2, 3>(sum)) == ARM_MATH_SUCCESS,
          "difference may alias first input");
    near(sum, A, "difference reference");

    // Reuse a larger buffer with a smaller packed shape, as missing KF rows do.
    matrix_t<3, 3> workspace{};
    check(matrix::multiply(matrix::view<2, 3>(A), matrix::view<3, 2>(B),
                           matrix::view<2, 2>(workspace)) == ARM_MATH_SUCCESS,
          "packed shape in larger buffer");
    check(workspace[2] == 139 && workspace[3] == 154 && workspace[4] == 0,
          "packed stride and unused tail");
}

static void operators()
{
    using A_t = matrix_t<2, 3>;
    using B_t = matrix_t<3, 2>;
    static_assert(can_multiply<A_t, B_t>::value);
    static_assert(!can_multiply<A_t, A_t>::value);
    static_assert(!can_add<A_t, B_t>::value);
    static_assert(!std::is_same_v<A_t, B_t>);
    static_assert(
        std::is_same_v<decltype(std::declval<A_t>() * std::declval<B_t>()),
                       matrix_t<2, 2>>);
    static_assert(
        std::is_same_v<decltype(std::declval<A_t>().transpose()), B_t>);
    constexpr A_t initialized{1, 2, 3};
    static_assert(initialized(0, 2) == 3 && initialized(1, 2) == 0);
    constexpr A_t from_array(std::array<float, 6>{1, 2, 3, 4, 5, 6});
    static_assert(from_array(1, 2) == 6);

    const A_t A{1, 2, 3, 4, 5, 6};
    const B_t B{7, 8, 9, 10, 11, 12};
    near(A * B, matrix_t<2, 2>{58, 64, 139, 154}, "operator product reference");
    near(A.transpose(), B_t{1, 4, 2, 5, 3, 6}, "operator transpose reference");
    near(A + A, A_t{2, 4, 6, 8, 10, 12}, "operator sum reference");
    near(A - A, A_t{}, "operator difference reference");
    near(-A, A_t{-1, -2, -3, -4, -5, -6}, "unary minus reference");
    near(A * 2, A + A, "right scalar product");
    near(2 * A, A + A, "left scalar product");
    near((A * 2) / 2, A, "scalar division");

    A_t accumulated = A;
    check(&(accumulated += A) == &accumulated, "sum assignment returns self");
    near(accumulated, A * 2, "sum assignment reference");
    accumulated -= A;
    accumulated *= 2;
    accumulated /= 2;
    near(accumulated, A, "compound scaling reference");
    accumulated += accumulated;
    near(accumulated, A * 2, "self sum assignment");
    accumulated -= accumulated;
    near(accumulated, A_t{}, "self difference assignment");

    matrix_t<2, 2> square{1, 2, 3, 4};
    check(&(square *= square) == &square, "product assignment returns self");
    near(square, matrix_t<2, 2>{7, 10, 15, 22}, "self product assignment");
    accumulated = A;
    const matrix_t<3, 3> identity{1, 0, 0, 0, 1, 0, 0, 0, 1};
    accumulated *= identity;
    near(accumulated, A, "rectangular product assignment");
    accumulated(1, 2) = 9;
    check(accumulated[5] == 9 && accumulated.data()[5] == 9,
          "row-column indexing preserves row-major layout");
    auto &storage = accumulated.as_array();
    storage[5]    = 6;
    check(accumulated == A && !(accumulated != A),
          "borrowed array and equality");
    accumulated = std::array<float, 6>{1, 2, 3, 4, 5, 6};
    near(accumulated, A, "array assignment conversion");
    accumulated = {1, 2};
    near(accumulated, A_t{1, 2, 0, 0, 0, 0},
         "list assignment zeroes omitted entries");
    const auto inferred = matrix::view(A);
    check(inferred.rows == 2 && inferred.cols == 3 && inferred.data == A.data(),
          "const view infers shape");
    check(vector_t<0>{}.empty(), "absent control type remains valid");

    // Exercise the covariance formula through the value interface.
    const matrix_t<2, 2> F{1, 0.5f, 0, 1}, P{2, 0.2f, 0.2f, 1};
    const matrix_t<2, 2> Q_d{0.01f, 0, 0, 0.02f};
    near(F * P * F.transpose() + Q_d, matrix_t<2, 2>{2.46f, 0.7f, 0.7f, 1.02f},
         "composed covariance expression");
    // The first assignment must finish all input reads before writing A.
    square = {1, 2, 3, 4};
    square = square * square;
    near(square, matrix_t<2, 2>{7, 10, 15, 22},
         "aliased expression assignment");
    check(A == from_array, "operators preserve inputs");
    check(!matrix::is_finite(A / 0.0f) && !matrix::is_finite(A / -0.0f),
          "division by zero yields non-finite result");
    const vector_t<1> huge{std::numeric_limits<float>::max()};
    check(!matrix::is_finite(huge * 2.0f),
          "scaling overflow remains detectable");
}

static void solvers()
{
    const matrix_t<2, 2> A{4, 1, 1, 3};
    const auto original = A;
    matrix_t<2, 2> inverted{}, scratch{};
    check(matrix::inverse(matrix::view<2, 2>(A), matrix::view<2, 2>(inverted),
                          matrix::view<2, 2>(scratch)) == ARM_MATH_SUCCESS,
          "inverse");
    near(inverted, matrix_t<2, 2>{3.0f / 11, -1.0f / 11, -1.0f / 11, 4.0f / 11},
         "inverse reference");
    check(A == original, "inverse preserves input");

    const matrix_t<2, 2> positive{4, 2, 2, 5};
    matrix_t<2, 2> L{99, 99, 99, 99};
    check(matrix::cholesky(matrix::view<2, 2>(positive),
                           matrix::view<2, 2>(L)) == ARM_MATH_SUCCESS,
          "Cholesky");
    near(L, matrix_t<2, 2>{2, 0, 1, 2},
         "Cholesky clears reused upper triangle");
    matrix_t<2, 2> Lt{}, reconstructed{};
    check(matrix::transpose(matrix::view<2, 2>(L), matrix::view<2, 2>(Lt)) ==
                  ARM_MATH_SUCCESS &&
              matrix::multiply(matrix::view<2, 2>(L), matrix::view<2, 2>(Lt),
                               matrix::view<2, 2>(reconstructed)) ==
                  ARM_MATH_SUCCESS,
          "Cholesky reconstruction");
    near(reconstructed, positive, "L*L^T reference");

    const matrix_t<2, 2> lower_rhs{2, 4, 7, 10};
    const matrix_t<2, 2> upper_rhs{5, 8, 6, 8};
    const matrix_t<2, 2> solution{1, 2, 3, 4};
    matrix_t<2, 2> X{};
    check(matrix::solve_lower(matrix::view<2, 2>(L),
                              matrix::view<2, 2>(lower_rhs),
                              matrix::view<2, 2>(X)) == ARM_MATH_SUCCESS,
          "lower solve multiple RHS");
    near(X, solution, "lower solve reference");
    check(matrix::solve_upper(matrix::view<2, 2>(Lt),
                              matrix::view<2, 2>(upper_rhs),
                              matrix::view<2, 2>(X)) == ARM_MATH_SUCCESS,
          "upper solve multiple RHS");
    near(X, solution, "upper solve reference");

    const matrix_t<2, 2> singular{1, 2, 2, 4};
    check(matrix::inverse(matrix::view<2, 2>(singular), matrix::view<2, 2>(X),
                          matrix::view<2, 2>(scratch)) == ARM_MATH_SINGULAR,
          "singular inverse status");
    const matrix_t<2, 2> indefinite{1, 2, 2, 1};
    check(matrix::cholesky(matrix::view<2, 2>(indefinite),
                           matrix::view<2, 2>(X)) != ARM_MATH_SUCCESS,
          "non-positive-definite Cholesky status");
    const matrix_t<2, 2> zero_diagonal{0, 0, 1, 2};
    check(matrix::solve_lower(matrix::view<2, 2>(zero_diagonal),
                              matrix::view<2, 2>(lower_rhs),
                              matrix::view<2, 2>(X)) == ARM_MATH_SINGULAR,
          "singular lower solve status");
    check(matrix::solve_upper(matrix::view<2, 2>(zero_diagonal),
                              matrix::view<2, 2>(upper_rhs),
                              matrix::view<2, 2>(X)) == ARM_MATH_SINGULAR,
          "singular upper solve status");
}

static void validation()
{
    matrix_t<2, 2> A{1, 2, 3, 4}, B{5, 6, 7, 8}, out{91, 92, 93, 94};
    const auto original = out;
    const auto a = matrix::view<2, 2>(A), b = matrix::view<2, 2>(B);
    const auto result = matrix::view<2, 2>(out);
    check(matrix::multiply({A.data(), 2, 1}, b, result) ==
              ARM_MATH_SIZE_MISMATCH,
          "product dimension mismatch");
    check(matrix::transpose({A.data(), 1, 4}, result) == ARM_MATH_SIZE_MISMATCH,
          "transpose dimension mismatch");
    check(matrix::add({A.data(), 1, 4}, b, result) == ARM_MATH_SIZE_MISMATCH,
          "sum dimension mismatch");
    check(matrix::inverse({A.data(), 1, 4}, result, b) ==
              ARM_MATH_SIZE_MISMATCH,
          "inverse requires square input");
    check(matrix::cholesky({A.data(), 1, 4}, result) == ARM_MATH_SIZE_MISMATCH,
          "Cholesky requires square input");
    check(matrix::solve_lower(a, {B.data(), 1, 4}, result) ==
              ARM_MATH_SIZE_MISMATCH,
          "triangular dimension mismatch");
    check(matrix::multiply({nullptr, 2, 2}, b, result) ==
              ARM_MATH_ARGUMENT_ERROR,
          "null input");
    check(matrix::multiply({A.data(), 0, 2}, b, result) ==
              ARM_MATH_ARGUMENT_ERROR,
          "zero dimension");
    check(matrix::multiply({A.data(), 65536, 2}, b, result) ==
              ARM_MATH_ARGUMENT_ERROR,
          "dimension cannot truncate to uint16");
    check(matrix::subtract(a, b, {nullptr, 2, 2}) == ARM_MATH_ARGUMENT_ERROR,
          "null output");
    check(matrix::multiply(a, b, a) == ARM_MATH_ARGUMENT_ERROR,
          "product rejects aliased output");
    check(matrix::transpose(a, a) == ARM_MATH_ARGUMENT_ERROR,
          "transpose rejects aliased output");
    check(matrix::inverse(a, result, a) == ARM_MATH_ARGUMENT_ERROR,
          "inverse rejects source workspace overlap");
    check(matrix::inverse(a, result, result) == ARM_MATH_ARGUMENT_ERROR,
          "inverse rejects output workspace overlap");
    check(matrix::cholesky(a, a) == ARM_MATH_ARGUMENT_ERROR,
          "Cholesky rejects aliased output");
    check(matrix::solve_lower(a, b, b) == ARM_MATH_ARGUMENT_ERROR,
          "triangular solve rejects aliased RHS");
    check(out == original, "validation failures leave output intact");
    check(A == matrix_t<2, 2>{1, 2, 3, 4},
          "validation failures preserve input");

    std::array<float, 5> partial{1, 2, 3, 4, 5};
    check(matrix::add({partial.data(), 2, 2}, b, {partial.data() + 1, 2, 2}) ==
              ARM_MATH_ARGUMENT_ERROR,
          "sum rejects partial overlap");
    check(partial == std::array<float, 5>{1, 2, 3, 4, 5},
          "overlap leaves data intact");
}

int main()
{
    const auto allocation_start = allocations;
    arithmetic();
    operators();
    solvers();
    validation();
    check(matrix::is_finite(0.0f) && matrix::is_finite(-1.0f),
          "finite scalars");
    check(!matrix::is_finite(std::numeric_limits<float>::infinity()) &&
              !matrix::is_finite(std::numeric_limits<float>::quiet_NaN()),
          "non-finite scalars under optimization");
    check(!matrix::is_finite(
              vector_t<2>{0, std::numeric_limits<float>::infinity()}),
          "non-finite arrays");
    check(allocations == allocation_start,
          "no heap allocation in matrix operations");
    std::puts("Matrix checks passed");
}
