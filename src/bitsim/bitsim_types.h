#ifndef BITSIM_TYPES_H
#define BITSIM_TYPES_H

/**
 * \file
 * \brief The basic types of bitsim. Code outside this file names Eigen's matrix and vector
 * types only through the aliases defined here.
 */

#include "Eigen/Core"

#include <cstddef>

namespace bitsim
{

///
/// \brief real_t
///
typedef double real_t;

///
/// \brief float
///
typedef float float_t;

///
/// \brief integer type
///
typedef int int_t;

///
/// \brief long int type
///
typedef long int lint_t;

///
/// \brief uint_t
///
typedef std::size_t uint_t;

///
/// \brief The type Eigen indexes and sizes matrices and vectors with
///
using index_t = Eigen::Index;

///
/// \brief Dynamically sized matrix to use
/// around the library
///
template <typename T> using DynMat = Eigen::MatrixX<T>;

///
/// \brief Dynamically sized matrix of type double
///
using RealMat = DynMat<real_t>;

///
/// \brief Square matrix with elements of type T
///
template <typename T, uint_t N> using SquareMat = Eigen::Matrix<T, N, N>;

///
/// \brief General fixed size matrix
///
template <typename T, uint_t N, uint_t M> using Mat = Eigen::Matrix<T, N, M>;

///
/// \brief General fixed size matrix that Eigen does not align. Use it for a fixed size
/// member of a class that is allocated with new or kept in a std::vector, where the
/// alignment Eigen asks for is not guaranteed
///
template <typename T, uint_t N, uint_t M>
using UnalignedMat =
    Eigen::Matrix<T, N, M,
                  Eigen::DontAlign | (N == 1 && M != 1 ? Eigen::RowMajor : Eigen::ColMajor)>;

///
/// \brief 2×2 matrix of type double.
///
using RealMat2d = Eigen::Matrix2<real_t>;

///
/// \brief 3×3 matrix of type double.
///
using RealMat3d = Eigen::Matrix3<real_t>;

///
/// \brief 4×4 matrix of type double.
///
using RealMat4d = Eigen::Matrix4<real_t>;

///
/// \brief 3×3 matrix of type float.
///
using FloatMat3d = Eigen::Matrix3<float_t>;

///
/// \brief Dynamically sized row vector
///
template <typename T> using DynVec = Eigen::RowVectorX<T>;

///
/// \brief single precision floating point vector
///
using FloatVec = DynVec<float_t>;

///
/// \brief double precision floating point vector
///
using RealVec = DynVec<real_t>;

///
/// \brief Column vector. Some maths operations
/// are easier using column vectors rather than DynVec
///
template <typename T> using ColVec = Eigen::VectorX<T>;

///
/// \brief Dynamically sized column vector
///
using RealColVec = ColVec<real_t>;

///
/// \brief Dynamically sized column vector
///
using FoatColVec = ColVec<float_t>;

///
/// \brief 2D column vector
///
using RealColVec2d = Eigen::Vector2d;

///
/// \brief 3D column vector
///
using RealColVec3d = Eigen::Vector3d;

///
/// \brief 4D column vector
///
using RealColVec4d = Eigen::Vector4d;

///
/// \brief 3D column vector
///
using FloatColVec3d = Eigen::Vector3f;

///
/// \struct IntegralRange
/// \brief A range of integer values in
/// [s, e]
///
template <uint_t s, uint_t e> struct IntegralRange
{
    typedef uint_t value_type;
    static constexpr uint_t S = s;
    static constexpr uint_t E = e;
    static constexpr uint_t size = e - s;
};

///
/// \struct RealRange
/// \brief A range of double precision floating point values
///
template <real_t s, real_t e> struct RealRange
{
    typedef real_t value_type;
    static constexpr real_t S = s;
    static constexpr real_t E = e;
    static constexpr real_t size = e - s;
};

///
/// \struct Null
/// \brief Null placeholder
///
struct Null
{
};

/// \enum DeviceType
/// \brief Enumeration of various device types.
///
enum class DeviceType
{
    INVALID_TYPE = 0,
    CPU = 1,
    GPU = 2
};

} // namespace bitsim

#endif // BITSIM_TYPES_H
