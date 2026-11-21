#ifndef BITSIM_EXPORT_H
#define BITSIM_EXPORT_H

///
/// @brief Marks a class or a function that bitsim's library exports from a source file compiled
/// with hidden visibility.
///
/// bitsim compiles the files that use Eigen's sparse matrices with hidden visibility, so that
/// the Eigen code they instantiate stays inside bitsim. Shared, it would replace the same code in
/// other libraries, such as Chrono, which may be compiled with other vector instructions, and
/// crash them. What those files offer is marked with BITSIM_EXPORT instead.
///
#if defined(__GNUC__) || defined(__clang__)
#define BITSIM_EXPORT __attribute__((visibility("default")))
#else
#define BITSIM_EXPORT
#endif

#endif // BITSIM_EXPORT_H
