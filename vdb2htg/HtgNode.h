// Copyright 2019-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// knobs
#define HTG_SHARE_INTERPOLATION_CODE
#define HTG_INVALID_VALUE 0.0 /* TODO using 0 to represent invalid value */

#if defined(ISPC)

#include "math/box.ih"
#include "math/vec.ih"
#include "openvkl/VKLDataType.h"

#if !defined(HTG_SHARE_INTERPOLATION_CODE)
inline float max(float a, float b, float c)
{
  return max(max(a, b), c);
}

inline float min(float a, float b, float c)
{
  return min(min(a, b), c);
}

inline float max(
    float a, float b, float c, float d, float e, float f, float g, float h)
{
  return max(max(max(a, b), max(c, d)), max(max(e, f), max(g, h)));
}

inline float min(
    float a, float b, float c, float d, float e, float f, float g, float h)
{
  return min(min(min(a, b), min(c, d)), min(min(e, f), min(g, h)));
}
#endif

#elif defined(__cplusplus)

#include <vector>
#if !defined(HTG_STANDALONE)
#include "../common/Data.h"
#include "../common/math.h"
#else
#include "rkcommon/math/vec.h"
#include "rkcommon/math/range.h"
#include "rkcommon/math/box.h"
using namespace rkcommon::math;
#endif

namespace openvkl
{ // some helper functions
  namespace ispc_driver
  {

    inline const char *c_str(const std::string &s)
    {
      return s.c_str();
    }

    template <typename T>
    inline T c_str(T s)
    {
      return s;
    }

    template <typename... Ts>
    std::string stringf(const std::string &format, Ts... rest)
    {
      ssize_t sz = snprintf(NULL, 0, format.c_str(), c_str(rest)...);
      char *bf = static_cast<char *>(malloc(sz + 1));
      snprintf(bf, sz + 1, format.c_str(), c_str(rest)...);
      std::string ret(bf);
      free(bf);
      return ret;
    }

    static inline int roundToPow2(int x)
    {
      int y;
      for (y = 1; y < x; y *= 2)
        ;
      return y;
    }

    static inline float roundToPow2(float x)
    {
      float y;
      for (y = 1.0; y < x; y *= 2)
        ;
      return y;
    }

  } // namespace ispc_driver
} // namespace openvkl

#endif // defined(__cplusplus)

#if defined(ISPC)
typedef unsigned int8 uint8_t;
typedef unsigned int16 uint16_t;
typedef unsigned int32 uint32_t;
typedef unsigned int64 uint64_t;
#define __varying varying
#define __uniform uniform
#elif defined(__cplusplus)
#define __varying
#define __uniform
#endif
#define __empty

#if defined(__cplusplus)
namespace openvkl
{ // some helper functions
  namespace ispc_driver
  {
#endif // defined(__cplusplus)

#if !defined(HTG_SHARE_INTERPOLATION_CODE)
    /*! enum to symbolically iterate the 8 corners of an octant */
    enum CORNER_INDEX
    {
      C000 = 0,
      C001,
      C010,
      C011,
      C100,
      C101,
      C110,
      C111
    };
#endif

    inline __varying double decodeDouble(__varying uint64_t i)
    {
      // union { uint64_t i; double f; } unionHack;
      // unionHack.i = i;
      // return unionHack.f;
      __varying double *__uniform v = (__varying double *__uniform) & i;
      return *v;
    }

    inline __varying uint64_t encodeDouble(__varying double f)
    {
      // union { uint64_t i; double f; } unionHack;
      // unionHack.f = f;
      // return unionHack.i;
      __varying uint64_t *__uniform v = (__varying uint64_t * __uniform) & f;
      return *v;
    }

#if defined(__cplusplus)
  } // namespace ispc_driver
} // namespace openvkl
#endif // defined(__cplusplus)

// #include "FormatStatic.h"
#include "FormatStream.h"
