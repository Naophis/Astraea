#include "rtwtypes.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include "limits"
#include "cmath"

extern "C"
{
  real_T rtNaN { -std::numeric_limits<real_T>::quiet_NaN() };

  real_T rtInf { std::numeric_limits<real_T>::infinity() };

  real_T rtMinusInf { -std::numeric_limits<real_T>::infinity() };

  real32_T rtNaNF { -std::numeric_limits<real32_T>::quiet_NaN() };

  real32_T rtInfF { std::numeric_limits<real32_T>::infinity() };

  real32_T rtMinusInfF { -std::numeric_limits<real32_T>::infinity() };
}

extern "C"
{
  boolean_T rtIsInf(real_T value)
  {
    return std::isinf(value);
  }

  boolean_T rtIsInfF(real32_T value)
  {
    return std::isinf(value);
  }

  boolean_T rtIsNaN(real_T value)
  {
    return std::isnan(value);
  }

  boolean_T rtIsNaNF(real32_T value)
  {
    return std::isnan(value);
  }
}
