#include "rtwtypes.h"

extern "C"
{

#include "rtGetNaN.h"

}

extern "C"
{
  real_T rtGetNaN(void)
  {
    return rtNaN;
  }

  real32_T rtGetNaNF(void)
  {
    return rtNaNF;
  }
}
