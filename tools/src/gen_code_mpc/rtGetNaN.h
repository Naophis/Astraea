#ifndef rtGetNaN_h_
#define rtGetNaN_h_

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rtwtypes.h"
#ifdef __cplusplus

extern "C"
{

#endif

  extern real_T rtGetNaN(void);
  extern real32_T rtGetNaNF(void);

#ifdef __cplusplus

}

#endif
#endif

