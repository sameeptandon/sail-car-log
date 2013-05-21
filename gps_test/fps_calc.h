#pragma once

#include "time.h"

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned countr = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++countr; \
    if (now - last >= 1.0) \
    { \
      cerr << "Average framerate("<< _WHAT_ << "): " << double(countr)/double(now - last) << " Hz.\n"; \
      countr = 0; \
      last = now; \
    } \
}while(false)
