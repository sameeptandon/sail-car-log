#pragma once

#include "time.h"

#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      cerr << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz. " << "\n"; \
      count = 0; \
      last = now; \
    } \
}while(false)
