
#ifndef _SAVE_FILES_
#define _SAVE_FILES_
#include <fstream>

#define csvout(outport, x, flag) \
  if (flag)                      \
  {                              \
    outport << #x << ",";        \
  }                              \
  else                           \
  {                              \
    outport << x << ",";         \
  }

#define csvoutN(outport, x, n, flag)                 \
  if (flag)                                          \
  {                                                  \
    for (int i = 0; i < n; ++i)                      \
    {                                                \
      outport << #x << "[" << i << "]/" << n << ","; \
    }                                                \
  }                                                  \
  else                                               \
  {                                                  \
    for (int j = 0; j < n; ++j)                      \
    {                                                \
      outport << x[j] << ",";                        \
    }                                                \
  }

  #endif
