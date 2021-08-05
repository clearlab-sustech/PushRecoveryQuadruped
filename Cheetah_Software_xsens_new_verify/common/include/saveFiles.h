#ifndef _SAVE_FILES_H_
#define _SAVE_FILES_H_

#include <cstdio>
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
    for (int i_save = 0; i_save < n; ++i_save)                      \
    {                                                \
      outport << #x << "[" << i_save << "]/" << n << ","; \
    }                                                \
  }                                                  \
  else                                               \
  {                                                  \
    for (int j_save = 0; j_save < n; ++j_save)                      \
    {                                                \
      outport << x[j_save] << ",";                        \
    }                                                \
  }

  #endif