#ifndef RNG_6HY8TBAM
#define RNG_6HY8TBAM

/*
File: RNG.h
Author: Samuel Barrett
Description: a random number generator based on tinymt32
Created:  2011-08-23
Modified: 2011-08-23
*/

#include "tinymt32.h"
#include <cstdlib>
#include <vector>

class RNG {
public:
  RNG (uint32_t seed) {
    internal.mat1 = 0x8f7011ee;
    internal.mat2 = 0xfc78ff1f;
    internal.tmat = 0x3793fdff;
    tinymt32::tinymt32_init(&internal,seed);
  }
  float randomFloat() {
    return tinymt32::tinymt32_generate_float(&internal);
  }

  uint32_t randomUInt() {
    return tinymt32::tinymt32_generate_uint32(&internal);
  }

  int32_t randomInt(int32_t max) {
    return tinymt32::tinymt32_generate_uint32(&internal) % max;
  }

  int32_t randomInt(int32_t min,int32_t max) {
    uint32_t temp = tinymt32::tinymt32_generate_uint32(&internal);
    int32_t val = temp % (max - min) - min;
    return val;
  }
  
  void randomOrdering(std::vector<uint32_t> &inds) {
    uint32_t j;
    uint32_t temp;
    for (uint32_t i = 0; i < inds.size(); i++)
      inds[i] = i;
    for (int i = (int)inds.size()-1; i >= 0; i--) {
      j = randomInt(i+1);
      temp = inds[i];
      inds[i] = inds[j];
      inds[j] = temp;
    }
  }

private:
  tinymt32::tinymt32_t internal;
};

#endif /* end of include guard: RNG_6HY8TBAM */
