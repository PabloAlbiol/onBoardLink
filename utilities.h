#ifndef UTILITIES_H_INCLUDED
#define UTILITIES_H_INCLUDED

#include <time.h>
#include <sys/time.h>

// Returns (a - b) in microseconds
inline long int timeDiff(struct timeval *a, struct timeval *b)
{
  return ((a->tv_sec*1000000 + a->tv_usec) - (b->tv_sec*1000000 + b->tv_usec));
}

inline void setBitBitfield(uint8_t *val, uint8_t bitIndex)
{
    *val |= (1 << bitIndex);
}

inline void clearBitBitfield(uint8_t *val, uint8_t bitIndex)
{
    *val &= ~(1 << bitIndex);
}

inline bool getBitBitfield(uint8_t val, uint8_t bitIndex)
{
    return (val & (1 << bitIndex));
}

#endif // UTILITIES_H_INCLUDED
