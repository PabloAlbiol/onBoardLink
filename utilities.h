#ifndef UTILITIES_H_INCLUDED
#define UTILITIES_H_INCLUDED

#include <time.h>
#include <sys/time.h>

// Returns (a - b) in microseconds
static long int timeDiff(struct timeval *a, struct timeval *b)
{
  return ((a->tv_sec*1000000 + a->tv_usec) - (b->tv_sec*1000000 + b->tv_usec));
}


#endif // UTILITIES_H_INCLUDED
