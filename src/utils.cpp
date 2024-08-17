#include "utils.h"


double SquareError(double sx, double sy, double fx, double fy) {
  return std::sqrt((fx - sx) * (fx - sx) + (fy - sy) * (fy - sy));
}