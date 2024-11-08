#include "UtilityFunctions.hpp"

int pwmCommandFromSpeed(float desiredSpeed_rad_per_sec) {
  // Replace these two coefficients with what you get
  // in the experiment. Note the trailing "f" after the
  // number -- this ensures that we use single precision
  // floating point (rather than double precision, which
  // would be substantially slower on the microcontroller).
  float a = -106.0f;  // the zeroth order term
  float b = 0.18f;  // the first order term

  // Check if the PWM is above 0, if not, return 0
  if (int(a + b * desiredSpeed_rad_per_sec) > 0) {
  return int(a + b * desiredSpeed_rad_per_sec);
  } else {
  return 0;
  }
}

float speedFromForce(float desiredForce_N) {
  // Remember to add the trailing "f" for single
  // precision!
  float const propConstant = 5.46e-08;

  //we implement a safety check,
  //  (no sqrtf for negative numbers)
  if (desiredForce_N <= 0) {
  return 0.0f;
  }

  return sqrtf(desiredForce_N / propConstant);
}
