#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//set target motor speed
float desiredMotorSpeed = 1000;

// Initialize variables
// Gyroscope Calibration
Vec3f estGyroBias = Vec3f(0, 0, 0);
// Corrected Gyroscope Readings
Vec3f rateGyro_corr = Vec3f(0, 0, 0);
// Estimated Roll Pitch and Yaws
float estRoll = 0.0f;
float estPitch = 0.0f;
float estYaw = 0.0f;

// Set rho variable, used for attitude estimation
float rho = 0.01f;

// Last inputs and outputs for debugging purposes
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

// Some constants that we may use:
const float mass = 30e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]

const float dt = 1.0f / 500.0f; //[s] period between successive calls to MainLoop

// Main Loop
MainLoopOutput MainLoop(MainLoopInput const &in) {
  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;

  // Make sure motors are not moving
  outVals.motorCommand1 = 0;
  outVals.motorCommand2 = 0;
  outVals.motorCommand3 = 0;
  outVals.motorCommand4 = 0;

  // Use the first second to calibrate the Gyroscope
  // Get the approx error by averaging Gyroscope readings from the first second (dt = 1/500)
  if (in.currentTime < 1.0f) {
    estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
  }
  // Correct Gyroscope value by subtracting Bias
  Vec3f rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;

  // Store our IMU Measurements
  lastMainLoopInputs.imuMeasurement.accelerometer = in.imuMeasurement.accelerometer;
  lastMainLoopInputs.imuMeasurement.rateGyro = in.imuMeasurement.rateGyro;

  // Estimate Roll Pitch and Yaw using corrected integrator and accelerometer measurements
  float phiMeasured = in.imuMeasurement.accelerometer.y / gravity;
  float thetaMeasured = -1.0f * in.imuMeasurement.accelerometer.x / gravity;

  estPitch = (1.0f - rho) * (estPitch + dt * rateGyro_corr.y) +  (rho * thetaMeasured);
  estRoll = (1.0f - rho) * (estRoll + dt * rateGyro_corr.x) + (rho * phiMeasured);
  estYaw = estYaw + dt * rateGyro_corr.z;


  // Log values in the Log output folder
  outVals.telemetryOutputs_plusMinus100[0] = estRoll;
  outVals.telemetryOutputs_plusMinus100[1] = estPitch;
  outVals.telemetryOutputs_plusMinus100[2] = estYaw;

  // If we press the blue button, reset the pitch. This is just so we didn’t have to disconnect the quadcopter between trials.
  if (in.joystickInput.buttonBlue == true) {
  estPitch = 0.0f;
  }
  return outVals;
}

// Print accelerometer and gyro rates, intermediate values, and estimated pitch
void PrintStatus() {
  //Accelerometer measurements
  printf("Acc: ");
  printf("x=%6.3f, ", (double)(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("y=%6.3f, ", (double)(lastMainLoopInputs.imuMeasurement.accelerometer.y));
  printf("z=%6.3f\n", (double)(lastMainLoopInputs.imuMeasurement.accelerometer.z));

  // Gyro rate measurements
  printf("Gyro: ");
  printf("x=%6.3f, ", (double)(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("y=%6.3f, ", (double)(lastMainLoopInputs.imuMeasurement.rateGyro.y));
  printf("z=%6.3f\n", (double)(lastMainLoopInputs.imuMeasurement.rateGyro.z));

  // Print calculated gyro rate bias
  printf("Bias:  = (%6.3f, %6.3f, %6.3f)\n",
      double(estGyroBias.x), double(estGyroBias.y),
      double(estGyroBias.z));

  // Print corrected gyro rates
  printf("Post Calibration Values:  = (%6.3f, %6.3f, %6.3f)\n",
      double(rateGyro_corr.x), double(rateGyro_corr.y),
      double(rateGyro_corr.z));

  // Print calculated estimated pitch
  printf("Estimated Pitch: %6.3f\n", (double)(estPitch));
}

