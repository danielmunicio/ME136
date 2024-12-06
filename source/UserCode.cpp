#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf
#include <iostream>
//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//set target motor speed
float desiredMotorSpeed = 0;
// Initialize variables
// Gyroscope Calibration
Vec3f estGyroBias = Vec3f(0, 0, 0);
// Corrected Gyroscope Readings
Vec3f rateGyro_corr = Vec3f(0, 0, 0);
// Estimated Roll Pitch and Yaws
float estRoll = 0.0f;
float estPitch = 0.0f;
float estYaw = 0.0f;

float lastHeightMeas_meas = 0;
float lastHeightMeas_time = 0;

float estHeight = 0;
float estVelocity_1 = 0;
float estVelocity_2 = 0;
float estVelocity_3 = 0;

// Set rho variable, used for attitude estimation
float rho = 0.01f;



float debug1 = 0.0f;
float debug2 = 0.0f;

// Last inputs and outputs for debugging purposes
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

// Some constants that we may use:
const float mass = 30e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]
const float l = 33e-3f; // Propeller Distance
const float dt = 1.0f / 500.0f; //[s] period between successive calls to MainLoop
const float k = 0.01;

// Controller Constants
// --------------------------------------------------------------
float const timeConstant_rollRate = 0.05f;
float const timeConstant_pitchRate = timeConstant_rollRate;
float const timeConstant_yawRate = 0.05f;

float const timeConstant_rollAngle = 0.25f;
float const timeConstant_pitchAngle = timeConstant_rollAngle;
float const timeConstant_yawAngle = 0.25f;

const float timeConst_horizVel = 2.0f;

const float natFreq_height = 2.0f;
const float dampingRatio_height = 0.7f;

// --------------------------------------------------------------
// Main Loop
MainLoopOutput MainLoop(MainLoopInput const &in) {
  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;

  // Make sure motors are not moving
  outVals.motorCommand1 = 0;
  outVals.motorCommand2 = 0;
  outVals.motorCommand3 = 0;
  outVals.motorCommand4 = 0;



  // Sensor Bias Calculation
  // -----------------------------------------------------------------------------------
  // Use the first second to calibrate the Gyroscope
  // Get the approx error by averaging Gyroscope readings from the first second (dt = 1/500)
  if (in.currentTime < 10.0f) {
	estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 5000.0f);
  }
  // Correct Gyroscope value by subtracting Bias
  Vec3f rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;
  // ---------------------------------------------------------------------------------------


  // Store our IMU Measurements
  lastMainLoopInputs.imuMeasurement.accelerometer = in.imuMeasurement.accelerometer;
  lastMainLoopInputs.imuMeasurement.rateGyro = in.imuMeasurement.rateGyro;


  // Attitude Estimation:
  // ------------------------------------------------------------------------
  // Estimate Roll Pitch and Yaw using corrected integrator and accelerometer measurements
  float phiMeasured = in.imuMeasurement.accelerometer.y / gravity;
  float thetaMeasured = -1.0f * in.imuMeasurement.accelerometer.x / gravity;

  estPitch = (1.0f - rho) * (estPitch + dt * rateGyro_corr.y) +  (rho * thetaMeasured);
  estRoll = (1.0f - rho) * (estRoll + dt * rateGyro_corr.x) + (rho * phiMeasured);
  estYaw = estYaw + dt * rateGyro_corr.z;
  // -------------------------------------------------------------------------

  // Attitude Controller:
  // --------------------------------------------------------------------------
  Vec3f cmdAngAcc = Vec3f(0, 0, 0);
  Vec3f cmdAngVel = Vec3f(0, 0, 0);
  Vec3f desiredAngle = Vec3f(0, 0, 0);


  cmdAngVel.x = ( -1.0f / timeConstant_rollAngle) * (estRoll - desiredAngle.x);
  cmdAngVel.y = (-1.0f / timeConstant_pitchAngle) * (estPitch - desiredAngle.y);
  cmdAngVel.z = (-1.0f / timeConstant_yawAngle) * (estYaw - desiredAngle.z);

  cmdAngAcc.x = (-1.0f / timeConstant_rollRate) * (rateGyro_corr.x - cmdAngVel.x);
  cmdAngAcc.y = (-1.0f / timeConstant_pitchRate) * (rateGyro_corr.y - cmdAngVel.y);
  cmdAngAcc.z = (-1.0f / timeConstant_yawRate) * (rateGyro_corr.z - cmdAngVel.z);
  // --------------------------------------------------------------------------

  // Vertical State Estimation
  // --------------------------------------------------------------------------
  // Prediction Step
  estHeight = estHeight + estVelocity_3 * dt;
  estVelocity_3 = estVelocity_3 + 0 * dt; // Assume Constant

  // Correction Step
  float const mixHeight = 0.3f;
  if (in.heightSensor.updated) {
	// Check that the measurement is reasonable
	if (in.heightSensor.value < 5.0f) {
  	float hMeas = in.heightSensor.value * cosf(estRoll) * cosf(estPitch);
  	estHeight = (1 - mixHeight) * estHeight + mixHeight * hMeas;

  	float v3Meas = (hMeas - lastHeightMeas_meas)
      	/ (in.currentTime - lastHeightMeas_time);

  	estVelocity_3 = (1 - mixHeight) * estVelocity_3 + mixHeight * v3Meas;

  	// Store this measurement for the next velocity update
  	lastHeightMeas_meas = hMeas;
  	lastHeightMeas_time = in.currentTime;
	}
  }

  // Define State Vector to be roll pitch yaw z for now 
  std::vector<float> stateVector(estPitch, estRoll, estYaw, estHeight);

  //motorInputs = 
  // --------------------------------------------------------------------------

  // Horizontal State Estimation
  // --------------------------------------------------------------------------

  // Correction Step
  float const mixHorizVel = 0.1f;
  if (in.opticalFlowSensor.updated) {
	float sigma_1 = in.opticalFlowSensor.value_x;
	float sigma_2 = in.opticalFlowSensor.value_y;
	float div = (cosf(estRoll) * cosf(estPitch));
	if (div > 0.5f) {
  	float deltaPredict = estHeight / div;

  	float v1Meas = (-sigma_1 + in.imuMeasurement.rateGyro.y) * deltaPredict;
  	float v2Meas = (-sigma_2 - in.imuMeasurement.rateGyro.x) * deltaPredict;

  	estVelocity_1 = (1 - mixHorizVel) * estVelocity_1 + mixHorizVel * v1Meas;
  	estVelocity_2 = (1 - mixHorizVel) * estVelocity_2 + mixHorizVel * v2Meas;
	}
  }
  // -----------------------------------------------------------------------------

  // Horizontal Controller
  // -----------------------------------------------------------------------------
  float desAcc1 = - (1 / timeConst_horizVel) * estVelocity_1;
  float desAcc2 = - (1 / timeConst_horizVel) * estVelocity_2;

  float desRoll = -desAcc2 / gravity;
  float desPitch = desAcc1 / gravity;
  float desYaw = 0;
  // ------------------------------------------------------------------------------

  // Vertical Controller
  // ------------------------------------------------------------------------------
  const float desHeight = 0.5f;
  const float desAcc3 = -2 * dampingRatio_height * natFreq_height * estVelocity_3 - natFreq_height * natFreq_height * (estHeight - desHeight);

  // ------------------------------------------------------------------------------
  // Inputs to Mixer Matrix
  Vec3f desTorque = Vec3f(0, 0, 0);
  desTorque.x = cmdAngAcc.x * inertia_xx;
  desTorque.y = cmdAngAcc.y * inertia_yy;
  desTorque.z = cmdAngAcc.z * inertia_zz;

  float desNormalizedAcceleration = (gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));
  debug1 = desNormalizedAcceleration;
  float totalForce = desNormalizedAcceleration * mass;


  // Mixer Matrix
  // ------------------------------------------------------------------------
  float cp1 = (0.25) * (totalForce + (desTorque.x / l) + (desTorque.y / -1.0f * l) + (desTorque.z / k));
  float cp2 = (0.25) * (totalForce + (desTorque.x / -1.0f * l) + (desTorque.y / -1.0f * l) + (-1.0f * desTorque.z / k));
  float cp3 = (0.25) * (totalForce + (desTorque.x / -1.0f * l) + (desTorque.y / l) + (desTorque.z / k));
  float cp4 = (0.25) * (totalForce + (desTorque.x / l) + (desTorque.y / l) + (desTorque.z / -1.0f * k));

  // -------------------------------------------------------------------------

  // Convert Forces -> Propeller Speedes -> PWM Commands
  // --------------------------------------------------
  float speed1 = speedFromForce(cp1);
  float speed2 = speedFromForce(cp2);
  float speed3 = speedFromForce(cp3);
  float speed4 = speedFromForce(cp4);

  int pwm1 = pwmCommandFromSpeed(speed1);
  int pwm2 = pwmCommandFromSpeed(speed2);
  int pwm3 = pwmCommandFromSpeed(speed3);
  int pwm4 = pwmCommandFromSpeed(speed4);
  // -------------------------------------------------
  debug2 = pwm1;

  outVals.motorCommand1 = pwm1;
  outVals.motorCommand2 = pwm2;
  outVals.motorCommand3 = pwm3;
  outVals.motorCommand4 = pwm4;

  // Log values in the Log output folder
  outVals.telemetryOutputs_plusMinus100[0] = estRoll;
  outVals.telemetryOutputs_plusMinus100[1] = estPitch;
  outVals.telemetryOutputs_plusMinus100[2] = estYaw;
  outVals.telemetryOutputs_plusMinus100[3] = estVelocity_1;
  outVals.telemetryOutputs_plusMinus100[4] = estVelocity_2;
  outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
  outVals.telemetryOutputs_plusMinus100[6] = estHeight;
  outVals.telemetryOutputs_plusMinus100[7] = desRoll;
  outVals.telemetryOutputs_plusMinus100[8] = desPitch;
  outVals.telemetryOutputs_plusMinus100[9] = desNormalizedAcceleration;


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


  printf("Desired Acceleration: %6.3f", double(debug1));
  printf("Motor 1 PWM: %6.3f\n", (double)(debug2));
//  printf("Last Range = %6.3fm, ", \
//     	double(lastMainLoopInputs.heightSensor.value));
//  printf("Last Flow: x=%6.3f, y= %6.3f\n", \
//  	double (lastMainLoopInputs.opticalFlowSensor.value_x), \
//  	double(lastMainLoopInputs.opticalFlowSensor.value_y));
//
//  // Print calculated estimated pitch
//  printf("Estimated Pitch: %6.3f\n", (double)(estPitch));
//  printf("Desired Motor Speed: %6.3f\n", (double)(desiredMotorSpeed));
//  printf("Motor 1: %6.3f\n", (double)(debug1));
//  printf("Motor 2: %6.3f\n", (double)(debug2));
//  printf("Motor 3: %6.3f\n", (double)(debug3));
//  printf("Motor 4: %6.3f\n", (double)(debug4));
}

