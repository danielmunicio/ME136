#include <eigen3/Eigen/Core>
#include <iostream>

// Define Mixer Matrix 
const Eigen::Matrix<int, 4, 4> mixerMatrix = (Eigen::Matrix<int, 4, 4>() <<
    1, -1,  1,  1,
    1,  1, -1,  1,
    1,  1,  1, -1,
    1,  1, -1, -1).finished();



// Controller Parameters
const Eigen::Vector4i Kp(10, 10, 10, 10);
const Eigen::Vector4i Ki(10, 10, 10, 10);
const Eigen::Vector4i Kd(10, 10, 10, 10);

// Desired State
const Eigen::Vector4i desiredState(0.5, 0, 0, 0)

int main () {
        Eigen::Matrix<short, 5, 5> M1;
        Eigen::Matrix<float, 20, 75> M2;

        // Define Mixer Matrix 
        Eigen::Matrix<int, 4, 4> mixerMatrix;
        mixerMatrix << 1, -1, 1, 1,
                        1, 1, -1, 1,
                        1, 1, 1, -1,
                        1, 1, -1, -1;


        // Define Input vector
        Eigen::Vector4i inputVector(1, 1, 1, 1);
        
        // Multiply Input Vector by Mixer Matrix
        std::cout << "Here is the product of input vector by the mixermatrix \n" << mixerMatrix * inputVector << std::endl;
        return 0;
}


Eigen::Vector4i controller(Eigen::Vector4f stateVector) {
  // State Vector has form 
  // <Height, Roll, Pitch, Yaw>

  // Calculate the Error vector
  Eigen::Vector4f error = stateVector - desiredState;
  
  float thrustInput;
  float rollInput;
  float pitchInput;
  float yawInput;


  Eigen::Vector4f inputVector(thrustInput, rollInput, pitchInput, yawInput);

  // Calculate Input Vector 
  Eigen::Vector4f floatResult = mixerMatrix.cast<float>() * inputVector;
  // Cast Input Vector into int 
  Eigen::Vector4i intResult = floatResult.cast<int>();
}


