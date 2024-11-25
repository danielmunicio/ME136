#include <eigen3/Eigen/Core>
#include <iostream>

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