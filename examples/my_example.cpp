#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

int main(int argc, char** argv){
    std::cout << "The value of argc is: " << argc << std::endl;
    try
    {
        franka::Robot robot(argv[1]);
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        
    }
    catch (const franka::Exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cout << "An error occurred" << std::endl;
    }
    
    std::cout << "This is my super stupid exampele"
    << "Please make sure that you are not stupid" << std::endl;
    return 0;
}