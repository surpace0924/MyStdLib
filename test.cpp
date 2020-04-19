#include <iostream>
#include "MyStdLib.h"

int main(void)
{
    std::cout << (myStd::Pose2D(5, 2, 3.2) - myStd::Pose2D(1, 1.73, 0)) << std::endl;
    return 0;
}
