#include <iostream>
#include "MyStdLib.h"

int main(void)
{
    std::cout << Vector2::getAngle(Vector2(0, 0), Vector2(1, 1.73)) * RAD_TO_DEG << std::endl;
    return 0;
}
