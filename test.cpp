#include <iostream>
#include "MyStdLib.h"

int main(void)
{
    myStd::PID<double> pid(1.0, 1, 1);
    pid.setMode(myStd::PID<double>::Mode::pPID);
    // for (int i = 0; i < 100; ++i)
    // {
    //     pid.update(100, i, 0.01);
    //     std::cout << pid.getControlVal() << std::endl;
    // }

    std::vector<myStd::Pose2D<double>> pose = {
        {5.0, 2.0, 3.0},
        {1.0, 1.0, 0.0}};

    myStd::PurePursuitControl<double, myStd::PID<double>> ppc(pose);
    ppc.push_back(myStd::Pose2D<double>(0.0, 2.0, 2.0));
    ppc.setController(pid, pid);
    for (int i = 0; i < 3; ++i)
    {
        myStd::Pose2D<double> now_pose(i, 0, 0);
        ppc.update(i, now_pose, 0.01);
        std::cout << ppc.getControlVal() << std::endl;
    }

    return 0;
}
