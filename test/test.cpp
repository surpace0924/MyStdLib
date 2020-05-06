#include <iostream>
#include "./../MyStdLib/MyStdLib.h"

int main(void)
{
    myStd::PID<double> pid(2.0, 0, 0);
    pid.setMode(myStd::PID<double>::Mode::pPID);
    std::vector<myStd::Pose2D<double>> pose = {
        {0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0}};

    myStd::PurePursuitControl<double, myStd::PID<double>> ppc(pose);
    ppc.push_back(myStd::Pose2D<double>(2.0, 3.0, 0.0));
    ppc.setController(pid, pid);
    myStd::Pose2D<double> now_pose(0.5, 0.5, 3.14 / 4);
    ppc.update(1, now_pose, 0.01);
    std::cout << ppc.getControlVal() << std::endl;

    return 0;
}
