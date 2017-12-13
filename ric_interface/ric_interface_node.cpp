#include <ros/ros.h>
#define LOOP_HZ 50.0
#define THREADS_NUM 2

#include <ric_interface/ric_interface.h>

ric_interface::RicInterface bm;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ric_interface_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner asyncSpinner(THREADS_NUM);
    asyncSpinner.start();

    bm.connect("/dev/armadillo2/RICBOARD");

    while (ros::ok())
    {
        bm.loop();
        ric_interface::protocol::servo actu_pkg;
        actu_pkg.cmd = 2000;
        bm.writeCmd(actu_pkg, sizeof(ric_interface::protocol::servo), ric_interface::protocol::Type::SERVO);
        ros::Duration(1 / LOOP_HZ).sleep();
        ros::spinOnce;
    }
}




