#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <armadillo2_teleop/armadillo_teleop.h>

Armadillo2Teleop *armadillo_teleop;

void printAxes()
{

}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    /* print axes for testing */
    /*for (int i=0; i<joy->axes.size(); i++)
        fprintf(stderr, "axes[%i]:%f | ", i, joy->axes[i]);
    fprintf(stderr, "\n");*/
    /*for (int i=0; i<joy->buttons.size(); i++)
        fprintf(stderr, "axes[%i]:%f | ", i, joy->buttons[i]);
    fprintf(stderr, "\n");*/

    /* drive robot */
    armadillo_teleop->twist.axis_angular = joy->axes[armadillo_teleop->twist.joy_axis_angular];
    armadillo_teleop->twist.axis_linear = joy->axes[armadillo_teleop->twist.joy_axis_linear];
    armadillo_teleop->drive();

    /* move torso */
    //TODO: INIT WITH JOINT INITIAL STATE TO PREVENT MOVEMENT TO 0 ON STARTUP
    //TODO: ALOW STOP FUNTCTION
    //TODO: ADD DEBOUNCER FOR BUTTONS
    if (joy->axes[armadillo_teleop->torso.joy_axis_updown] == 1)
        armadillo_teleop->torso.axis_updown += armadillo_teleop->torso.inc_updown;
    else if (joy->axes[armadillo_teleop->torso.joy_axis_updown] == -1)
        armadillo_teleop->torso.axis_updown -= armadillo_teleop->torso.inc_updown;
    fprintf(stderr, "[%f]\n", armadillo_teleop->torso.axis_updown );
    if (joy->axes[armadillo_teleop->torso.joy_axis_updown] != 0)
        armadillo_teleop->moveTorso();

}


int main(int argc, char** argv) {

    ros::init(argc, argv, "armadillo2_teleop_node");
    ros::NodeHandle nh;
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
    armadillo_teleop = new Armadillo2Teleop(nh);
    ros::spin();
    delete armadillo_teleop;
    return 0;
}

