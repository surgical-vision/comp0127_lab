#include "ros/ros.h"
//TODO: Include the header file rotate_point service (look in the package example_msg_srv)
#include "sol_example_msg_srv/rotate_point.h"

//TODO: Complete this function
bool rotate_point_3d(sol_example_msg_srv::rotate_point::Request &req, sol_example_msg_srv::rotate_point::Response &res){
    double x = req.input_p.x; double y = req.input_p.y; double z = req.input_p.z;
    double qw = req.q.w; double qx = req.q.x;
    double qy = req.q.y; double qz = req.q.z;
    double new_x; double new_y; double new_z;

    //quaternion multiplication
    new_x = (1 - 2*pow(qy,2) - 2*pow(qz,2))*x + (2*qx*qy+2*qz*qw)*y + (2*qx*qz-2*qy*qw)*z;
    new_y = (2*qx*qy-2*qz*qw)*x + (1 - 2*pow(qx,2) - 2*pow(qz,2))*y + (2*qy*qz+2*qx*qw)*z;
    new_z = (2*qx*qz+2*qy*qw)*x + (2*qy*qz-2*qx*qw)*y + (1 - 2*pow(qx,2) - 2*pow(qy,2))*z;

    res.output_p.x = new_x; res.output_p.y = new_y; res.output_p.z = new_z;

    return true;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_point_server");

    ros::NodeHandle nh;

    //TODO: Initialise the service 'rotate_point' and have it calls the function 'rotate_point_3d'
    ros::ServiceServer server = nh.advertiseService("rotate_point", rotate_point_3d);

    ros::spin();

    return 1;
}
