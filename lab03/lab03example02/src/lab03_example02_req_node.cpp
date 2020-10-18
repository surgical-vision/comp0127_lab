#include "ros/ros.h"
#include "example_msg_srv/test_srv.h"

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rot_convert_client");
    ros::NodeHandle nh;
    ros::ServiceClient c = nh.serviceClient<example_msg_srv::test_srv>("rot_convert"); //Initialise client for the service "rot_convert"

    example_msg_srv::test_srv srv; //Initialise request message (defined by test_srvRequest).

    while (nh.ok())
    {
	srand(time(0));
        srv.request.x.data = fRand(-2.0, 2.0);
	srand(time(0));
	srv.request.y.data = fRand(-1.8, 1.7);
	srand(time(0));
	srv.request.z.data = fRand(-2.9, 1.6);
	if (c.call(srv)) //Get the response from the service.
        	ROS_INFO("The resulting quaternion (qx, qy, qz, qw): (%.3f, %.3f, %.3f, %.3f)\n", srv.response.q.x, srv.response.q.y, srv.response.q.z, srv.response.q.w);

        usleep(10000);
    }

    return 123;
}
