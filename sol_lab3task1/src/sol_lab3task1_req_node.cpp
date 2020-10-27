#include "ros/ros.h"
//TODO: Include the header file rotate_point service (look in the package example_msg_srv)
#include "sol_example_msg_srv/rotate_point.h"

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "rotate_point_client");

    ros::NodeHandle nh;

    //TODO: Initialise the client and makes sure that it corresponds to the correct service
    ros::ServiceClient c = nh.serviceClient<sol_example_msg_srv::rotate_point>("rotate_point");

    //TODO: Initialise the service message
    sol_example_msg_srv::rotate_point srv;

    while (nh.ok())
    {   
        //TODO: Generate a random point
    	srand(time(0));
        srv.request.input_p.x = fRand(-2.0, 2.0);
    	srand(time(0));
    	srv.request.input_p.y = fRand(-1.7, 1.7);
    	srand(time(0));
    	srv.request.input_p.z = fRand(-1.0, 1.8);

        //TODO: Generate a random quaternion (make sure its norm is 1)
        std::vector<double> quat, quat_unit;
        
        srand(time(0)); quat.push_back(fRand(-2.0, 2.0));
        srand(time(0)); quat.push_back(fRand(-2.0, 2.0));
        srand(time(0)); quat.push_back(fRand(-2.0, 2.0));
        srand(time(0)); quat.push_back(fRand(-2.0, 2.0));
        
        double magnitute = sqrt(pow(quat[0],2) + pow(quat[1],2) + pow(quat[2],2) + pow(quat[3],2));
        
        quat_unit.push_back(quat[0] / magnitute); quat_unit.push_back(quat[1] / magnitute);
        quat_unit.push_back(quat[2] / magnitute); quat_unit.push_back(quat[3] / magnitute);

        srv.request.q.w = quat_unit[0];
    	srv.request.q.x = quat_unit[1];
    	srv.request.q.y = quat_unit[2];
    	srv.request.q.z = quat_unit[3];

        //TODO: Send a request.
        double begin = ros::Time::now().toSec();;
	    c.call(srv);

	    //TODO: Show the computational time between requests (from sending a request and getting a response).
        double end = ros::Time::now().toSec();;
        double time = end - begin;
        ROS_INFO("The computational time is %.4f seconds\n", time);

        //TODO: Change the output value
        ROS_INFO("The input point (x, y, z) = (%.2f, %.2f, %.2f)\n", srv.request.input_p.x, srv.request.input_p.y, srv.request.input_p.z);
        ROS_INFO("The output point (x, y, z) = (%.2f, %.2f, %.2f)\n", srv.response.output_p.x, srv.response.output_p.y, srv.response.output_p.z);

        usleep(10000);
    }

    return 1;
}
