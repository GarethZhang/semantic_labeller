//example_ros_class.cpp:
//wsn, Feb 2015
//illustrates how to use classes to make ROS nodes
// constructor can do the initialization work, including setting up subscribers, publishers
// can use member variables to pass data from subscribers to other member functions

// can test this function manually with terminal commands, e.g. (in separate terminals):
// rosrun example_ros_class example_ros_class
// rostopic echo exampleMinimalPubTopic
// rostopic pub -r 4 exampleMinimalSubTopic std_msgs/Float32 2.0


// this header incorporates all the necessary  files and defines the class "semantic_labeller_class"
#include "semantic_labeller/semantic_labeller.hpp"

//CONSTRUCTOR:  this will get called whenever an instance of this class is created
// want to put all dirty work of initializations here
// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
semantic_labeller_class::semantic_labeller_class(){ // constructor
    //initialize variables here, as needed
}


void semantic_labeller_class::PointCloudXYZItoXYZ (const pcl::PointCloud<pcl::PointXYZI>& in,
                                                   pcl::PointCloud<pcl::PointXYZ>& out)
{
    out.width   = in.width;
    out.height  = in.height;
    for (const auto &point : in.points)
    {
        pcl::PointXYZ p;
        p.x = point.x; p.y = point.y; p.z = point.z;
        out.points.push_back (p);
    }
}