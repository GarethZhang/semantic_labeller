// example_ros_class.h header file //
// wsn; Feb, 2015
// include this file in "example_ros_class.cpp"

// here's a good trick--should always do this with header files:
// create a unique mnemonic for this header file, so it will get included if needed,
// but will not get included multiple times

#ifndef SEMANTIC_LABELLER_CLASS_H_
#define SEMANTIC_LABELLER_CLASS_H_

//some generically useful stuff to include...
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

////message types used in this example code;  include more message types, as needed
//#include <std_msgs/Bool.h>
//#include <std_msgs/Float32.h>

// define a class, including a constructor, member variables and member functions
class semantic_labeller_class
{
public:
    semantic_labeller_class(); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    static void PointCloudXYZItoXYZ (const pcl::PointCloud<pcl::PointXYZI>& in,
                                     pcl::PointCloud<pcl::PointXYZ>& out);
private:

}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match