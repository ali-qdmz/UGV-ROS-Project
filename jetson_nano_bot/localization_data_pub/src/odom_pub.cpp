/*
 * Automatic Addison
 * Website: https://ubuntu.com
 * Subscribe: ROS node that subscribes to the following topics:
 *  right_ticks : Tick counts from the right motor encoder (std_msgs/Int16)
 * 
 *  left_ticks : Tick counts from the left motor encoder  (std_msgs/Int16)
 * 
 *  imu/data : Data from the Inertial Measurement Unit (IMU) sensor  
 *                 (sensor_msgs/Imu.msg)
 *  initial_2d : The initial position and orientation of the robot.
 *               (geometry_msgs/PoseStamped)
 *
 * Publish: This node will publish to the following topics:
 *  odom_data_euler : Position and velocity estimate. The orientation.z 
 *                    variable is an Euler angle representing the yaw angle.
 *                    (nav_msgs/Odometry)
 *  odom_data_quat : Position and velocity estimate. The orientation is 
 *                   in quaternion format.
 *                   (nav_msgs/Odometry)
 * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */

// Include various libraries
#include "ros/ros.h" 
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

// Create odometry data publishers
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

// Robot physical constants
const double TICKS_PER_REVOLUTION = 620; // For reference purposes.
const double WHEEL_RADIUS = 0.033; // Wheel radius in meters
const double WHEEL_BASE = 0.17; // Center of left tire to center of right tire
const double TICKS_PER_METER = 2880;

// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;

// Flag to see if initial pose has been received
bool initialPoseRecieved = false;

// IMU variables
float imuHeading = 0;
float headingOffset = 0;
bool haveNewImuHeading = false;
bool imuHeadingInitialized = false;

using namespace std;

// Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {

  cout << "Received initial pose." << endl;

  headingOffset = rvizClick.pose.orientation.z - imuHeading;
  
  cout << "heading offset = " << rvizClick.pose.orientation.z << " - " << imuHeading << " = " << headingOffset << endl;

  odomOld.pose.pose.position.x = rvizClick.pose.position.x;
  odomOld.pose.pose.position.y = rvizClick.pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;
  imuHeadingInitialized = true;
}

// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const std_msgs::Int16& leftCount) {

  static int lastCountL = 0;
  if(leftCount.data != 0 && lastCountL != 0) {
		
    int leftTicks = (leftCount.data - lastCountL);

    if (leftTicks > 10000) {
      leftTicks = 0 - (65535 - leftTicks);
    }
    else if (leftTicks < -10000) {
      leftTicks = 65535-leftTicks;
    }
    else{}
    distanceLeft = leftTicks/TICKS_PER_METER;
  }
  lastCountL = leftCount.data;
}

// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::Int16& rightCount) {
  
  static int lastCountR = 0;
  if(rightCount.data != 0 && lastCountR != 0) {

    int rightTicks = rightCount.data - lastCountR;
    
    if (distanceRight > 10000) {
      distanceRight = (0 - (65535 - distanceRight))/TICKS_PER_METER;
    }
    else if (rightTicks < -10000) {
      rightTicks = 65535 - rightTicks;
    }
    else{}
    distanceRight = rightTicks/TICKS_PER_METER;
  }
  lastCountR = rightCount.data;
}

// Publish a nav_msgs::Odometry message in quaternion format
// I will use small, dummy values for the covariance matrix. These values 
// are sometimes provided in the datasheet for the IMU.
void publish_quat() {

  tf2::Quaternion q;
		
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.1;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }

  odom_data_pub_quat.publish(quatOdom);
}

// See if the rate of change is reasonable or if it is an error.
bool isRateOfChangeSafe(float headings[], double times[]) {
	
  double rate[3] = {0};
	
  for (int i = 0; i < 3; i++) {
    double timeElapsed = headings[i] - headings[i+1];
		
    if(timeElapsed != 0) {
      
      // Compute the rate in radians per second
      rate[i] = abs(headings[i] - headings[i+1]) / timeElapsed;
    }
    // .785 rad/sec is 45 degrees/sec
    if(rate[i] > .785 || timeElapsed == 0) { 
    
      return false;
    }
  }
  return true;
}

// Update the heading
void update_heading(const sensor_msgs::Imu &imuMsg) {

  // The zero element has the most recent heading data from the IMU
  static float headings[4] = {0};
  static double times[4] = {0};

  if(imuMsg.orientation_covariance[0] != -1) {
    tf::Quaternion q(imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Record the last three cycles of yaw angle and time stamps
    for(size_t i = 3; i > 0; i--) {
      headings[i] = headings[i-1];
      times[i] = times[i-1];
    }

    headings[0] = yaw;
    times[0] = ros::Time::now().toSec();

    // If this is false, we have a possible bad reading.
    // Get three consecutive stable readings.
    if(isRateOfChangeSafe(headings, times)) {
      
      if(imuHeadingInitialized == false) {
        headingOffset = odomOld.pose.pose.orientation.z - yaw;
        imuHeadingInitialized = true;
        cout << "heading offset = " << odomOld.pose.pose.orientation.z << " - " << yaw << " = " << headingOffset << endl;
      }
      else {
        haveNewImuHeading = true;
        cout << "using imuheading" << endl;
      }
    }
    else {
      haveNewImuHeading = false;
    }
  }
}

// Update odometry information
void update_odom() {

  // Calculate the average distance
  double cycleDistance = (distanceRight + distanceLeft) / 2;
  
  // Calculate the number of radians the robot has turned since the last cycle
  double cycleAngle = asin((distanceRight-distanceLeft)/WHEEL_BASE);

  // Average angle during the last cycle
  double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;
	
  if (avgAngle > PI) {
    avgAngle -= 2*PI;
  }
  else if (avgAngle < -PI) {
    avgAngle += 2*PI;
  }
  else{}

  // Calculate the new pose (x, y, and theta)
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
     || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }

  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
	else{}

  // Compute the velocity
  odomNew.header.stamp = ros::Time::now();
  odomNew.twist.twist.linear.x = cycleDistance/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
  odomNew.twist.twist.angular.z = cycleAngle/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());

  // Update the orientation data if we have a smart heading from the IMU
  if (haveNewImuHeading == true) {
    odomNew.pose.pose.orientation.z = imuHeading+headingOffset;
    haveNewImuHeading = false;
    for (int i = 0; i<36; i++) {
      if (i == 0 || i == 7 || i == 14) {
        odomNew.pose.covariance[i] = .002;
      }
      else if (i == 21 || i == 28 || i== 35) {
        odomNew.pose.covariance[i] = .001;
      }
      else {
        odomNew.pose.covariance[i] = 0;
      }
    }
  }
  else {
    for (int i = 0; i < 36; i++) {
    
      if (i == 0 || i == 7 || i == 14) {
        odomNew.pose.covariance[i] = .001;
      }
      else if (i == 21 || i == 28 || i== 35) {
        if (cycleAngle != 0 )
          odomNew.pose.covariance[i] += .0001;
      }
      else {
        odomNew.pose.covariance[i] = 0;
      }
    }
  }

  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;

  // Publish the odometry message
  odom_data_pub.publish(odomNew);
}

int main(int argc, char **argv) {
  
  // Set the data fields of the odometry message
  odomNew.header.frame_id = "odom";
  odomNew.pose.pose.position.z = 0;
  odomNew.pose.pose.orientation.x = 0;
  odomNew.pose.pose.orientation.y = 0;
  odomNew.twist.twist.linear.x = 0;
  odomNew.twist.twist.linear.y = 0;
  odomNew.twist.twist.linear.z = 0;
  odomNew.twist.twist.angular.x = 0;
  odomNew.twist.twist.angular.y = 0;
  odomNew.twist.twist.angular.z = 0;
  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;

  // Launch ROS and create a node
  ros::init(argc, argv, "odom_pub");
  ros::NodeHandle node;

  // Subscribe to ROS topics
  ros::Subscriber subForRightCounts = node.subscribe("right_ticks", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForLeftCounts = node.subscribe("left_ticks", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);
  ros::Subscriber subImu = node.subscribe("imu/data", 100, update_heading);

  // Publisher of simple odom message where orientation.z is an euler angle
  odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_data_euler", 100);

  // Publisher of full odom message where orientation is quaternion
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_data_quat", 100);

  ros::Rate loop_rate(5); 
	
  while(ros::ok()) {
    
    if(initialPoseRecieved) {
      update_odom();
      publish_quat();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
