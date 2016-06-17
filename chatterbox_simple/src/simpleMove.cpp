#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"
#include "ca_msgs/Bumper.h"

static const float moveRate = 0.5;
static const float turnRate = 1.0;
static const int extraRays = 6;     // Even number please! Half will be added to either side of the target ray
static const float wallThreshould = 0.8;
static const int backupDuration = 20;
static const int turnDuration = 20;

nav_msgs::Odometry::ConstPtr odomVal;
ros::Publisher cmd_velPub;

bool obstaclesInWay(const sensor_msgs::LaserScan::ConstPtr& laser){
    int midRange = ((laser -> ranges).size())/2;
    float midVal = (laser -> ranges)[midRange]; 
    for(int i = 1; i <= extraRays/2; i++){              // For redundancy
        midVal +=   (laser->ranges)[midRange+i] + 
                    (laser->ranges)[midRange-i]; 
    }
    midVal /= (extraRays + 1);
    return midVal < wallThreshould; 
}

bool obstaclesInWay(const ca_msgs::Bumper::ConstPtr& bumper){
    return (bumper->is_left_pressed) || (bumper->is_right_pressed);
}

void bumperCallBack(const ca_msgs::Bumper::ConstPtr& bumper){
	static int turnCount = 0;
	static int backupCount = 0;
	static int turnDir = turnRate;
	static bool turning = false;

	geometry_msgs::Twist msg;
	if(!obstaclesInWay(bumper) && !turning){
	    	msg.linear.x = moveRate;
	} else { 
		if (backupCount == 0 && turnCount == 0){
			turning = true;
			backupCount = rand() % backupDuration + backupDuration;
			ROS_INFO("[Start Backup]\n");
		} else if (backupCount > 0 && turnCount == 0){
			msg.linear.x = -moveRate;	
			backupCount--;
			ROS_INFO("[Backs UP...]\n");
			if (backupCount == 0){
				turnCount = rand() % turnDuration + turnDuration;
				turnDir = pow(-1,rand()%2) * turnRate;
				ROS_INFO("[Start Turn] direction: %i\n", turnDir);
			}
		} else {
			msg.angular.z = turnDir;
			turnCount--;
			ROS_INFO("[Turning...]\n");
			if (turnCount == 0)
				turning = false;
		}			
	}
	cmd_velPub.publish(msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "simpleMove");
    ros::NodeHandle n;
    ros::Subscriber bumperSub = n.subscribe("bumper", 100, bumperCallBack);
    cmd_velPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::spin();
    return 0;
}
