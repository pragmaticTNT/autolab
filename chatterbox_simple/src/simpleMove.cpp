#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"

static const float moveRate = 1.0;
static const float turnRate = 2.0;
static const int extraRays = 6;     // Even number please! Half will be added to either side of the target ray
static const float angleThreshould = 0.2;
static const float wallThreshould = 0.8;

nav_msgs::Odometry::ConstPtr odomVal;
ros::Publisher cmd_velPub;
bool move = true;
bool initTurn = true;
float initAngle = 0;

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

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser){
    if (odomVal){
        float theta = tf::getYaw((odomVal->pose).pose.orientation);
        geometry_msgs::Twist msg;
        if(!obstaclesInWay(laser)){
            msg.linear.x = (float)(rand()%10 + 1)/4.0;
        } else {
            msg.angular.z = turnRate;
        }
        cmd_velPub.publish(msg);
    }
}

void odomCallBack(const nav_msgs::Odometry::ConstPtr& odom){
    odomVal = odom;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "simpleMove");
    ros::NodeHandle n;
    ros::Subscriber laserSub = n.subscribe("base_scan", 100, laserCallBack);
    ros::Subscriber odomSub = n.subscribe("base_pose_ground_truth", 100, odomCallBack);
    cmd_velPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::spin();
    return 0;
}
