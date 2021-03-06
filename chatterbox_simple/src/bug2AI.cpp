#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"

// ===> Switch Variables
static const float sink_x = 7.0;                
static const float sink_y = 7.0;
static const float goalThreshould = 1.5;        // Acceptable error near goal
static const float wallThreshould = 0.8;    // Distance to obstacle considered blocking
static const float stopDistance = 0.2;
static const float angleThreshould = 0.2;
static const int   extraRays = 40;         // Must be even. Check addition laser rays near target ray to reduce uncertainty
static const float turnRate = 1.0;
static const float moveRate = 1.0;

// ===> Global Variables (w/r robot)
ros::Publisher cmd_velPub;
nav_msgs::Odometry::ConstPtr odomVal;
bool atGoal = false;
bool goalSeek = true;
bool blocked = false;

// PURPOSE: get distance to goal
// PARAMS:  current x and y coordinates (floats)
float getDistance(float xDist, float yDist){
    return sqrt(pow(xDist-sink_x, 2) + pow(yDist-sink_y, 2));
}

// PURPOSE: get angle between vector from robot to goal and the x-axis 
// PARAMS:  current x and y coordinates (floats)
// NOTE:    above the x-axis ccw is from 0~pi, below x-axis ccw -pi~0
float getAngle(float xDist, float yDist){
   float delta_x = sink_x - xDist;
   float delta_y = sink_y - yDist;
   float angleSize = acos(delta_x/sqrt(pow(delta_x,2)+pow(delta_y,2)));
   return (yDist < sink_y) ? angleSize : -1 * angleSize;
}

// PURPOSE: determine if the robot is facing an obstacle
// PARAMS:  laser readings (sensor_msg/LaserScan)
bool faceObstacle(const sensor_msgs::LaserScan::ConstPtr& laser){
    int midRange = ((laser -> ranges).size())/2;
    float midVal = (laser -> ranges)[midRange]; 
    for(int i = 1; i <= extraRays/2; i++){     // min of all rays for redundancy
        midVal = std::min(midVal,
                 std::min((laser->ranges)[midRange+i],
                          (laser->ranges)[midRange-i])); 
    }
    //ROS_INFO("[Front Obstacle] midVal: %.2f\n", midVal);
    return midVal < wallThreshould;
}

// PURPOSE: determine if the direction towards the goal is blocked
// PARAMS:  laser readings (sensor_msg/LaserScan)
bool obstaclesInWay(float goalAngle, const sensor_msgs::LaserScan::ConstPtr& laser){
    float angleInc = (laser->angle_increment);
    int midRange = ((laser->ranges).size())/2;
    int raysOffCenter = (goalAngle)/angleInc;
    int checkIndex = midRange + raysOffCenter;   
    float checkRay = (laser->ranges)[checkIndex];
    for (int i = 1; i <= extraRays/2; i++){
        checkRay = std::min(checkRay,
                   std::min((laser->ranges)[checkIndex+i],
                            (laser->ranges)[checkIndex-i]));
    }
    //ROS_INFO("[To Goal Obstacle] Ray: %.2f Goal Angle: %.2f OffCenter: %i\n", checkRay, goalAngle, raysOffCenter);
    return checkRay < wallThreshould;
}

bool tooClose(const sensor_msgs::LaserScan::ConstPtr& laser){
    int numRays = (laser->ranges).size();
    for (int i = 0; i < numRays; i++){
        if ( (laser->ranges)[i] < stopDistance )
            return true;
    }
    return false;
}

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser){
    if (odomVal){
        float xx = (odomVal->pose).pose.position.x;
        float yy = (odomVal->pose).pose.position.y;
        float theta = tf::getYaw((odomVal->pose).pose.orientation);
        float goalDist = getDistance(xx, yy);
        float goalAngle = getAngle(xx, yy) - theta; 
        //ROS_INFO("Goal distance: %.2f\n", goalDist);
    
        geometry_msgs::Twist msg;

        if (!atGoal){
            blocked = faceObstacle(laser);
            if (goalDist < goalThreshould){
                ROS_INFO("[At Goal!]\n");
                atGoal = true;
            } else {
                msg.linear.x = (!blocked && !tooClose(laser)) ? moveRate : 0;
                if (goalSeek) {
                    if (fabs(goalAngle) > angleThreshould)
                        msg.angular.z = (goalAngle > 0) ? turnRate : turnRate * (-1);
                    //ROS_INFO("[Seeking Goal] angleDiff:%.2f blocked: %i\n", goalAngle, blocked);
                    if(blocked || obstaclesInWay(goalAngle, laser))
                        goalSeek = false;
                } else {
                    msg.angular.z = blocked ? turnRate : 0;
                    //ROS_INFO("[Wall Follow] theta: %.2f goalAngle: %.2f blocked: %i\n", theta, goalAngle, blocked);
                    if(!obstaclesInWay(goalAngle, laser))
                        goalSeek = true;
                }
            }
        } else if (goalDist < goalThreshould){
            blocked = faceObstacle(laser);
            if (!blocked)
                msg.linear.x = moveRate;
            //ROS_INFO("[At Goal!] faceObstacle: %i\n", blocked);  
        }
        cmd_velPub.publish(msg);
    }
}

void odomCallBack(const nav_msgs::Odometry::ConstPtr& odom){
    odomVal = odom;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "bug2AI");
    ros::NodeHandle n;
    ros::Subscriber laserSub = n.subscribe("base_scan", 100, laserCallBack);
    ros::Subscriber odomSub = n.subscribe("base_pose_ground_truth", 100, odomCallBack);
    cmd_velPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::spin();
    return 0;
}
