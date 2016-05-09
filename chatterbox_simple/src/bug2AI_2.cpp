#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"

// ===> Switch Variables
static const float[] sink = {7.0, 7.0};     // Sink coordinates                
static const float goalThreshould = 1.5;    // Acceptable error near goal
static const float wallThreshould = 0.8;    // Closest distance to stationary obstacle
static const float stopThreshould = 0.2;    // Distance at which something (probably another robot) is considered too close!
static const float angleThreshould = 0.2;   // Acceptable facing angle towards the goal 
static const float turnRate = 1.0;
static const float moveRate = 1.0;
static const int avoidTime = 5;

// ===> Global Variables (w/r robot)
ros::Publisher cmd_velPub;
nav_msgs::Odometry::ConstPtr odomVal;

// PURPOSE: get distance to goal
// PARAMS:  current x and y coordinates (floats)
float getDistance(float xDist, float yDist){
    return sqrt(pow(xDist-sink[0], 2) + pow(yDist-sink[1], 2));
}

// PURPOSE: get angle between vector from robot to goal and the x-axis 
// PARAMS:  current x and y coordinates (floats)
// NOTE:    above the x-axis ccw is from 0~pi, below x-axis ccw -pi~0
float getAngle(float xDist, float yDist){
   float delta_x = sink[0] - xDist;
   float delta_y = sink[0] - yDist;
   float angleSize = acos(delta_x/sqrt(pow(delta_x,2)+pow(delta_y,2)));
   return (yDist < sink_y) ? angleSize : -1 * angleSize;
}

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser){
    static bool atGoal      = false;
    static bool goalSeek    = true;
    static bool blocked     = false;
    static bool stopped     = false;
    static int avoidCount   = 0;
    static int avoidTurn    = turnAngle;
    static float angleInc   = (laser->angle_increment);
    static int sampleCount  = (laser->ranges).size();
    static int midRayIndex  = sampleCount/2;
    static int raySector    = sampleCount/3;

    if (odomVal){
        // ===> Initialize current robot position and heading
        float xx = (odomVal->pose).pose.position.x;
        float yy = (odomVal->pose).pose.position.y;
        float theta = tf::getYaw((odomVal->pose).pose.orientation);
        float goalDist = getDistance(xx, yy);
        float goalAngle = getAngle(xx, yy) - theta; 
        //ROS_INFO("Goal distance: %.2f\n", goalDist);

        geometry_msgs::Twist msg;

        if (!atGoal){
            // ===> Evaluate laser scanner data
            float minLeft = 5.0;
            float minRight = 5.0;
            //float minCheck = 5.0;
            //int raysOffCenter = (goalAngle)/angleInc;
            //int checkIndex = midRayIndex + raysOffCenter;

            for (std::size_t i = 0; i < sampleCount; i++){
                // -> Front third of rays: check obstruction
                if (i > raySector && i < (sampleCount - raySector)){
                    if ((laser->ranges)[i] < wallThreshould)
                        blocked = true;
                }
                // -> All rays: check if anything is too close
                if ((laser->ranges)[i] < stopThreshould){
                    stopped = true;
                }
                // -> Right to Left rays: check min distance
                if (i > sampleCount/2){
                    minRight = std::min(minRight, (laser->ranges)[i]);
                } else{ 
                    minLeft = std::min(minLeft, (laser->ranges)[i]); 
                }
                // -> Check raySector towards ray point at goal
                //if (i < checkIndex+raySector/2 &&
                //    i > checkIndex-raySector/2)
                //    minCheck = std::min(minCheck, (laser->ranges)[i]);
            } 

            // ===> Decide how to move 
            if (blocked || stopped || avoidCount > 0){  // Turn away from closest obstacle
                if (avoidCount > 0) {                   // Start turn
                    avoidTurn = (minRight < minLeft) ? -turnRate : turnRate;
                    avoidCount = rand() % avoidTime + avoidTime;
                } else {                                // Keep turning
                    msg.angular.z = avoidTurn; 
                    avoidTime--;
                }
            } else {
                msg.linear.x = moveRate;
            }
            //if (goalDist < goalThreshould){
            //    ROS_INFO("[At Goal!]\n");
            //    atGoal = true;
            //} else {
            //    msg.linear.x = (!blocked && !tooClose(laser)) ? moveRate : 0;
            //    if (goalSeek) {
            //        if (fabs(goalAngle) > angleThreshould)
            //            msg.angular.z = (goalAngle > 0) ? turnRate : turnRate * (-1);
            //        //ROS_INFO("[Seeking Goal] angleDiff:%.2f blocked: %i\n", goalAngle, blocked);
            //        if(blocked || obstaclesInWay(goalAngle, laser))
            //            goalSeek = false;
            //    } else {
            //        msg.angular.z = blocked ? turnRate : 0;
            //        //ROS_INFO("[Wall Follow] theta: %.2f goalAngle: %.2f blocked: %i\n", theta, goalAngle, blocked);
            //        if(!obstaclesInWay(goalAngle, laser))
            //            goalSeek = true;
            //    }
            //}

        } else if (goalDist < goalThreshould && !blocked){
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
    ros::init(argc, argv, "wanderCtrl");
    ros::NodeHandle n;
    ros::Subscriber laserSub = n.subscribe("base_scan", 100, laserCallBack);
    ros::Subscriber odomSub = n.subscribe("base_pose_ground_truth", 100, odomCallBack);
    cmd_velPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::spin();
    return 0;
}
