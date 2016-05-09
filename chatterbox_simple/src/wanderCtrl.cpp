#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"

// ===> Switch Variables
static const float wallThreshould = 0.8;    // Closest distance to stationary obstacle
static const float stopThreshould = 0.2;    // Distance at which something (probably another robot) is considered too close!
static const float turnRate = 1.0;
static const float moveRate = 0.7;
static const int avoidTime = 10;

// ===> Global Variables (w/r robot)
ros::Publisher cmd_velPub;

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser){
    static int avoidCount   = 0;
    static float avoidTurn  = turnRate;
    static int sampleCount  = (laser->ranges).size();
    static int midRayIndex  = sampleCount/2;
    static int raySector    = sampleCount/3;

    geometry_msgs::Twist msg;

    // ===> Evaluate laser scanner data
    float minLeft = 5.0;
    float minRight = 5.0;
    bool blocked     = false;
    bool stopped     = false;

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
    } 
    ROS_INFO("minLeft: %.2f minRight: %.2f blocked: %i stopped: %i\n", minLeft, minRight, blocked, stopped);

    // ===> Decide how to move 
    if (blocked || stopped || avoidCount > 0){  // Turn away from closest obstacle
        if (avoidCount < 1) {                   // Start turn
            avoidTurn = (minRight < minLeft) ? -turnRate : turnRate;
            avoidCount = rand() % avoidTime + avoidTime;
            ROS_INFO("[Start Turn]\n");
        } else {                                // Keep turning
            msg.angular.z = avoidTurn; 
            avoidCount--;
            ROS_INFO("[Keep Turning] rate: %.2f\n", avoidTurn);
        }
    } else {
        msg.linear.x = moveRate;
    }
    cmd_velPub.publish(msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "wanderCtrl");
    ros::NodeHandle n;
    ros::Subscriber laserSub = n.subscribe("scan", 100, laserCallBack);
    cmd_velPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::spin();
    return 0;
}
