#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"

static const float nearDist = 1.0; 
static const float turnAngle = 1.0;
static const float moveRate = 1.0;
static const float desErr = 0.3;
static const float angErr = 0.3; 
static const float sink_x = 7;
static const float sink_y = 7;
static const float pi     = 3.1415;
static const float rayCheck = 10;           // How many additional rays to check

ros::Publisher cmd_velPub;
nav_msgs::Odometry::ConstPtr odomVal;
bool avoidObstacle = false;

bool onSink(float xPos, float yPos){
    return (abs(xPos - sink_x) < desErr) && (abs(yPos - sink_y) < desErr);
}

bool nearObject(const sensor_msgs::LaserScan::ConstPtr& laserScan){
    int midRange = ((laserScan -> ranges).size())/2;
    float midVal = (laserScan -> ranges)[midRange]; 
    for(int i = 1; i <= rayCheck/2; i++){   //check additional values for redundancy
        midVal += (laserScan->ranges)[midRange+i] + (laserScan->ranges)[midRange-i]; 
    }
    return nearDist > (midVal)/(rayCheck+1);
}

// 1 : if closer to left wall    0: if closer to right wall 
bool leftWallCloser(const sensor_msgs::LaserScan::ConstPtr& laserScan){
    int scanSize = (laserScan -> ranges).size();
    int leftInd = 3*(scanSize-1)/4;
    int rightInd = (scanSize-1)/4;
    return (laserScan->ranges)[leftInd] < (laserScan->ranges)[rightInd]; 
}

// Which direction to turn --- CW or CCW
// 1 : CC   -1 : CCW
float reorient(const sensor_msgs::LaserScan::ConstPtr& laser){
    int midRange = ((laser -> ranges).size())/2;
    float left = 0;
    float right = 0;
    for (int i = 0; i < midRange; i ++){
        right = (laser -> ranges)[i] > right ? (laser->ranges)[i] : right;
        left = (laser -> ranges)[i+midRange] > left ? (laser->ranges)[i+midRange] : left;
    }
    return left < right ? 1 : -1;            
}

bool blockPath(const sensor_msgs::LaserScan::ConstPtr& laser, float yaw, float optYaw){
    float angleInc = (laser->angle_increment);
    int midRange = ((laser->ranges).size())/2;
    int raysOffCenter = (int)(optYaw-yaw)/angleInc;
    int checkIndex = midRange + raysOffCenter;   
    float checkRay = (laser->ranges)[checkIndex];
    for (int i = 1; i <= rayCheck/2; i++){
        checkRay += (laser->ranges)[checkIndex+i] + (laser->ranges)[checkIndex-i]; 
    }
    checkRay = checkRay/(rayCheck+1);
    ROS_INFO("[RAY] %.2f\n", checkRay);
    return 2.5 > checkRay; 
}

float optAngle(float xx, float yy){
   float delta_x = sink_x - xx;
   float delta_y = sink_y - yy;
   float angleSize = acos(delta_x/sqrt(pow(delta_x,2)+pow(delta_y,2)));
   return (yy < sink_y) ? angleSize : -1 * angleSize;
}

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& laser){
    if(odomVal){ 
        float xx, yy, yaw, optYaw, turnDir;
        int obstacle = 0;
        geometry_msgs::Twist msg;

        xx = (odomVal -> pose).pose.position.x;
        yy = (odomVal -> pose).pose.position.y;
        yaw = tf::getYaw((odomVal -> pose).pose.orientation); 
        optYaw = optAngle(xx, yy);

        if (!onSink(xx,yy)){
            if (nearObject(laser)){                 // Obstacle
                avoidObstacle = true;
                turnDir = reorient(laser);
                msg.angular.z = turnAngle;
                blockPath(laser, yaw, optYaw);
                ROS_INFO("[Avoiding Obstacle] yaw: %.2f, avoid: %i\n", yaw, avoidObstacle);
            } else if ((fabs(yaw-optYaw) > angErr) && !avoidObstacle){  // Not facing sink
                msg.angular.z = turnAngle; 
                ROS_INFO("[Current Location] (x,y): (%.2f, %.2f)\n", xx, yy);
                ROS_INFO("[Adjust Direction] yaw: %.2f opt: %.2f \n", yaw, optYaw); 
            } else {                                // Move robot 
                msg.linear.x = moveRate;
                ROS_INFO("[Moving...]\n");      
                avoidObstacle = blockPath(laser, yaw, optYaw);
                if(!avoidObstacle)
                    ROS_INFO("[Moving towards destination!!!]\n");
            } 
            cmd_velPub.publish(msg);
        }
    }
}

void odomCallBack(const nav_msgs::Odometry::ConstPtr& odom){ 
    odomVal = odom;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "chatterboxAI");
    ros::NodeHandle n;
    ros::Subscriber laserSub = n.subscribe("base_scan", 100, laserCallBack);
    ros::Subscriber odomSub = n.subscribe("base_pose_ground_truth", 100, odomCallBack);
    cmd_velPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::spin();

    return 0;
}
