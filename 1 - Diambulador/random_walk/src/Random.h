/*
 * Random.h
 *
 *  Created on: Oct 27, 2016
 *      Author: viki
 */

#ifndef WANDER_BOT_SRC_Random_H_
#define WANDER_BOT_SRC_Random_H_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Random {
public:
    // Tunable parameters
    double FORWARD_SPEED;
    double ANGLE_SPEED;
    double MIN_DISTANCE_OBSTACLE;

    const static double MIN_SCAN_ANGLE = -30.0/180*M_PI;
    const static double MAX_SCAN_ANGLE = +30.0/180*M_PI;

    Random();
    void startMoving();
    void rotate(); //Donde realizamos los giros

private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
    bool keepMoving; // Indicates whether the robot should continue moving
    int direccion;
    void moveForward();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif /* WANDER_BOT_SRC_Random_H_ */
