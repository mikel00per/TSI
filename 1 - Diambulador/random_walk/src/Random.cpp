/*
 * Random.cpp
 *
 *  Created on: Oct 27, 2016
 *      Author: viki
 */

#include "Random.h"
#include "geometry_msgs/Twist.h"

Random::Random()
{
    keepMoving = true;

    // Advertise a new publisher for the robot's velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    // Subscribe to the simulated robot's laser scan topic
    laserSub = node.subscribe("scan", 1, &Random::scanCallback, this);

    ANGLE_SPEED = 0.3;
    FORWARD_SPEED = 0.3;
}

// Send a velocity command
void Random::moveForward() {
    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED;
    commandPub.publish(msg);
}

void Random::rotate() {

double rotationSpeed = ANGLE_SPEED;

  if(this->direccion == 0){
    rotationSpeed = rotationSpeed;
  }else if (this->direccion = 1) {
    rotationSpeed = -1 * rotationSpeed;
  }else
    rotationSpeed = rotationSpeed + 3;

  geometry_msgs::Twist msg;
  msg.angular.z = rotationSpeed;
  commandPub.publish(msg);
}

// Process the incoming laser scan message
void Random::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  int posObstaculo = 0;
	bool isObstacleInFront = false;

  // Find the closest range between the defined minimum and maximum angles
  int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
  int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

  int primerTercio = (minIndex+1+maxIndex)/3;
  int segundoTercio = (primerTercio*2)-(primerTercio/2);

  node.getParam("distancia_minima", MIN_DISTANCE_OBSTACLE);

  for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
    if (scan->ranges[currIndex] < MIN_DISTANCE_OBSTACLE) { //Si encuentro un obstaculo almaceno el indice donde ocurre
      ROS_INFO("Obscalo detectado");
  		posObstaculo = currIndex;
  		isObstacleInFront = true;
  		break;
    }
  }

  if (isObstacleInFront) {
    keepMoving = false;
    ROS_INFO("Calculo direccion");

    if(posObstaculo <= primerTercio){
      ROS_INFO("Obstaculo a la derecha");
      direccion = 0;
    }
    else if(posObstaculo <= segundoTercio){
      ROS_INFO("Giro fuerte");
      direccion = 2;
    }
    else{
      ROS_INFO("Obstaculo a la izquierda");
      direccion = 1;
    }
  }else
    keepMoving = true;
}

void Random::startMoving()
{
    ros::Rate rate(10);
    ROS_INFO("Start moving");

    // Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
    while (ros::ok()) {
      if (keepMoving) {
        moveForward();
      }else{
        rotate();
      }
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      rate.sleep();
    }
}
