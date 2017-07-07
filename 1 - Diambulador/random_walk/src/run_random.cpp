/*
 * run_random.cpp
 *
 *  Created on: Oct 27, 2016
 *      Author: viki
 */

#include "Random.h"
#include <iostream>

int main(int argc, char **argv) {
    // Initiate new ROS node named "Random"
    ros::init(argc, argv, "random");



    Random random;

    // Start the movement
    random.startMoving();

    return 0;
};
