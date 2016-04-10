/*
 * rosbert_map.hpp
 *
 *  Created on: Apr 9, 2016
 *      Author: christopher
 */

#ifndef ROSBERT_MAP_HPP
#define ROSBERT_MAP_HPP

#include "ros/ros.h"

class RosbertMap
{
public:
	RosbertMap();

private:
	ros::Subscriber mapSub;
	ros::Subscriber startSub;
	ros::Subscriber goalSub;

	ros::Publisher mapCheck;

	void mapCallback();

	void publishFrontier();




};



#endif /* A_STAR_HPP */
