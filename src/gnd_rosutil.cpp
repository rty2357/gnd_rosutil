/*
 * gnd_rosutil.cpp
 *
 *  Created on: 2014/08/04
 *      Author: tyamada
 */

#include "gnd/gnd_rosutil.hpp"

void gnd::rosutil::blockingSpinOnce(double to) {
	ros::CallbackQueue *query;

	query = ros::getGlobalCallbackQueue();

	query->callAvailable(ros::WallDuration(to));
}
