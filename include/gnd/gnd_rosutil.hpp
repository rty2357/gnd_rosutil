/*
 * gnd_rosutil.hpp
 *
 *  Created on: 2014/08/04
 *      Author: tyamada
 */

#ifndef GND_ROSUTIL_HPP_
#define GND_ROSUTIL_HPP_


#include <ros/ros.h>
#include <ros/callback_queue.h>

namespace gnd {
	namespace rosutil {
		/**
		 * \brief blocking spinOnce
		 * \param [in] to : timeout
		 * \note it uses in the case that multiple kind of data is required
		 *       and missing data is not allowed
		 * \return 0  : subscribe and call the callback function
		 * \return !0 : timeout
		 */
		void blockingSpinOnce(double to);
	}
}

#endif /* GND_ROSUTIL_HPP_ */
