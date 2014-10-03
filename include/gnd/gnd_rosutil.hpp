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



// ---> function for ros message sequence id
namespace gnd {
	namespace rosutil {
		typedef uint32_t seq_t;
		const seq_t SEQ_MAX = UINT32_MAX;

		bool is_sequence_overflowed( seq_t seq_from, seq_t seq_to ) {
			return ( (seq_from >= SEQ_MAX / 2)
					&& (seq_to < SEQ_MAX / 2));
		}
		bool is_sequence_underflowed( seq_t seq_from, seq_t seq_to ) {
			return ( (seq_from < SEQ_MAX / 2)
					&& (seq_to >= SEQ_MAX / 2));
		}

		bool is_sequence_updated( seq_t seq_from, seq_t seq_to ) {
			return (seq_to > seq_from) || is_sequence_overflowed(seq_from, seq_to);
		}
	}
}
// <--- function for ros message sequence id

namespace gnd {
	namespace rosutil {


		/**
		 * @brief blocking spinOnce
		 * @param [in] to : timeout
		 * @note it uses in the case that multiple kind of data is required
		 *       and missing data is not allowed
		 * @return 0  : subscribe and call the callback function
		 * @return !0 : timeout
		 */
		void blockingSpinOnce(double to);
	}
}

#endif /* GND_ROSUTIL_HPP_ */
