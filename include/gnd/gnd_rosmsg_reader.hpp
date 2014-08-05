/*
 * gnd_rosmsg_reader.hpp
 *
 *  Created on: 2014/06/24
 *      Author: tyamada
 */

#ifndef GND_ROSMSG_READER_
#define GND_ROSMSG_READER_

#include <std_msgs/Header.h>

#include "gnd/gnd-lib-error.h"

// ---> type declaration
namespace gnd {
	namespace rosutil {
		template< class M >
		class rosmsgs_reader;
		template< class M >
		class stampedmsgs_reader;
	}
}
// <--- type declaration


// ---> type definition
namespace gnd {
	namespace rosutil {

		template< class M >
		class rosmsgs_reader {
		// type definition
		public:
			typedef M msg_t;
			typedef M const const_msg_t;

		// constructor, destructor
		public:
			rosmsgs_reader() {
				msgs_ = 0;
				length_ = 0;
				header_ = 0;
				ndata_ = 0;
			}
			rosmsgs_reader( long l ) {
				msgs_ = 0;
				length_ = 0;
				header_ = 0;
				ndata_ = 0;
				allocate(l);
			}
			~rosmsgs_reader(){
				deallocate();
			}

		// msg data
		protected:
			// ring buffer
			msg_t *msgs_;
			int length_;
			int header_;
			int ndata_;

		//  allocate, deallocate
		public:
			int is_allocated() {
				return msgs_ != 0 && length_ > 0;
			}

			int allocate( int l ) {
				gnd_assert( is_allocated(), -1, "buffer is already allocated");
				msgs_ = new msg_t[l];
				if( msgs_ ) length_ = l;
				header_ = 0;
				ndata_ = 0;
				return msgs_ != 0;
			}

			int deallocate() {
				gnd_error( !is_allocated(), -1, "buffer is not allocated");
				delete[] msgs_;
				length_ = 0;
				header_ = 0;
				ndata_ = 0;
				return 0;
			}
		// get
		public:
			int ndata() {
				return ndata_;
			}

		// seek
		public:
			int copy_latest( msg_t* m ) {
				{ // ---> assertion
					gnd_assert(!m, -1, "invalid null argument");
					gnd_assert(!is_allocated(), -1, "buffer null");
					gnd_error(ndata_ <= 0, -1, "have no data");
				} // <--- assertion

				*m = msgs_[header_];
				return 0;
			}

		// callback for ROS topic subscribe
		public:
			/**
			 * \brief copy from ROS topic (callback for ROS topic subscription)
			 * \memo usage: MessageReader<foo> foo_reader;
			 *              ros::NodeHandle nodehandle;
			 *              nodehandle.subscribe("topic", 10, &MessageReader<foo>::rosmsg_read, foo_reader.reader_pointer());
			 */
			void rosmsg_read(const boost::shared_ptr< M const>& m ){
				gnd_assert_void( !is_allocated() , "null buffer" );
				header_ = (header_ + 1) % length_;
				msgs_[header_] = *m;
				ndata_ = ndata_ < length_ ? ndata_ + 1 : length_;
			}
			rosmsgs_reader<M>* reader_pointer() {
				return this;
			}
		};




		/**
		 * \brief class to subscribe and store stamped massages
		 */
		template< class M >
		class stampedmsgs_reader :
			public rosmsgs_reader<M>{
		// type definition
		public:
			typedef M msg_t;
			typedef M const const_msg_t;

		// constructor, destructor
		public:
			stampedmsgs_reader() {
			}
			~stampedmsgs_reader(){
			}

		public:
			virtual int copy_at_time( msg_t* dest, double query ) {
				{ // ---> assertion
					gnd_assert(!dest, -1, "invalid null argument");
					gnd_assert(!is_allocated(), -1, "buffer null");
					gnd_error(ndata_ <= 0, -1, "have no data");
				} // <--- assertion

				{ // ---> operation
					double query_sec = query;
					double sec = msgs_[header_].header.stamp.toSec();
					int i, j;

					// ---> seek scan of nearest query time
					if( sec < query_sec || ndata_ <= 1  ) {
						// query time is after last scan timestamp
						i = header_;
					}
					else {
						double diff1, diff2;
						i = header_;
						for( j = 1; j < length_ - 1  && j < ndata_ - 1; j++ ) {
							i = (header_ - j + length_) % length_;
							sec = msgs_[header_].header.stamp.toSec();
							if( sec < query_sec ) {
								break;
							}
						}
						diff1 = query_sec - msgs_[i].header.stamp.toSec();
						i = (header_ - j - 1 + length_) % length_;
						diff2 = msgs_[i].header.stamp.toSec() - query_sec;
						i = diff1 < diff2 ? (header_ - j + length_) % length_ : (header_ - j - 1 + length_) % length_;
					}
					// <--- seek scan of nearest query time

					// copy
					*dest = msgs_[i];
				} // <--- operation

				return 0;
			}

			virtual int copy_at_time( msg_t* dest, ros::Time *query ) {
				{ // ---> assertion
					gnd_assert(!dest, -1, "invalid null argument");
					gnd_assert(!query, -1, "invalid null argument");
					gnd_assert(!is_allocated(), -1, "buffer null");
					gnd_error(ndata_ <= 0, -1, "have no data");
				} // <--- assertion

				return copy_at_time(dest, query->toSec());
			}

			virtual int copy_next( msg_t* dest,  uint32_t query){
				int ret = 0;
				{ // ---> assertion
					gnd_assert(!dest, -1, "invalid null argument");
					gnd_assert(!is_allocated(), -1, "buffer null");
					gnd_error(ndata_ <= 0, -1, "have no data");
				} // <--- assertion

				{ // ---> operation
					int i, j;
					// ---> seek scan of nearest query time
					if( msgs_[header_].header.seq <= query || ndata_ <= 1 ) {
						// query time is after last scan timestamp
						i = header_;
						ret = 1;
					}
					else {
						for( j = 1; j < length_  && j < ndata_ ; j++ ) {
							i = (header_ - j + length_) % length_;
							if( msgs_[i].header.seq <= query ) {
								break;
							}
						}
						i = (header_ - j + 1 + length_) % length_;
					}
					// <--- seek scan of nearest query time

					// copy
					*dest = msgs_[i];
				} // <--- operation

				return ret;
			}

			virtual int copy_next( msg_t* dest, std_msgs::Header *query){
				return this->copy_next(dest, query->seq);
			}

		};


	}
}
// <--- type definition


#endif /* GND_ROSMSG_READER_ */
