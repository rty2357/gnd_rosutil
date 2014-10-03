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
#include "gnd/gnd_rosutil.hpp"

// ---> type declaration
namespace gnd {
	namespace rosutil {
		template< class M >
		class rosmsgs_storage;
		template< class M >
		class rosmsgs_reader;
		template< class M >
		class rosmsgs_reader_stamped;
	}
}
// <--- type declaration


// ---> type definition
namespace gnd {
	namespace rosutil {


		template< class M >
		class rosmsgs_storage {
		public:
			typedef M msg_t;
			typedef M const const_msg_t;

			// constructor, destructor
		public:
			rosmsgs_storage() {
				msgs_ = 0;
				length_ = 0;
				header_ = 0;
				ndata_ = 0;
				n_ = 0;
			}
			rosmsgs_storage( long l ) {
				msgs_ = 0;
				length_ = 0;
				header_ = 0;
				ndata_ = 0;
				n_ = 0;
				allocate(l);
			}
			virtual ~rosmsgs_storage(){
				deallocate();
			}

			// msg data
		protected:
			// ring buffer
			msg_t *msgs_;	///< buffer
			int length_;	///< length of buffer
			int header_;	///< header index of ring buffer(not ros header)
			int ndata_;		///< number of valid data
			uint64_t n_;	///< latest data's sequential number in this reader (not ros header)

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
			/**
			 * \brief get sequential serial number of latest
			 */
			int nlatest() {
				return n_;
			}

			// seek
		public:
			int copy( msg_t* m, int i ) {
				int i_ = (header_ - i + length_) % length_;
				{ // ---> assertion
					gnd_assert(!m, -1, "invalid null argument");
					gnd_assert(!is_allocated(), -1, "buffer null");
					gnd_error(ndata_ <= i, -1, "have no data");
				} // <--- assertion

				*m = msgs_[i_];
				return 0;
			}

			int copy_latest( msg_t* m ) {
				return copy( m, 0 );
			}


		public:
			int push( const_msg_t* m ) {
				gnd_assert( !is_allocated(), -1 , "null buffer" );
				header_ = (header_ + 1) % length_;
				msgs_[header_] = *m;
				ndata_ = ndata_ < length_ ? ndata_ + 1 : length_;
				n_++;
				return 0;
			}

		};


		template< class M >
		class rosmsgs_storage_stamped : virtual public rosmsgs_storage<M> {
		public:
			typedef M msg_t;
			typedef M const const_msg_t;

		public:
			rosmsgs_storage_stamped() : rosmsgs_storage() {
			}
			rosmsgs_storage_stamped(long l) : rosmsgs_storage(l) {
			}
			virtual ~rosmsgs_storage_stamped() {
			}


		public:
			virtual bool is_updated(uint32_t query_seq) {
				return is_sequence_updated(query_seq, msgs_[header_].header.seq);
			}
			virtual bool is_updated(double query_time) {
				return (msgs_[header_].header.stamp.toSec() > query_time);
			}
			virtual bool is_updated(ros::Time *query_time) {
				return is_updated( query_time->toSec() );
			}


		public:
			virtual int copy_at_time( msg_t* dest, double query ) {
				int h = header_;
				{ // ---> assertion
					gnd_assert(!dest, -1, "invalid null argument");
					gnd_assert(!is_allocated(), -1, "buffer null");
					gnd_error(ndata_ <= 0, -1, "have no data");
				} // <--- assertion

				{ // ---> operation
					double query_sec = query;
					double sec = msgs_[h].header.stamp.toSec();
					int i, j, k;

					// ---> seek scan of nearest query time
					if( sec < query_sec || ndata_ <= 1  ) {
						// query time is after last scan timestamp
						i = h;
					}
					else {
						double diff1, diff2;
						i = h;
						for( j = 1; j < length_ - 1  && j < ndata_ - 1; j++ ) {
							i = (h - j + length_) % length_;
							sec = msgs_[i].header.stamp.toSec();
							if( sec < query_sec ) {
								break;
							}
						}
						diff1 = fabs(query_sec - msgs_[i].header.stamp.toSec());
						i = (h - (j - 1) + length_) % length_;
						diff2 = fabs(msgs_[i].header.stamp.toSec() - query_sec);
						i = diff1 < diff2 ? (h - j + length_) % length_ : (h - (j - 1) + length_) % length_;
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

			virtual int copy_new( msg_t* dest,  uint32_t query){
				int ret = 0;
				int h = header_;
				{ // ---> assertion
					gnd_assert(!dest, -1, "invalid null argument");
					gnd_assert(!is_allocated(), -1, "buffer null");
					gnd_error(ndata_ <= 0, -1, "have no data");
				} // <--- assertion

				{ // ---> operation
					if( !is_updated(query) || ndata_ <= 1 ) {
						// query time is after last scan timestamp
						ret = -1;
					}
					else {
						// copy
						*dest = msgs_[h];
						ret = 0;
					}
				} // <--- operation

				return ret;
			}

			virtual int copy_new( msg_t* dest,  double query){
				int ret = 0;
				int h = header_;
				{ // ---> assertion
					gnd_assert(!dest, -1, "invalid null argument");
					gnd_assert(!is_allocated(), -1, "buffer null");
					gnd_error(ndata_ <= 0, -1, "have no data");
				} // <--- assertion

				{ // ---> operation
					double query_sec = query;
					double sec = msgs_[h].header.stamp.toSec();

					// ---> seek scan of nearest query time
					if( sec <= query_sec || ndata_ <= 1  ) {
						// query time is after last scan timestamp
						ret = -1;
					}
					else {
						// copy
						*dest = msgs_[h];
						ret = 0;
					}
				} // <--- operation

				return ret;
			}

			virtual int copy_new( msg_t* dest, ros::Time *query ) {
				{ // ---> assertion
					gnd_assert(!dest, -1, "invalid null argument");
					gnd_assert(!query, -1, "invalid null argument");
					gnd_assert(!is_allocated(), -1, "buffer null");
					gnd_error(ndata_ <= 0, -1, "have no data");
				} // <--- assertion

				return copy_new(dest, query->toSec());
			}

			virtual int copy_next( msg_t* dest,  uint32_t query){
				int ret = 0;
				int h = header_;
				{ // ---> assertion
					gnd_assert(!dest, -1, "invalid null argument");
					gnd_assert(!is_allocated(), -1, "buffer null");
					gnd_error(ndata_ <= 0, -1, "have no data");
				} // <--- assertion

				{ // ---> operation
					int i, j;
					// ---> seek scan of nearest query time
					if( !is_updated(query) || ndata_ <= 1 ) {
						// query time is after last scan timestamp
						ret = -1;
					}
					else {
						for( j = 1; j < length_ - 1 && j < ndata_ ; j++ ) {
							i = (h - j + length_) % length_;
							if( msgs_[i].header.seq <= query || is_sequence_overflowed(query, msgs_[i].header.seq) ) {
								break;
							}
						}

						if ( msgs_[i].header.seq > query ) {
							i = (h - j + length_) % length_;
						}
						else {
							i = (h - (j - 1) + length_) % length_;
						}
						// copy
						*dest = msgs_[i];
					}
					// <--- seek scan of nearest query time

				} // <--- operation

				return ret;
			}

			virtual int copy_next( msg_t* dest, std_msgs::Header *query){
				return this->copy_next(dest, query->seq);
			}

			virtual int copy_prev( msg_t* dest,  uint32_t query){
				int ret = 0;
				int h = header_;
				{ // ---> assertion
					gnd_assert(!dest, -1, "invalid null argument");
					gnd_assert(!is_allocated(), -1, "buffer null");
					gnd_error(ndata_ <= 0, -1, "have no data");
				} // <--- assertion

				{ // ---> operation
					int i, j;
					// ---> seek scan of nearest query time
					if( ndata_ <= 1 ) {
						ret = -1;
					}
					else {
						for( j = 0; j < length_ - 2 && j < ndata_ - 1 ; j++ ) {
							i = (h - j + length_) % length_;
							if( msgs_[i].header.seq <= query  || is_sequence_underflowed(query, msgs_[i].header.seq) ) {
								break;
							}
						}

						if ( msgs_[i].header.seq > query ) {
							return -1;
						}
						else if ( msgs_[i].header.seq == query ) {
							i = (h - (j + 1) + length_) % length_;
						}
						else {
							i = (h - j + length_) % length_;
						}
						// copy
						*dest = msgs_[i];
					} // <--- seek scan of nearest query time

				} // <--- operation

				return ret;
			}

			virtual int copy_prev( msg_t* dest, std_msgs::Header *query){
				return this->copy_prev(dest, query->seq);
			}

		};


		template< class M >
		class rosmsgs_reader :
				virtual public rosmsgs_storage<M> {
			// type definition

			// callback for ROS topic subscribe
				public:
			/**
			 * \brief copy from ROS topic (callback for ROS topic subscription)
			 * \memo usage: rosmsg_reader<foo> foo_reader;
			 *              ros::NodeHandle nodehandle;
			 *              nodehandle.subscribe("topic", 10, &rosmsg_reader<foo>::rosmsg_read, foo_reader.reader_pointer());
			 */
			void rosmsg_read(const boost::shared_ptr< M const>& m ){
				push( boost::get_pointer(m) );
			}
			rosmsgs_reader<M>* reader_pointer() {
				return this;
			}
		};




		/**
		 * \brief class to subscribe and store stamped massages
		 */
		template< class M >
		class rosmsgs_reader_stamped :
				public rosmsgs_storage_stamped<M>, public rosmsgs_reader<M> {
			// type definition
				public:
			typedef M msg_t;
			typedef M const const_msg_t;

			// constructor, destructor
				public:
			rosmsgs_reader_stamped() {
			}
			virtual ~rosmsgs_reader_stamped(){
			}
		};


	}
}
// <--- type definition


#endif /* GND_ROSMSG_READER_ */
