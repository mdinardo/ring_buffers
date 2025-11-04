#ifndef __LAP_PTR_RB_H_INCLUDED__
#define __LAP_PTR_RB_H_INCLUDED__

// #define TEST_LAP_PTR_RB
// #define DBG_LAP_PTR_RB

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * The buffer size must be a power-of-2; ie. 2^N.
 * Maintain read and write pointers of M>=(1+N) bits in size: rd_ptr[M-1:0] , wr_ptr[M-1:0].
 * valid read and write indexes are 0 <= ptr[N-1:0] < 2^N.
 * Pointer upper bits ptr[M-1:N] count the "laps" for each (mod 2^(M-N)).
 * The write pointer can only ever be ahead by 1 full "lap", ie 2^N,
 * so only ptr[N] matters for lap count comparison.
 * 
 * If the read and write indexes match, ie: wr_ptr[N-1:0] == rd_ptr[N-1:0] , then
 * the buffer is either full (wr_ptr[N]!=rd_ptr[N]) , or empty (wr_ptr[N]==rd_ptr[N])
 *
 * With this scheme, counters for used/free are to be derived from the pointers as needed.
 * The benefit of this scheme is that in some cases a mutex is not required;
 * specifically if there is only one reader and writer separated by threads or main/interrupt contexts.
 *
 * A disadvantage of this scheme is that each context may have a bit of delay in picking up size changes
 * due to pointer updates getting synced to memory.
 */

typedef struct lap_ptr_rb_header {
	volatile size_t _rd_ptr; // Internal use only
	volatile size_t _wr_ptr; // Internal use only
	size_t size; // must be 2^N, with (N+1) <= #bits in the field type.
	uint8_t buffer[];
} lap_ptr_rb_header_t;

typedef struct lap_ptr_rb_state
{
#ifdef DBG_LAP_PTR_RB
	size_t _dbg_rd_ptr; // debug
	size_t _dbg_wr_ptr; // debug
#endif
	size_t rd_idx;
	size_t wr_idx;
	bool full;
	bool empty;
	size_t used_size; // total bytes used
	size_t free_size; // total bytes free
	size_t contiguous_used_size; // number of bytes used starting from current rd_idx without wrap-around.
	size_t contiguous_free_size; // number of bytes free starting from current wr_idx without wrap-around.
} lap_ptr_rb_state_t;

/**
 * Calculate the ring buffer state. full/empty , number of used/free bytes, and
 * number of contiguous used/free bytes without wrap-around. 
 */
__attribute__((nonnull(1,2))) static inline void rb_get_state(volatile lap_ptr_rb_header_t *const rb_in,  lap_ptr_rb_state_t *const restrict state) {
	lap_ptr_rb_header_t rb = *rb_in;

	size_t rd_ptr = rb._rd_ptr;
	size_t wr_ptr = rb._wr_ptr;		
	size_t size   = rb.size;
	size_t idx_mask = (size - 1U);

  #ifdef DBG_LAP_PTR_RB
	// debug
	state->_dbg_rd_ptr = rd_ptr;
	state->_dbg_wr_ptr = wr_ptr;
  #endif // DBG_LAP_PTR_RB

	size_t wr_idx = state->wr_idx = wr_ptr & idx_mask; // wr_ptr[N-1:0]
	size_t rd_idx = state->rd_idx = rd_ptr & idx_mask; // rd_ptr[N-1:0]

  size_t ptr_xor_mask = (size | idx_mask) & (rd_ptr ^ wr_ptr);
	bool full  = state->full  = (size == ptr_xor_mask);
	bool empty = state->empty = (  0U == ptr_xor_mask);

	state->free_size = (empty) ? size : (rd_idx - wr_idx) & idx_mask;  // rely on uint negative wrap-around.
	state->used_size = (full)  ? size : (wr_idx - rd_idx) & idx_mask;

	state->contiguous_free_size = (full)  ? 0U : (wr_idx >= rd_idx) ? (size - wr_idx) : (rd_idx - wr_idx);
	state->contiguous_used_size = (empty) ? 0U : (wr_idx <= rd_idx) ? (size - rd_idx) : (wr_idx - rd_idx);

	//state->_out = state;
}

/**
 * Only use this if free or used >= n have been validated prior to using these functions.
 * After calling them, any previously collected lap_ptr_rb_state_t is invalid.
 */
__attribute__((nonnull(1))) static inline void rb_advance_write_unsafe(rb_header_t *rb, size_t n) {
	rb->_wr_ptr += n;
};
__attribute__((nonnull(1))) static inline void rb_advance_read_unsafe(rb_header_t *rb, size_t n) {
	rb->_rd_ptr += n;
};

inline void rb_get_contiguous_read(void) {};

//////////////


#ifdef TEST_LAP_PTR_RB
#include <stdio.h>
#include <pthread.h>
#include <unistd.h> // for sleep
#define min(_x,_y) (((_x) < (_y) ? (_x) : (_y)))

#define BUFF_SIZE (1U << 14)
struct {
	rb_header_t rb;
	uint8_t buffer[BUFF_SIZE];
} static g_buff = {.rb={.wr_ptr=0, .rd_ptr=0, .size = BUFF_SIZE}};

pthread_mutex_t stop_mutex = PTHREAD_MUTEX_INITIALIZER;
bool stop_flag = false;

void* generator(void*arg) {
	uint8_t wr_counter = 0;
	while (true) {
		pthread_mutex_lock(&stop_mutex);
		if (stop_flag) {
			pthread_mutex_unlock(&stop_mutex);
			printf("Worker: Stop requested. Exiting.\n");
			break;
		}
		pthread_mutex_unlock(&stop_mutex);

		rb_state_t rb_state;
		rb_get_state(&g_buff.rb, &rb_state);

		size_t write_size = BUFF_SIZE >> 2;
		if(rb_state.free_size >= write_size) {
			size_t n = min(write_size, rb_state.contiguous_free_size);
			//n-=200;

			printf("wr_ptr=0x%04x rd_ptr=0x%04x wr_idx=%-5u rd_idx=%-5u full=%-5u empty=%-5u used=%-5u free=%-5u cused=%-5u cfree=%-5u. Writing %u bytes.\n",
			       rb_state._dbg_wr_ptr, rb_state._dbg_rd_ptr,
			       rb_state.wr_idx, rb_state.rd_idx,
			       rb_state.full, rb_state.empty,
			       rb_state.used_size, rb_state.free_size,
			       rb_state.contiguous_used_size, rb_state.contiguous_free_size,
			       n
			      );

			size_t interval = n >> 7;
			int interval_counter = 0;
			for(int i = 0; i < n; i++) {
				g_buff.rb.buffer[i + rb_state.wr_idx] = wr_counter++;

				if(n > 0 && 0 == (++interval_counter)%interval) {
					rb_advance_write_unsafe(&g_buff.rb, interval_counter);
					interval_counter = 0;
				}
				sleep(0.001);
			}

			rb_advance_write_unsafe(&g_buff.rb, interval_counter);

			//rb_advance_write_unsafe(&g_buff.rb, n);
		}
		sleep(0.1);
	}
	return NULL;
}


void* consumer(void*arg) {
	uint8_t wr_counter = 0;
	while (true) {
		pthread_mutex_lock(&stop_mutex);
		if (stop_flag) {
			pthread_mutex_unlock(&stop_mutex);
			printf("Worker: Stop requested. Exiting.\n");
			break;
		}
		pthread_mutex_unlock(&stop_mutex);

		rb_state_t rb_state;
		rb_get_state(&g_buff.rb, &rb_state);

		size_t read_size = BUFF_SIZE >> 6;
		if(rb_state.used_size >= read_size) {
			size_t n = min(read_size, rb_state.contiguous_used_size);

			printf("wr_ptr=0x%04x rd_ptr=0x%04x wr_idx=%-5u rd_idx=%-5u full=%-5u empty=%-5u used=%-5u free=%-5u cused=%-5u cfree=%-5u. Reading %u bytes.\n",
			       rb_state._dbg_wr_ptr, rb_state._dbg_rd_ptr,
			       rb_state.wr_idx, rb_state.rd_idx,
			       rb_state.full, rb_state.empty,
			       rb_state.used_size, rb_state.free_size,
			       rb_state.contiguous_used_size, rb_state.contiguous_free_size,
			       n
			      );

			rb_advance_read_unsafe(&g_buff.rb, n);
		}
		sleep(0.1);
	}
	return NULL;
}



int main(void) {

	printf("hello world\n");
	printf("%zu\n", sizeof(g_buff) - BUFF_SIZE);

	pthread_t generator_thread;
	pthread_create(&generator_thread, NULL, generator, NULL);

	sleep(2);
	pthread_t consumer_thread;
	pthread_create(&consumer_thread, NULL, consumer, NULL);

	// Let the worker run for a bit
	sleep(1);

	// Request the worker to stop
	pthread_mutex_lock(&stop_mutex);
	stop_flag = true;
	pthread_mutex_unlock(&stop_mutex);
	printf("Main: Requested worker to stop.\n");

	pthread_join(generator_thread, NULL);
	pthread_join(consumer_thread, NULL);
	printf("Main: Worker thread joined.\n");

	return 0;
}
#endif // TEST_LAP_PTR_RB

#endif // __LAP_PTR_RB_H_INCLUDED__
