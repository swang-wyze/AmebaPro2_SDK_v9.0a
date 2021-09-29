/*
 * Copyright (c) 2016, Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_KERNEL_H_
#define ZEPHYR_INCLUDE_KERNEL_H_

#include <zephyr/types.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#include <toolchain/toolchain.h>
#include <linker/sections.h>
#include <sys/__assert.h>
#include <sys/dlist.h>
#include <sys/sflist.h>
#include <sys/slist.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <arch/common/ffs.h>
#include <kernel_structs.h>
#include <stack/stack.h>
//#include <tracing/tracing.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * MEMORY SYSTEM
 */
#if !CONFIG_USE_STATIC_MEM
void k_sys_q_init(void);
void k_sys_q_free(void);
#endif

/******************************************************************************
 * MEMORY
 */
__syscall void *k_malloc(size_t size);
__syscall void  k_free(void *ptr);

/******************************************************************************
 * IRQ
 */
__syscall unsigned int irq_lock(void);
__syscall void irq_unlock(unsigned int key);

/******************************************************************************
 * SEMAPHORE
 */
__syscall int  k_sem_init(struct k_sem *sem, unsigned int initial_count, unsigned int limit);
__syscall int  k_sem_take(struct k_sem *sem, k_timeout_t timeout);
__syscall void k_sem_give(struct k_sem *sem);
__syscall void k_sem_free(struct k_sem *sem);
__syscall unsigned int k_sem_count_get(struct k_sem *sem);
static inline void k_sem_reset(struct k_sem *sem)
{
    /* FIXME: whem some threads is taking sem, maybe wrong */
    k_sem_init(sem, 0, 1);
}

/******************************************************************************
 * MUTEX
 */
__syscall int  k_mutex_init(struct k_mutex *mutex);
__syscall int  k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout);
__syscall int  k_mutex_unlock(struct k_mutex *mutex);
__syscall void k_mutex_free(struct k_mutex *mutex);

/******************************************************************************
 * SPINLOCK
 */
__syscall int k_spin_lock_init(struct k_spinlock *l);
__syscall void k_spin_lock_free(struct k_spinlock *l);
__syscall k_spinlock_key_t k_spin_lock(struct k_spinlock *l);
__syscall void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t key);

/******************************************************************************
 * QUEUE
 */
__syscall void k_queue_init(struct k_queue *queue);
__syscall void k_queue_deinit(struct k_queue *queue);
__syscall void k_queue_cancel_wait(struct k_queue *queue);
__syscall void k_queue_append(struct k_queue *queue, void *data);
__syscall void k_queue_prepend(struct k_queue *queue, void *data);
__syscall int  k_queue_append_list(struct k_queue *queue, void *head, void *tail);
__syscall void *k_queue_get(struct k_queue *queue, k_timeout_t timeout);
__syscall bool k_queue_remove(struct k_queue *queue, void *data);
__syscall int  k_queue_is_empty(struct k_queue *queue);
__syscall void *k_queue_peek_head(struct k_queue *queue);
__syscall void *k_queue_peek_tail(struct k_queue *queue);

/******************************************************************************
 * FIFO QUEUE
 */
#define k_fifo_init(fifo)         k_queue_init(&(fifo)->_queue)
#define k_fifo_deinit(fifo)       k_queue_deinit(&(fifo)->_queue)
#define k_fifo_cancel_wait(fifo)  k_queue_cancel_wait(&(fifo)->_queue)
#define k_fifo_put(fifo, data)    k_queue_append(&(fifo)->_queue, data)
#define k_fifo_put_list(fifo, head, tail) k_queue_append_list(&(fifo)->_queue, head, tail)
#define k_fifo_get(fifo, timeout) k_queue_get(&(fifo)->_queue, timeout)
#define k_fifo_is_empty(fifo)     k_queue_is_empty(&(fifo)->_queue)
#define k_fifo_peek_head(fifo)    k_queue_peek_head(&(fifo)->_queue)
#define k_fifo_peek_tail(fifo)    k_queue_peek_tail(&(fifo)->_queue)

/******************************************************************************
 * LIFO QUEUE
 */
#define k_lifo_init(lifo)         k_queue_init(&(lifo)->_queue)
#define k_lifo_put(lifo, data)    k_queue_prepend(&(lifo)->_queue, data)
#define k_lifo_get(lifo, timeout) k_queue_get(&(lifo)->_queue, timeout)

/******************************************************************************
 * Thread
 */
__syscall k_tid_t k_current_get(void);
__syscall k_tid_t k_thread_create(struct k_thread *new_thread,
				  k_thread_stack_t stack,
				  size_t stack_size,
				  k_thread_entry_t entry,
				  void *p1, void *p2, void *p3,
				  int prio, uint32_t options, k_timeout_t delay);
__syscall void  k_thread_stop(k_tid_t thread, k_thread_exit_t fn, void* ctx);
__syscall void  k_thread_abort(k_tid_t thread);
__syscall void  k_thread_start(k_tid_t thread);
__syscall int   k_thread_name_set(k_tid_t thread_id, const char *value);
__syscall char* k_thread_name_get(k_tid_t thread_id);
__syscall bool  k_is_in_isr(void);

/******************************************************************************
 * Shedule
 */
__syscall void k_sched_lock(void);
__syscall void k_sched_unlock(void);
__syscall int32_t k_sleep(k_timeout_t timeout);
__syscall void k_busy_wait(uint32_t usec_to_wait);
__syscall void k_yield(void);
static inline int32_t k_msleep(int32_t ms)
{
	return k_sleep(K_MSEC(ms));
}

/******************************************************************************
 * MEMORY SLAB
 */
__syscall int k_mem_slab_alloc(struct k_mem_slab *slab, void **mem, k_timeout_t timeout);
__syscall void k_mem_slab_free(struct k_mem_slab *slab, void **mem);
static inline uint32_t k_mem_slab_num_free_get(struct k_mem_slab *slab)
{
	return slab->num_blocks - slab->num_used;
}

/******************************************************************************
 * Time & Timer & Timeout
 */
__syscall int64_t  k_uptime_get(void);
__syscall uint32_t k_uptime_get_32(void);
__syscall int64_t  k_uptime_delta(uint64_t *reftime);
__syscall uint32_t k_cycle_get_32(void);
__syscall uint64_t k_cyc_to_ns_floor64(uint64_t cycle);
__syscall int64_t z_tick_get(void);
__syscall uint64_t z_timeout_end_calc(k_timeout_t timeout);
static inline uint32_t k_ticks_to_ms_floor32(uint32_t t)
{
    return rtw_systime_to_ms(t);
}

/******************************************************************************
 * WORK QUEUE
 */
__syscall void k_work_init(struct k_work *work,
          k_work_handler_t handler);
__syscall int k_work_busy_get(const struct k_work *work);
__syscall int k_work_submit_to_queue(struct k_work_q *queue,
               struct k_work *work);
__syscall int k_work_submit(struct k_work *work);
__syscall int k_work_cancel(struct k_work *work);
__syscall bool k_work_cancel_sync(struct k_work *work, struct k_work_sync *sync);
__syscall void k_work_queue_start(struct k_work_q *queue,
            k_thread_stack_t stack, size_t stack_size,
            int prio, const struct k_work_queue_config *cfg);
__syscall void k_work_queue_stop(struct k_work_q *queue);
__syscall void k_work_init_delayable(struct k_work_delayable *dwork,
               k_work_handler_t handler);
__syscall static inline struct k_work_delayable *
               k_work_delayable_from_work(struct k_work *work);
__syscall int k_work_delayable_busy_get(const struct k_work_delayable *dwork);
__syscall int k_work_schedule_for_queue(struct k_work_q *queue,
                   struct k_work_delayable *dwork,
                   k_timeout_t delay);
__syscall int k_work_schedule(struct k_work_delayable *dwork,
                   k_timeout_t delay);
__syscall int k_work_reschedule_for_queue(struct k_work_q *queue,
                 struct k_work_delayable *dwork,
                 k_timeout_t delay);
__syscall int k_work_reschedule(struct k_work_delayable *dwork,
                     k_timeout_t delay);
__syscall int k_work_cancel_delayable(struct k_work_delayable *dwork);
__syscall bool k_work_cancel_delayable_sync(struct k_work_delayable *dwork,
                  struct k_work_sync *sync);

static inline bool k_work_is_pending(const struct k_work *work)
{
    return k_work_busy_get(work) != 0;
}

static inline struct k_work_delayable *
k_work_delayable_from_work(struct k_work *work)
{
    return CONTAINER_OF(work, struct k_work_delayable, work);
}

static inline bool k_work_delayable_is_pending(
    const struct k_work_delayable *dwork)
{
    return k_work_delayable_busy_get(dwork) != 0;
}

static inline k_ticks_t k_work_delayable_remaining_get(
    const struct k_work_delayable *dwork)
{
    if (!dwork)
        return 0;

    int32_t remain = dwork->timeout.timeout.ms - (rtw_systime_to_ms(rtw_get_current_time()) - dwork->timeout.start_ms);

    return (remain < 0)?(0):(remain);
}


/******************************************************************************
 * POLL (MESSAGE QUEUE)
 */
int k_msg_init(k_msg_queue* msg_q, unsigned int msg_num, unsigned int msg_size);
int z_msg_send(k_msg_queue* msg_q, void *p_msg, k_timeout_t timeout);
static inline int k_msg_send(k_msg_queue* msg_q, void *p_msg, int tag, k_timeout_t timeout)
{
	k_tx_message_t tx_message;
	tx_message.pointer = p_msg;
	tx_message.tag = tag;

    return z_msg_send(msg_q, &tx_message, timeout);
}
int k_msg_recv(k_msg_queue* msg_q, void *p_msg, k_timeout_t timeout);
int k_msg_free(k_msg_queue* msg_q);

/******************************************************************************
 * ERROR HANDLE
 */
void k_oops(void);
void k_panic(void);
// #define k_oops() ASSERT(0)
// #define k_panic() ASSERT(0)

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_KERNEL_H_ */
