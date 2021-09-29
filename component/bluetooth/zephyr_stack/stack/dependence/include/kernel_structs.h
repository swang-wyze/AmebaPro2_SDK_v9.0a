/*
 * Copyright (c) 2016 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_KERNEL_INCLUDE_KERNEL_STRUCTS_H_
#define ZEPHYR_KERNEL_INCLUDE_KERNEL_STRUCTS_H_

#include <osdep_service.h>
/* for compile */
//#undef u8
#undef s8
#undef s16
#undef u16
#undef s32
#undef u32
#undef s64
#undef u64
#undef i8
#undef i16
#undef i32
#ifndef atomic_t
#define atomic_t k_atomic_t
#endif
// #ifndef cli
// #define cli mesh_cfg_cli 
// #endif
/* for compile */

#include <sys/atomic.h>
#include <zephyr/types.h>
#include <sys/util.h>
//#include <sys/sflist.h>

#ifdef __cplusplus
extern "C" {
#endif

struct k_thread;
struct k_mutex;
struct k_sem;
struct k_queue;
struct k_fifo;
struct k_lifo;
struct k_mem_slab;
struct k_work;
struct k_work_q;
struct k_delayed_work;

/******************************************************************************
 * Due to we cant adapter to osdep_service.h, have to define some unique VARs
 */
#if CONFIG_KERNEL_RTOS_FREERTOS
#undef atomic_read
#undef atomic_set

#define OSDEP_PRIO_COOP(x)            (x)
#define OSDEP_PRIO_PREEMPT(x)         (x)

#define OSDEP_MAX_UINT32              0xFFFFFFFF
#define OSDEP_TIMER_MAX_DELAY         OSDEP_MAX_UINT32

#define OSDEP_IS_HANDLE_IN_THREAD(pthread, hdl) \
    (pthread->task_t.task == hdl)

#if CONFIG_USE_STATIC_MEM
/* FREERTOS 10.2.1 */
#define STATIC_SEMA_SIZE              80 //sizeof(StaticSemaphore_t)
#define STATIC_MUTEX_SIZE             80 //sizeof(StaticSemaphore_t)
#define STATIC_SPINLOCK_SIZE          80 //sizeof(StaticSemaphore_t)
#define STATIC_TIMER_SIZE             44 //sizeof(StaticTimer_t)

typedef char z_sem[STATIC_SEMA_SIZE];
typedef char z_mutex[STATIC_MUTEX_SIZE];
typedef char z_spinlock[STATIC_SPINLOCK_SIZE];
typedef char z_timer[STATIC_TIMER_SIZE];

#define OSDEP_SEM_INITIALIZER         {0}
#define OSDEP_MUTEX_INITIALIZER       {0}
#define OSDEP_SPINLOCK_INITIALIZER    {0}
#define OSDEP_TIMER_INITIALIZER       {0}
#else
typedef _sema z_sem;
typedef _mutex z_mutex;
typedef _lock z_spinlock;
typedef _timerHandle z_timer;

#define OSDEP_SEM_INITIALIZER         NULL
#define OSDEP_MUTEX_INITIALIZER       NULL
#define OSDEP_SPINLOCK_INITIALIZER    NULL
#define OSDEP_TIMER_INITIALIZER       NULL
#endif

#endif

/******************************************************************************
 * MEMORY SYSTEM
 */

/******************************************************************************
 * MEMORY
 */

/******************************************************************************
 * IRQ
 */

/******************************************************************************
 * SEMAPHORE
 */
struct k_sem
{
	void *_reserved;
	z_sem sema;
	uint32_t init_count;
	uint32_t limit;
    int inited;
};

#define Z_SEM_INITIALIZER(obj, initial_count, count_limit) \
	{                                                      \
		._reserved = NULL,                                 \
		.sema = OSDEP_SEM_INITIALIZER,                     \
		.init_count = initial_count,                       \
		.limit = count_limit,                              \
		.inited = 0,                                       \
	}

#define K_SEM_DEFINE(name, initial_count, count_limit)       \
	Z_STRUCT_SECTION_ITERABLE(k_sem, name) =                 \
		Z_SEM_INITIALIZER(name, initial_count, count_limit); \
	BUILD_ASSERT(((count_limit) != 0) &&                     \
				 ((initial_count) <= (count_limit)));

/******************************************************************************
 * MUTEX
 */
struct k_mutex
{
	void *_reserved;
	z_mutex mutex;
    int inited;
};

#define Z_MUTEX_INITIALIZER(obj)          \
	{                                     \
		._reserved = NULL,                \
		.mutex = OSDEP_MUTEX_INITIALIZER, \
		.inited = 0,                      \
	}

#define K_MUTEX_DEFINE(name) \
	Z_STRUCT_SECTION_ITERABLE(k_mutex, name) = \
		Z_MUTEX_INITIALIZER(name)

/******************************************************************************
 * SPINLOCK
 */
struct z_spinlock_key
{
	int key;
};

typedef struct z_spinlock_key k_spinlock_key_t;

struct k_spinlock
{
	void *_reserved;
	z_spinlock lock;
    int inited;
};

#define Z_SPINLOCK_INITIALIZER(obj)         \
	{                                       \
		._reserved = NULL,                  \
		.lock = OSDEP_SPINLOCK_INITIALIZER, \
		.inited = 0,                        \
	}

/******************************************************************************
 * QUEUE
 */
struct k_queue
{
	sys_sflist_t data_q;
	struct k_spinlock lock;
    //atomic_t is_pending;
	struct k_sem wait_sema;
};

#define Z_QUEUE_INITIALIZER(obj)                                \
	{                                                           \
		.data_q      = SYS_SFLIST_STATIC_INIT(&obj.data_q),     \
		.lock        = Z_SPINLOCK_INITIALIZER(&obj.lock),       \
		.wait_sema   = Z_SEM_INITIALIZER(&obj.wait_sema, 0, 0xFFFFFFFFUL), \
	}

/******************************************************************************
 * FIFO QUEUE
 */
struct k_fifo
{
	struct k_queue _queue;
};

#define Z_FIFO_INITIALIZER(obj)                   \
	{                                             \
		._queue = Z_QUEUE_INITIALIZER(obj._queue) \
	}

#define K_FIFO_DEFINE(name)                                      \
	Z_STRUCT_SECTION_ITERABLE_ALTERNATE(k_queue, k_fifo, name) = \
		Z_FIFO_INITIALIZER(name)

/******************************************************************************
 * LIFO QUEUE
 */
struct k_lifo
{
	struct k_queue _queue;
};

#define Z_LIFO_INITIALIZER(obj)                   \
	{                                             \
		._queue = Z_QUEUE_INITIALIZER(obj._queue) \
	}

/******************************************************************************
 * Thread
 */
#define K_PRIO_COOP(x)                   OSDEP_PRIO_COOP(x)
#define K_PRIO_PREEMPT(x)                OSDEP_PRIO_PREEMPT(x)

#define K_KERNEL_STACK_SIZEOF(sym)       (sym)
#define K_KERNEL_STACK_DEFINE(sym, size) \
    size_t sym = size
#define K_KERNEL_STACK_MEMBER(sym, size) \
    size_t sym
#define K_RFCONN_DLC_STACK_SZIE 256

struct k_thread
{
	struct task_struct task_t;
	struct k_sem run_sema;
	char run;
};
#define IS_HANDLE_IN_THREAD(pthread, hdl) OSDEP_IS_HANDLE_IN_THREAD(pthread, hdl)

typedef size_t k_thread_stack_t;
typedef struct k_thread *k_tid_t;
typedef void (*k_thread_entry_t)(void *p1, void *p2, void *p3);
typedef void (*k_thread_exit_t)(void *p1);

/******************************************************************************
 * Shedule
 */

/******************************************************************************
 * MEMORY SLAB
 */
struct k_mem_slab
{
	uint32_t num_blocks;
	size_t block_size;
	uint32_t num_used;
};

#define Z_MEM_SLAB_INITIALIZER(obj, slab_buffer, slab_block_size, slab_num_blocks) \
	{                                                                              \
		.num_blocks = slab_num_blocks,                                             \
		.block_size = slab_block_size,                                             \
		.num_used = 0,                                                             \
	}

#define K_MEM_SLAB_DEFINE(name, slab_block_size, slab_num_blocks, slab_align) \
	Z_STRUCT_SECTION_ITERABLE(k_mem_slab, name) =                             \
		Z_MEM_SLAB_INITIALIZER(name, _k_mem_slab_buf_##name,                  \
							   WB_UP(slab_block_size), slab_num_blocks)

/******************************************************************************
 * Time & Timer & Timeout
 */

#ifdef CONFIG_TIMEOUT_64BIT
typedef int64_t k_ticks_t;
#else
typedef uint32_t k_ticks_t;
#endif

typedef uint32_t k_time_t;
typedef k_time_t k_ms_t;
typedef struct {
	k_ms_t ms;
} k_timeout_t;
#define NSEC_PER_USEC      1000U
#define USEC_PER_MSEC      1000U
#define MSEC_PER_SEC       1000U

#define Z_TIMEOUT_TICKS(t) ((k_timeout_t) { .ms = rtw_systime_to_ms(t) })

#define K_TIMEOUT_EQ(a, b) ((a).ms == (b).ms)
#define K_MSEC(t)         ((k_timeout_t) { .ms = t })
#define K_SECONDS(s)       K_MSEC((s)*MSEC_PER_SEC)
#define K_MINUTES(m)       K_SECONDS((m)*60)
#define K_HOURS(h)         K_MINUTES((h)*60)
#define K_NO_WAIT          ((k_timeout_t) { .ms = 0 })
#define K_FOREVER          ((k_timeout_t) { .ms = ((k_ms_t) - 1) })

#define Z_TIMER_MAX_DELAY  OSDEP_TIMER_MAX_DELAY
#define K_SEM_MAX_LIMIT    OSDEP_MAX_UINT32

#define SYS_FOREVER_MS     (-1)
#define SYS_TIMEOUT_MS(ms) ((ms) == SYS_FOREVER_MS ? K_FOREVER : K_MSEC(ms))

struct _timeout
{
	void *_reserved;
	void *timerID;
    z_timer timerhandle;
	uint32_t start_ms;   /* Record start time */
	k_timeout_t timeout; /* Record timeout time */
    int inited;
};
typedef void (*_timeout_func_t)(void* t);

/******************************************************************************
 * WORK QUEUE
 */
enum {
/**
 * @cond INTERNAL_HIDDEN
 */

	/* The atomic API is used for all work and queue flags fields to
	 * enforce sequential consistency in SMP environments.
	 */

	/* Bits that represent the work item states.  At least nine of the
	 * combinations are distinct valid stable states.
	 */
	K_WORK_RUNNING_BIT = 0,
	K_WORK_CANCELING_BIT = 1,
	K_WORK_QUEUED_BIT = 2,
	K_WORK_DELAYED_BIT = 3,

	K_WORK_MASK = BIT(K_WORK_DELAYED_BIT) | BIT(K_WORK_QUEUED_BIT)
		| BIT(K_WORK_RUNNING_BIT) | BIT(K_WORK_CANCELING_BIT),

	/* Static work flags */
	K_WORK_DELAYABLE_BIT = 8,
	K_WORK_DELAYABLE = BIT(K_WORK_DELAYABLE_BIT),

	/* Dynamic work queue flags */
	K_WORK_QUEUE_STARTED_BIT = 0,
	K_WORK_QUEUE_STARTED = BIT(K_WORK_QUEUE_STARTED_BIT),
	K_WORK_QUEUE_BUSY_BIT = 1,
	K_WORK_QUEUE_BUSY = BIT(K_WORK_QUEUE_BUSY_BIT),
	K_WORK_QUEUE_DRAIN_BIT = 2,
	K_WORK_QUEUE_DRAIN = BIT(K_WORK_QUEUE_DRAIN_BIT),
	K_WORK_QUEUE_PLUGGED_BIT = 3,
	K_WORK_QUEUE_PLUGGED = BIT(K_WORK_QUEUE_PLUGGED_BIT),

	/* Static work queue flags */
	K_WORK_QUEUE_NO_YIELD_BIT = 8,
	K_WORK_QUEUE_NO_YIELD = BIT(K_WORK_QUEUE_NO_YIELD_BIT),

/**
 * INTERNAL_HIDDEN @endcond
 */
	/* Transient work flags */

	/** @brief Flag indicating a work item that is running under a work
	 * queue thread.
	 *
	 * Accessed via k_work_busy_get().  May co-occur with other flags.
	 */
	K_WORK_RUNNING = BIT(K_WORK_RUNNING_BIT),

	/** @brief Flag indicating a work item that is being canceled.
	 *
	 * Accessed via k_work_busy_get().  May co-occur with other flags.
	 */
	K_WORK_CANCELING = BIT(K_WORK_CANCELING_BIT),

	/** @brief Flag indicating a work item that has been submitted to a
	 * queue but has not started running.
	 *
	 * Accessed via k_work_busy_get().  May co-occur with other flags.
	 */
	K_WORK_QUEUED = BIT(K_WORK_QUEUED_BIT),

	/** @brief Flag indicating a delayed work item that is scheduled for
	 * submission to a queue.
	 *
	 * Accessed via k_work_busy_get().  May co-occur with other flags.
	 */
	K_WORK_DELAYED = BIT(K_WORK_DELAYED_BIT),
};

typedef void (*k_work_handler_t)(struct k_work *work);

struct k_work {
	sys_snode_t node;
	k_work_handler_t handler;
	struct k_work_q* queue; /* The queue on which last submitted. */
	uint32_t flags;
};

#define Z_WORK_INITIALIZER(work_handler) { \
    .node = {NULL}, \
	.handler = work_handler, \
	.queue = NULL, \
	.flags = 0, \
}

#define K_WORK_DEFINE(work, work_handler) \
	struct k_work work = Z_WORK_INITIALIZER(work_handler)

struct k_work_delayable {
	struct k_work work;
	struct _timeout timeout;
	struct k_work_q* queue;  /* The queue should be submitted. */
};

#define Z_WORK_DELAYABLE_INITIALIZER(work_handler) { \
	.work = Z_WORK_INITIALIZER(work_handler), \
    .timeout = { \
        ._reserved = NULL, \
        .timerhandle = OSDEP_TIMER_INITIALIZER, \
        .timerID = NULL, \
        .start_ms = 0, \
        .timeout = { .ms = 0 }, \
        .inited = 0, \
    }, \
    .queue = NULL, \
}

#define K_WORK_DELAYABLE_DEFINE(work, work_handler) \
	struct k_work_delayable work \
	  = Z_WORK_DELAYABLE_INITIALIZER(work_handler)

struct z_work_canceller {
	sys_snode_t node;
	struct k_work *work;
	struct k_sem sem;
};

struct k_work_sync {
	union {
		//struct z_work_flusher flusher;
		struct z_work_canceller canceller;
	};
};

struct k_work_queue_config {
	const char *name;
	bool no_yield;
};

struct k_work_q {
	struct k_thread thread;
	sys_slist_t pending;
	struct k_sem notifyq;
	//_wait_q_t drainq;
	uint32_t flags;
};

extern struct k_work_q k_sys_work_q;

/******************************************************************************
 * POLL (MESSAGE QUEUE)
 */
typedef struct {
    void *pointer;
    int tag;
} k_tx_message_t;
typedef _xqueue k_msg_queue;

/******************************************************************************
 * ERROR HANDLE
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_KERNEL_INCLUDE_KERNEL_STRUCTS_H_ */
