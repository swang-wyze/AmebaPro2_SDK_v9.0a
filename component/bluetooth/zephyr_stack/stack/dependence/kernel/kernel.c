/*
 * Copyright (c) 2021, Realsil Semiconductor Corporation. All rights reserved.
 */

#include <kernel.h>

/******************************************************************************
 * OS INTF DEBUG OUTPUT
 */
#ifdef CONFIG_OS_INTF_DEBUG
#define BT_DBG_OS(fmt, ...) \
    printk("\n\r(os_dbg) %s:%d "fmt"\n", __func__, __LINE__, ##__VA_ARGS__)
#else
#define BT_DBG_OS(fmt, ...)
#endif

/******************************************************************************
 * MEMORY SYSTEM
 */
#if !CONFIG_USE_STATIC_MEM
enum k_sys_q_idx
{
    K_SEM_Q,
    K_MUTEX_Q,
    K_SPINLOCK_Q,
    K_TIMER_Q,
};

/* Strictly order by k_sys_q_idx */
static sys_sflist_t k_sys_q[4] = {0};

void k_sys_q_init(void)
{
	int flag = irq_lock();
    for (int idx = K_SEM_Q; idx < K_TIMER_Q; idx++)
	    sys_sflist_init(&k_sys_q[idx]);
	irq_unlock(flag);
}

static void z_sys_q_free_item(void* data, int idx)
{
    struct k_sem*      sem_item;
    struct k_mutex *   mutex_item;
    struct k_spinlock* spinlock_item;
    struct _timeout *  timeout_item;

    switch (idx)
    {
    case K_SEM_Q:
        sem_item = (struct k_sem*)data;
        if (sem_item && sem_item->sema)
        {
            rtw_free_sema(&sem_item->sema);
            sem_item->sema = OSDEP_SEM_INITIALIZER;
            sem_item->inited = 0;
        }
        break;
    case K_MUTEX_Q:
        mutex_item = (struct k_mutex *)data;
        if (mutex_item && mutex_item->mutex)
        {
            rtw_mutex_free(&mutex_item->mutex);
            mutex_item->mutex = OSDEP_MUTEX_INITIALIZER;
            mutex_item->inited = 0;
        }
        break;
    case K_SPINLOCK_Q:
        spinlock_item = (struct k_spinlock*)data;
        if (spinlock_item && spinlock_item->lock)
        {
            rtw_spinlock_free(&spinlock_item->lock);
            spinlock_item->lock = OSDEP_SPINLOCK_INITIALIZER;
            spinlock_item->inited = 0;
        }
        break;
    case K_TIMER_Q:
        timeout_item = (struct _timeout *)data;
        if (timeout_item && timeout_item->timerhandle)
        {
            rtw_timerDelete(timeout_item->timerhandle, OSDEP_TIMER_MAX_DELAY);
            timeout_item->timerhandle = NULL;
            timeout_item->inited = 0;
        }
        break;
    default:
        break;
    }
}

void k_sys_q_free(void)
{
    struct k_sem*      sem_item;
    struct k_mutex *   mutex_item;
    struct k_spinlock* spinlock_item;
    struct _timeout *  timeout_item;

    int flag = irq_lock();
    for (int idx = K_SEM_Q; idx < K_TIMER_Q; idx++)
    {
        do
        {
            if (sys_sflist_is_empty(&k_sys_q[idx]))
                break;

            z_sys_q_free_item((void*)sys_sflist_get_not_empty(&k_sys_q[idx]), idx);
        } while (1);
    }
    irq_unlock(flag);
}

static void k_sys_q_insert(void *data, int idx)
{
    int flag = irq_lock();
    sys_sfnode_init(data, 0x0);
    sys_sflist_insert(&k_sys_q[idx], NULL, data);
    irq_unlock(flag);
}

static bool k_sys_q_find_and_remove(void *data, int idx, bool remove)
{
    uint32_t flag = irq_lock();
    if (remove)
        bool ret = sys_sflist_find_and_remove(&k_sys_q[idx], data);
    else
        bool ret = sys_sflist_find(&k_sys_q[idx], data);
    irq_unlock(flag);
    return ret;
}
#endif

/******************************************************************************
 * MEMORY
 */
void *k_malloc(size_t size)
{
    /* align size controled by os config */
    return rtw_malloc((uint32_t)size);
}

void k_free(void *ptr)
{
    rtw_mfree(ptr, 0);
}

/******************************************************************************
 * IRQ
 */
unsigned int irq_lock(void)
{
    /* XXX: Only in Freertos! */
    rtw_enter_critical(NULL, NULL);
    return 0;
}

void irq_unlock(unsigned int key)
{
    ARG_UNUSED(key);

    /* XXX: Only in Freertos! */
    rtw_exit_critical(NULL, NULL);
}

/******************************************************************************
 * SEMAPHORE
 */
int k_sem_init(struct k_sem *sem, unsigned int initial_count, unsigned int limit)
{
    __ASSERT(sem, "sem is null\n");

	sem->init_count = initial_count;
	sem->limit = limit;

#if CONFIG_USE_STATIC_MEM
	memset(sem->sema, 0, sizeof(sem->sema));
	void* psema = sem->sema;
    if (rtw_sema_init(&psema, sem->init_count, sem->limit, 1))
        return -EINVAL;
#else
    if (true == k_sys_q_find_and_remove(sem, K_SEM_Q, 1))
        rtw_free_sema(&sem->sema);
    sem->sema = NULL;
    if (rtw_sema_init(&sem->sema, sem->init_count, sem->limit, 0))
        return -EINVAL;
    k_sys_q_insert(sem, K_SEM_Q);
#endif
    sem->inited = 1;

    return 0;
}

void k_sem_give(struct k_sem *sem)
{
    __ASSERT(sem, "sem is null\n");

#if CONFIG_USE_STATIC_MEM
    if (!sem->inited)
        if (k_sem_init(sem, sem->init_count, sem->limit))
            return;
    void* psema = sem->sema;
    rtw_up_sema(&psema);
#else
    if (false == k_sys_q_find_and_remove(sem, K_SEM_Q, 0))
    {
        if (k_sem_init(sem, sem->init_count, sem->limit))
            return;
        k_sys_q_insert(sem, K_SEM_Q);
    }
    rtw_up_sema(&sem->sema);
#endif

}

int k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
    __ASSERT(sem, "sem is null\n");

#if CONFIG_USE_STATIC_MEM
    if (!sem->inited)
        if (k_sem_init(sem, sem->init_count, sem->limit))
            return -EINVAL;
    void* psema = sem->sema;
    if (rtw_down_timeout_sema(&psema, timeout.ms) != _TRUE)
        return -EINVAL;
#else
    if (false == k_sys_q_find_and_remove(sem, K_SEM_Q, 0))
    {
        if (k_sem_init(sem, sem->init_count, sem->limit))
            return -EINVAL;
        k_sys_q_insert(sem, K_SEM_Q);
    }
    if (rtw_down_timeout_sema(&sem->sema, timeout.ms) != _TRUE)
        return -EINVAL;
#endif

    return 0;
}

void k_sem_free(struct k_sem *sem)
{
    __ASSERT(sem, "sem is null\n");

#if CONFIG_USE_STATIC_MEM
#else
    if (true == k_sys_q_find_and_remove(sem, K_SEM_Q, 1))
        rtw_free_sema(&sem->sema);
#endif
    sem->inited = 0;
}

unsigned int k_sem_count_get(struct k_sem *sem)
{
    __ASSERT(sem, "sem is null\n");

    return rtw_sema_get_count(sem->sema);
}

/******************************************************************************
 * MUTEX
 */
int k_mutex_init(struct k_mutex *mutex)
{
    __ASSERT(mutex, "mutex is null\n");

#if CONFIG_USE_STATIC_MEM
    memset(mutex->mutex, 0, sizeof(mutex->mutex));
    void* pmutex = mutex->mutex;
    if (rtw_mutex_init_static(&pmutex))
        return -EINVAL;
#else
    /* Only not in queue, can we create mutex */
    if (true == k_sys_q_find_and_remove(mutex, K_MUTEX_Q, 1))
        rtw_mutex_free(&mutex->mutex);
    rtw_mutex_init(&mutex->mutex);
    k_sys_q_insert(mutex, K_MUTEX_Q);
#endif

    mutex->inited = 1;
    return 0;
}

int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
    __ASSERT(mutex, "mutex is null\n");

#if CONFIG_USE_STATIC_MEM
    if (!mutex->inited)
        if (k_mutex_init(mutex))
            return -EINVAL;
    void* pmutex = mutex->mutex;
    if (rtw_mutex_get_timeout(&pmutex, timeout.ms) != 0)
        return -EINVAL;
#else
    if (false == k_sys_q_find_and_remove(mutex, K_MUTEX_Q, 0))
    {
        if (k_mutex_init(mutex))
            return -EINVAL;
        k_sys_q_insert(mutex, K_MUTEX_Q);
    }
    if (rtw_mutex_get_timeout(&mutex->mutex, timeout.ms) != 0)
        return -EINVAL;
#endif

    return 0;
}

int k_mutex_unlock(struct k_mutex *mutex)
{
    __ASSERT(mutex, "mutex is null\n");

#if CONFIG_USE_STATIC_MEM
    if (!mutex->inited)
        if (k_mutex_init(mutex))
            return -EINVAL;
    void* pmutex = mutex->mutex;
    rtw_mutex_put(&pmutex);
#else
    if (false == k_sys_q_find_and_remove(mutex, K_MUTEX_Q, 0))
    {
        if (k_mutex_init(mutex))
            return -EINVAL;
        k_sys_q_insert(mutex, K_MUTEX_Q);
    }
    rtw_mutex_put(&mutex->mutex);
#endif
    return 0;
}

void k_mutex_free(struct k_mutex *mutex)
{
    __ASSERT(mutex, "mutex is null\n");

#if CONFIG_USE_STATIC_MEM
#else
    if (true == k_sys_q_find_and_remove(mutex, K_MUTEX_Q, 1))
        rtw_mutex_free(&mutex->mutex);
#endif

    mutex->inited = 0;
}

/******************************************************************************
 * SPINLOCK
 */
int k_spin_lock_init(struct k_spinlock *l)
{
    __ASSERT(l, "spinlock is null\n");

#if CONFIG_USE_STATIC_MEM
    memset(l->lock, 0, sizeof(l->lock));
    void* pspinlock = l->lock;
    if (rtw_spinlock_init_static(&pspinlock))
        return -EINVAL;
#else
    if (true == k_sys_q_find_and_remove(l, K_SPINLOCK_Q, 1))
        rtw_spinlock_free(&l->lock);
    rtw_spinlock_init(&l->lock);
    k_sys_q_insert(l, K_SPINLOCK_Q);
#endif

    l->inited = 1;
    return 0;
}

k_spinlock_key_t k_spin_lock(struct k_spinlock *l)
{
    __ASSERT(l, "spinlock is null\n");
    k_spinlock_key_t spinlock_key;
    spinlock_key.key = 1;

#if CONFIG_USE_STATIC_MEM
    if (!l->inited)
        if (k_spin_lock_init(l))
            spinlock_key.key = 0;
    void* pspinlock = l->lock;
    rtw_spin_lock(&pspinlock);
#else
    if (false == k_sys_q_find_and_remove(l, K_SPINLOCK_Q, 0))
    {
        if (k_spin_lock_init(l))
            spinlock_key.key = 0;
        k_sys_q_insert(l, K_SPINLOCK_Q);
    }
    rtw_spin_lock(&l->lock);
#endif

    return spinlock_key;
}

void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t key)
{
    __ASSERT(l, "spinlock is null\n");
    ARG_UNUSED(key);

#if CONFIG_USE_STATIC_MEM
    if (!l->inited)
        if (k_spin_lock_init(l))
            return;
    void* pspinlock = l->lock;
    rtw_spin_unlock(&pspinlock);
#else
    if (false == k_sys_q_find_and_remove(l, K_SPINLOCK_Q, 0))
    {
        if (k_spin_lock_init(l))
            return;
        k_sys_q_insert(l, K_SPINLOCK_Q);
    }
    rtw_spin_unlock(&l->lock);
#endif
}

void k_spin_lock_free(struct k_spinlock *l)
{
    __ASSERT(l, "spinlock is null\n");

#if CONFIG_USE_STATIC_MEM
#else
    /* Only in queue, can we free sem */
    if (true == k_sys_q_find_and_remove(l, K_SPINLOCK_Q, 1))
        rtw_spinlock_free(&l->lock);
#endif

    l->inited = 0;
}

/******************************************************************************
 * Thread
 */
#define BT_STACK_THREAD_NUM 30

typedef struct thread_ctx_t
{
    k_thread_entry_t entry;
    k_tid_t thread;
    void *p1;
    void *p2;
    void *p3;
} THREAD_CTX_T;

static k_tid_t sys_thread_array[BT_STACK_THREAD_NUM] = {0};

static bool k_sys_thread_array_insert(void *data)
{
    int i = 0;

    while (i < BT_STACK_THREAD_NUM)
    {
        if (!sys_thread_array[i])
        {
            sys_thread_array[i] = data;
            return true;
        }
        i++;
    }
    return false;
}

static int k_sys_thread_array_find(void *data)
{
    int i = 0;

    while (i < BT_STACK_THREAD_NUM)
    {
        if (sys_thread_array[i] == data)
            return i;
        i++;
    }
    return BT_STACK_THREAD_NUM;
}

static void k_sys_thread_array_remove(int idx)
{
    if (idx >= BT_STACK_THREAD_NUM)
        return;
    sys_thread_array[idx] = 0;
}

static k_tid_t k_sys_thread_array_find_hdl(void *taskhdl)
{
    if (!taskhdl)
        return NULL;

    int i = 0;

    while (i < BT_STACK_THREAD_NUM)
    {
        if (IS_HANDLE_IN_THREAD(sys_thread_array[i], taskhdl))
            return sys_thread_array[i];
        i++;
    }
    return NULL;
}

static thread_return adapter_fun_entry(thread_context context)
{
    if (!context)
        return;

    THREAD_CTX_T *ctx = context;

    rtw_thread_enter("");
    k_sem_give(&ctx->thread->run_sema);

    ctx->entry(ctx->p1, ctx->p2, ctx->p3);
    rtw_free(ctx);

    k_sem_give(&ctx->thread->run_sema);
    rtw_thread_exit();
}

k_tid_t k_current_get(void)
{
    return (k_tid_t)k_sys_thread_array_find_hdl((void *)rtw_get_current_TaskHandle());
}

k_tid_t k_thread_create(struct k_thread *new_thread,
                        k_thread_stack_t stack,
                        size_t stack_size,
                        k_thread_entry_t entry,
                        void *p1, void *p2, void *p3,
                        int prio, uint32_t options, k_timeout_t delay)
{
    ARG_UNUSED(stack);
    ARG_UNUSED(options);

    if (k_sys_thread_array_find(new_thread) < BT_STACK_THREAD_NUM)
        return NULL;

    THREAD_CTX_T *context = (THREAD_CTX_T *)rtw_malloc(sizeof(THREAD_CTX_T));
    if (!context)
        return NULL;
    context->entry = entry;
    context->thread = new_thread;
    context->p1 = p1;
    context->p2 = p2;
    context->p3 = p3;
    new_thread->run = 1;
    k_sem_init(&new_thread->run_sema, 0, 1);
    k_sys_thread_array_insert(new_thread);

    if (!rtw_create_task(&new_thread->task_t, "bt_task",
                         (uint32_t)stack_size, (uint32_t)prio, (thread_func_t)adapter_fun_entry, (void *)context))
    {
        new_thread->run = 0;
        k_sys_thread_array_remove(k_sys_thread_array_find(new_thread));
        k_sem_free(&new_thread->run_sema);
        rtw_free(context);
        return NULL;
    }
    k_sem_take(&new_thread->run_sema, K_FOREVER);

    //if (K_TIMEOUT_EQ(delay, K_FOREVER))
    //    rtw_suspend_task(new_thread->task_t.task);

    return new_thread;
}

void k_thread_stop(k_tid_t thread, k_thread_exit_t fn, void* ctx)
{
    thread->run = 0;
    if (fn) fn(ctx);
    k_sem_take(&thread->run_sema, K_FOREVER);
    k_sem_free(&thread->run_sema);
    k_sys_thread_array_remove(k_sys_thread_array_find(thread));
}

void k_thread_abort(k_tid_t thread)
{
    rtw_delete_task(&thread->task_t);
}

void k_thread_start(k_tid_t thread)
{
    //rtw_resume_task(thread->task_t.task);
}

int k_thread_name_set(k_tid_t thread, const char *value)
{
    return rtw_set_task_name(&thread->task_t, value);
}

extern int rtw_in_interrupt(void);
bool k_is_in_isr(void)
{
    return rtw_in_interrupt();
}

/******************************************************************************
 * Shedule
 */
K_MUTEX_DEFINE(sched_lock);
void k_sched_lock(void)
{
    k_mutex_lock(&sched_lock, K_FOREVER);
    /* XXX: Mask this Intf due to no implementation. */
    //rtw_set_scheduler_state(0);
}

void k_sched_unlock(void)
{
    k_mutex_unlock(&sched_lock);
    /* XXX: Mask this Intf due to no implementation. */
    //rtw_set_scheduler_state(1);
}

int32_t k_sleep(k_timeout_t timeout)
{
    rtw_msleep_os(timeout.ms);
    return 0;
}

void k_busy_wait(uint32_t usec_to_wait)
{
    rtw_udelay_os(usec_to_wait);
}

void k_yield(void)
{
    rtw_yield_os();
}

/******************************************************************************
 * Time & Timer & Timeout
 */
int64_t z_tick_get(void)
{
    return rtw_get_current_time();
}

uint64_t z_timeout_end_calc(k_timeout_t timeout)
{
	uint64_t dt;

	if (K_TIMEOUT_EQ(timeout, K_FOREVER))
		return UINT64_MAX;
	else if (K_TIMEOUT_EQ(timeout, K_NO_WAIT))
		return z_tick_get();

	dt = rtw_ms_to_systime(timeout.ms);

	return z_tick_get() + MAX(1, dt);
}

int64_t k_uptime_get(void)
{
    return rtw_systime_to_ms(rtw_get_current_time());
}

uint32_t k_uptime_get_32(void)
{
    return (uint32_t)k_uptime_get();
}

int64_t k_uptime_delta(uint64_t *reftime)
{
    int64_t uptime, delta;

    uptime = k_uptime_get();
    delta = uptime - *reftime;
    *reftime = uptime;

    return delta;
}

uint32_t k_cycle_get_32(void)
{
    /* What returned here is nano seconds, lose accuracy here */
    return (uint32_t)(rtw_systime_to_ms(rtw_get_current_time()) * 1000000);
}

uint64_t k_cyc_to_ns_floor64(uint64_t cycle)
{
    return cycle;
}

int z_add_timeout(struct _timeout *to, _timeout_func_t fn, k_timeout_t timeout)
{
    /* FIXME: start_ms is not real start time */
    to->start_ms = rtw_systime_to_ms(rtw_get_current_time());
    to->timeout.ms = timeout.ms;
    to->timerID = to;

#if CONFIG_USE_STATIC_MEM
    if (!to->inited)
    {
        memset(to->timerhandle, 0, sizeof(to->timerhandle));
        void *ptimerhandle = to->timerhandle;
        if (rtw_timerCreate_static(&ptimerhandle, "Timer", Z_TIMER_MAX_DELAY, 0, to->timerID, (TIMER_FUN)fn))
            return -EIO;
        if(0 == rtw_timerChangePeriod(to->timerhandle, rtw_ms_to_systime(timeout.ms), Z_TIMER_MAX_DELAY))
            return -EIO;
        to->inited = 1;
    }
#else
    if (to->timerhandle == NULL)
    {
        to->timerhandle = rtw_timerCreate("Timer", Z_TIMER_MAX_DELAY, 0, to->timerID, (TIMER_FUN)fn);
        if (!to->timerhandle)
            return -EIO;
        k_sys_q_insert(to, K_TIMER_Q);
        if(0 == rtw_timerChangePeriod(to->timerhandle, rtw_ms_to_systime(timeout.ms), Z_TIMER_MAX_DELAY))
            return -EIO;
        to->inited = 1;
    }
#endif
    else
    {
        if (rtw_timerIsTimerActive(to->timerhandle)) {
            if (0 == rtw_timerStop(to->timerhandle, Z_TIMER_MAX_DELAY))
                return -EIO;
        }
        if (0 == rtw_timerChangePeriod(to->timerhandle, rtw_ms_to_systime(timeout.ms), Z_TIMER_MAX_DELAY)) {
            BT_DBG_OS("Fail to set timer period");
            return -EIO;
        }
    }

    return 0;
}

bool z_is_inactive_timeout(struct _timeout *to)
{
    return rtw_timerIsTimerActive(to->timerhandle);
}

int z_abort_timeout(struct _timeout *to)
{
    if (!rtw_timerIsTimerActive(to->timerhandle))
        return 0;

    if (0 == rtw_timerStop(to->timerhandle, Z_TIMER_MAX_DELAY))
        return -EIO;

    return 0;
}

/******************************************************************************
 * MEMORY SLAB
 */
int k_mem_slab_alloc(struct k_mem_slab *slab,
		void **mem, k_timeout_t timeout)
{
    ARG_UNUSED(timeout);

	void *buffer = rtw_malloc((uint32_t)slab->block_size);
	if (!buffer)
		return -EINVAL;

	*mem = buffer;
	return 0;
}

void k_mem_slab_free(struct k_mem_slab *slab, void **mem)
{
	free(*mem);
	*mem = NULL;
}

/******************************************************************************
 * POLL (MESSAGE QUEUE)
 */
int k_msg_init(k_msg_queue* msg_q, unsigned int msg_num, unsigned int msg_size)
{	
	if(rtw_init_xqueue(msg_q, "message queue", msg_size, msg_num))
        return -EINVAL;

    return 0;
}

int z_msg_send(k_msg_queue* msg_q, void *p_msg, k_timeout_t timeout)
{
	if(rtw_push_to_xqueue(msg_q, p_msg, timeout.ms))
        return -EINVAL;

    return 0;
}

int k_msg_recv(k_msg_queue* msg_q, void *p_msg, k_timeout_t timeout)
{
	if(rtw_pop_from_xqueue(msg_q, p_msg, timeout.ms))
        return -EINVAL;

	return 0;
}

int k_msg_free(k_msg_queue* msg_q)
{
	if(rtw_deinit_xqueue(msg_q))
        return -EINVAL;

	return 0;
}

/******************************************************************************
 * ERROR HANDLE
 */

void k_oops(void)
{
    HALT();
}

void k_panic(void)
{
    HALT();
}