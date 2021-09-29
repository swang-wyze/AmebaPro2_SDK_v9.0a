/*
 * Copyright (c) 2021, Realsil Semiconductor Corporation. All rights reserved.
 */

#include <kernel.h>

extern void handle_poll_events(struct k_queue *queue, uint32_t state);

void k_queue_init(struct k_queue *queue)
{
    sys_sflist_init(&queue->data_q);
    k_spin_lock_init(&queue->lock);    
    k_sem_init(&queue->wait_sema, 0, 1);
}

void k_queue_deinit(struct k_queue *queue)
{
    sys_sflist_init(&queue->data_q);
    k_spin_lock_free(&queue->lock);
    k_sem_free(&queue->wait_sema);
}

void k_queue_cancel_wait(struct k_queue *queue)
{
    k_sem_give(&queue->wait_sema);
}

void k_queue_append(struct k_queue *queue, void *data)
{
    void *prev;

    k_spinlock_key_t key = k_spin_lock(&queue->lock);

    prev = sys_sflist_peek_tail(&queue->data_q);
    sys_sfnode_init(data, 0x0);
    sys_sflist_insert(&queue->data_q, prev, data);

    k_spin_unlock(&queue->lock, key);

    k_sem_give(&queue->wait_sema);
}

void k_queue_prepend(struct k_queue *queue, void *data)
{
    k_spinlock_key_t key = k_spin_lock(&queue->lock);

    sys_sfnode_init(data, 0x0);
    sys_sflist_insert(&queue->data_q, NULL, data);

    k_spin_unlock(&queue->lock, key);

    k_sem_give(&queue->wait_sema);
}

int k_queue_append_list(struct k_queue *queue, void *head, void *tail)
{
    if (head == NULL || tail == NULL)
        return -1;

    k_spinlock_key_t key = k_spin_lock(&queue->lock);

    sys_sflist_append_list(&queue->data_q, head, tail);

    k_spin_unlock(&queue->lock, key);

    k_sem_give(&queue->wait_sema);

    return 0;
}

void *k_queue_get(struct k_queue *queue, k_timeout_t timeout)
{
    k_spinlock_key_t key;
    void *data = NULL;
    sys_sfnode_t *node;

    k_sem_take(&queue->wait_sema, timeout);

    key = k_spin_lock(&queue->lock);

    if (!sys_sflist_is_empty(&queue->data_q))
    {
        node = sys_sflist_get_not_empty(&queue->data_q);
        data = (void *)node;
    }

    k_spin_unlock(&queue->lock, key);

    return data;
}

bool k_queue_remove(struct k_queue *queue, void *data)
{
    bool ret;
    k_spinlock_key_t key;

    key = k_spin_lock(&queue->lock);
    ret = sys_sflist_find_and_remove(&queue->data_q, (sys_sfnode_t *)data);
    k_spin_unlock(&queue->lock, key);

    return ret;
}

int k_queue_is_empty(struct k_queue *queue)
{
    return (int)sys_sflist_is_empty(&queue->data_q);
}

void *k_queue_peek_head(struct k_queue *queue)
{
    return (void *)sys_sflist_peek_head(&queue->data_q);
}

void *k_queue_peek_tail(struct k_queue *queue)
{
    return (void *)sys_sflist_peek_tail(&queue->data_q);
}
