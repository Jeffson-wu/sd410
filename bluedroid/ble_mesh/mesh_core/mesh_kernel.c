/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2016 Wind River Systems, Inc.
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_TAG "mesh_kernel"
#include <utils/Log.h>

#include <unistd.h>
#include <errno.h>
#include <time.h>

#include "config.h"
#include "mesh_kernel.h"
#include "hash_map.h"
#include "alarm.h"
#include "hash_functions.h"

#if CONFIG_BT_MESH
#include "provisioner_prov.h"

static pthread_mutex_t ble_mesh_alarm_lock;
static pthread_mutex_t mesh_irq_lock;
static hash_map_t *ble_mesh_alarm_hash_map;
static const size_t BLE_MESH_GENERAL_ALARM_HASH_MAP_SIZE = 20 + CONFIG_BT_MESH_PBA_SAME_TIME + \
        CONFIG_BT_MESH_PBG_SAME_TIME;

static const clockid_t CLOCK_ID = CLOCK_BOOTTIME;
typedef struct alarm_t {
    /* timer id point to here */
    pthread_mutex_t callback_lock;
    period_ms_t deadline;
    alarm_callback_t cb;
    void *data;
} osi_alarm_t;

static void ble_mesh_alarm_cb(void *data)
{
    assert(data != NULL);
    struct k_delayed_work *work = (struct k_delayed_work *)data;
    work->work.handler(&work->work);
    return;
}

unsigned int irq_lock(void)
{
#if defined(CONFIG_BT_MESH_IRQ_LOCK) && CONFIG_BT_MESH_IRQ_LOCK
    unsigned int key = XTOS_SET_INTLEVEL(XCHAL_EXCM_LEVEL);
    return key;
#else /* !CONFIG_BT_MESH_IRQ_LOCK */
    /* Change by Espressif. In BLE Mesh, in order to improve the real-time
     * requirements of bt controller, we use task lock to replace IRQ lock.
     */
    pthread_mutex_lock(&mesh_irq_lock);
    return 0;
#endif /*#if (CONFIG_BT_MESH_IRQ_LOCK) */
}

void irq_unlock(unsigned int key)
{
#if defined(CONFIG_BT_MESH_IRQ_LOCK) && CONFIG_BT_MESH_IRQ_LOCK
    XTOS_RESTORE_INTLEVEL(key);
#else /* !CONFIG_BT_MESH_IRQ_LOCK */
    pthread_mutex_unlock(&mesh_irq_lock);
#endif /*#if (CONFIG_BT_MESH_IRQ_LOCK) && CONFIG_BT_MESH_IRQ_LOCK */
}

s64_t k_uptime_get(void)
{
    /** k_uptime_get_32 is in in milliseconds,
     * but esp_timer_get_time is in microseconds
     */
    struct timespec ts;
    if (clock_gettime(CLOCK_ID, &ts) == -1) {
      ALOGE("%s unable to get current time: %s", __func__, strerror(errno));
      return 0;
    }

    return (ts.tv_sec * 1000LL) + (ts.tv_nsec / 1000000LL);
}

u32_t k_uptime_get_32(void)
{
    /** k_uptime_get_32 is in in milliseconds,
     * but esp_timer_get_time is in microseconds
     */
    struct timespec ts;
    if (clock_gettime(CLOCK_ID, &ts) == -1) {
      ALOGE("%s unable to get current time: %s", __func__, strerror(errno));
      return 0;
    }

    return (u32_t)(ts.tv_sec * 1000LL) + (ts.tv_nsec / 1000000LL);

}

void k_sleep(s32_t duration)
{
    usleep(duration);
    return;
}

void mesh_k_init(void)
{
    pthread_mutex_init(&ble_mesh_alarm_lock, NULL);
    pthread_mutex_init(&mesh_irq_lock, NULL);
    ble_mesh_alarm_hash_map = hash_map_new(BLE_MESH_GENERAL_ALARM_HASH_MAP_SIZE,
                                           hash_function_pointer, NULL, (data_free_fn)alarm_free, NULL);
    if (ble_mesh_alarm_hash_map == NULL) {
        goto error_exit;
    }

    return;
error_exit:
    ALOGE("%s Unable to allocate resources for ble mesh alarm hash map", __func__);
    return;
}

void k_delayed_work_init(struct k_delayed_work *work, k_work_handler_t handler)
{
    assert(work != NULL && ble_mesh_alarm_hash_map != NULL);

    k_work_init(&work->work, handler);
    _init_timeout(&work->timeout, NULL);
    work->work_q = NULL;

    osi_alarm_t *alarm = NULL;

    // Get the alarm for the timer list entry.
    pthread_mutex_lock(&ble_mesh_alarm_lock);
    if (!hash_map_has_key(ble_mesh_alarm_hash_map, (void *)work)) {
        alarm = alarm_new();
        if (!hash_map_set(ble_mesh_alarm_hash_map, work, (void *)alarm)) {
            ALOGE("%s Unable to add the work timer to the mesh alram hash map.", __func__);
        }
    }
    pthread_mutex_unlock(&ble_mesh_alarm_lock);

    alarm = hash_map_get(ble_mesh_alarm_hash_map, work);
    if (alarm == NULL) {
        ALOGE("%s Unable to create alarm", __func__);
        return;
    }
    // Just init the work timer only, don't start it.
    alarm_cancel(alarm);
    // The following line code just for testing.
    return;

}

int k_delayed_work_submit(struct k_delayed_work *work,
                          s32_t delay)
{
    assert(work != NULL);

    alarm_t *alarm = hash_map_get(ble_mesh_alarm_hash_map, (void *)work);
    if (alarm == NULL) {
        ALOGE("%s, The ble mesh hash map didn't find the alram., work = %p", __func__, work);
        return -EINVAL;
    }
    //cancel the alarm first, before start the alarm.
    alarm_cancel(alarm);

    alarm_set(alarm, delay, ble_mesh_alarm_cb, (void *)work);
    return 0;
}


int k_delayed_work_cancel(struct k_delayed_work *work)
{
    assert(work != NULL);

    // Check the work have been store in the ble_mesh timer list or not.
    osi_alarm_t *alarm = hash_map_get(ble_mesh_alarm_hash_map, (void *)work);
    if (alarm == NULL) {
        ALOGE("%s, The ble mesh hash map didn't find the alram.", __func__);
        return -EINVAL;
    }
    alarm_cancel(alarm);
    return 0;
}

int k_delayed_work_free(struct k_delayed_work *work)
{
    assert(work != NULL);

    // Get the alarm for the timer list entry.
    osi_alarm_t *alarm = hash_map_get(ble_mesh_alarm_hash_map, work);
    if (alarm == NULL) {
        ALOGE("%s Unable to find expected alarm in hashmap", __func__);
        return -EINVAL;
    }

    hash_map_erase(ble_mesh_alarm_hash_map, work);

    return 0;
}

s32_t k_delayed_work_remaining_get(struct k_delayed_work *work)
{
    assert(work != NULL);
    s32_t remain_time = 0;
    int64_t now = k_uptime_get();
    osi_alarm_t *alarm = hash_map_get(ble_mesh_alarm_hash_map, (void *)work);
    if (alarm == NULL) {
        ALOGE("%s, The mesh hash map didn't find the alarm.", __func__);
        return 0;
    }
    if ((alarm->deadline - now) < 0x1FFFFFFFFFF) {
        remain_time = (alarm->deadline - now) / 1000;
    } else {
        return 0;
    }
    return remain_time;
}

#if 0
void k_sem_give(struct k_sem *sem)
{
    assert(sem != NULL);
    pthread_mutex_unlock(sem->mutex);
    return;
}

void k_sem_init(struct k_sem *sem, unsigned int initial_count,
                unsigned int limit)
{
    assert(sem != NULL);
    sem->mutex = xSemaphoreCreateBinary();
    if (sem->mutex == NULL) {
        LOG_WARN("%s, the mutex alloc fail", __func__);
        return;
    }

    return;
}

int k_sem_take(struct k_sem *sem, s32_t timeout)
{
    assert(sem != NULL);
    return pthread_mutex_lock(sem->mutex, timeout);
}
#endif
#endif /* #if CONFIG_BT_MESH */
