/******************************************************************************
 *
 *  Copyright (C) 2014 Google, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/
#define LOG_TAG "osi_future"

#include <assert.h>
#include "future.h"
#include "osi.h"

void future_free(future_t *future);

future_t *future_new(void)
{
    size_t capacity = 2;
    future_t *ret = malloc(sizeof(future_t));
    if (!ret) {
        ALOGE("%s unable to allocate memory for return value.", __func__);
        goto error;
    }

    ret->handler = semaphore_new(capacity);
    if (!(ret->handler)) {
        ALOGE("%s unable to allocate memory for the semaphore.", __func__);
        goto error;
    }

    ret->ready_can_be_called = true;
    return ret;
error:;
    future_free(ret);
    return NULL;
}

future_t *future_new_immediate(void *value)
{
    future_t *ret = malloc(sizeof(future_t));
    if (!ret) {
        ALOGE("%s unable to allocate memory for return value.", __func__);
        goto error;
    }

    ret->result = value;
    ret->ready_can_be_called = false;
    return ret;
error:;
    future_free(ret);
    return NULL;
}

void future_ready(future_t *future, void *value)
{
    assert(future != NULL);
    assert(future->ready_can_be_called);

    future->ready_can_be_called = false;
    future->result = value;
    semaphore_post(future->handler);
}

void *future_await(future_t *future)
{
    assert(future != NULL);

    // If the future is immediate, it will not have a semaphore
    if (future->handler) {
        semaphore_wait(future->handler);
    }

    void *result = future->result;
    future_free(future);
    return result;
}

void future_free(future_t *future)
{
    if (!future) {
        return;
    }

    if (future->handler) {
        semaphore_free(future->handler);
    }

    free(future);
}
