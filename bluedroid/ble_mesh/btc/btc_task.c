// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define LOG_TAG "btc_task"
#include <utils/Log.h>

#include <stdlib.h>
#include <string.h>
#include "btc_task.h"
#include "btc_ble_mesh_prov.h"
#include "btc_ble_mesh_health.h"
#include "btc_ble_mesh_config_client.h"
#include "btc_ble_mesh_generic_client.h"
#include "btc_ble_mesh_light_client.h"
#include "btc_ble_mesh_sensor_client.h"
#include "btc_ble_mesh_time_scene_client.h"

static btc_func_t profile_tab[BTC_PID_NUM] = {
    [BTC_PID_PROV]            = {btc_mesh_prov_call_handler, btc_mesh_prov_cb_handler},
    [BTC_PID_MODEL]           = {btc_mesh_model_call_handler, btc_mesh_model_cb_handler},
    [BTC_PID_HEALTH_CLIENT]   = {btc_mesh_health_client_call_handler, btc_mesh_health_client_cb_handler},
    [BTC_PID_HEALTH_SERVER]   = {btc_mesh_health_server_call_handler, btc_mesh_health_server_cb_handler},
    [BTC_PID_CFG_CLIENT]      = {btc_mesh_cfg_client_call_handler, btc_mesh_cfg_client_cb_handler},
    [BTC_PID_GENERIC_CLIENT]  = {btc_mesh_generic_client_call_handler, btc_mesh_generic_client_cb_handler},
    [BTC_PID_LIGHT_CLIENT]    = {btc_mesh_light_client_call_handler, btc_mesh_light_client_cb_handler},
    [BTC_PID_SENSOR_CLIENT]   = {btc_mesh_sensor_client_call_handler, btc_mesh_sensor_client_cb_handler},
    [BTC_PID_TIME_SCENE_CLIENT] = {btc_mesh_time_scene_client_call_handler, btc_mesh_time_scene_client_cb_handler},
};

bt_status_t btc_transfer_context(btc_msg_t *msg, void *arg, int arg_len, btc_arg_deep_copy_t copy_func)
{
    btc_msg_t lmsg;

    if (msg == NULL) {
        return BT_STATUS_PARM_INVALID;
    }

    ALOGE("%s msg %u %u %u %p\n", __func__, msg->sig, msg->pid, msg->act, arg);

    memcpy(&lmsg, msg, sizeof(btc_msg_t));
    if (arg) {
        lmsg.arg = (void *)malloc(arg_len);
        if (lmsg.arg == NULL) {
            return BT_STATUS_NOMEM;
        }
        memset(lmsg.arg, 0x00, arg_len);    //important, avoid arg which have no length
        memcpy(lmsg.arg, arg, arg_len);
        if (copy_func) {
            copy_func(&lmsg, lmsg.arg, arg);
        }
    } else {
        lmsg.arg = NULL;
    }

    switch (lmsg.sig) {
    case BTC_SIG_API_CALL:
        profile_tab[lmsg.pid].btc_call(&lmsg);
        break;
    case BTC_SIG_API_CB:
        profile_tab[lmsg.pid].btc_cb(&lmsg);
        break;
    default:
        break;
    }
    if (lmsg.arg) {
        free(lmsg.arg);
    }

    return BT_STATUS_SUCCESS;
}

