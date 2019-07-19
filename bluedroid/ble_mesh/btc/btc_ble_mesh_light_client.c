// Copyright 2017-2018 Espressif Systems (Shanghai) PTE LTD
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
#define LOG_TAG "btc_ble_mesh_light_client"
#include <utils/Log.h>

#include <string.h>
#include <errno.h>

#include "btc_manage.h"

#include "light_client.h"
#include "btc_ble_mesh_light_client.h"
#include "esp_ble_mesh_lighting_model_api.h"

static inline void btc_ble_mesh_cb_to_app(esp_ble_mesh_light_client_cb_event_t event,
        esp_ble_mesh_light_client_cb_param_t *param)
{
    esp_mesh_light_client_cb_t btc_mesh_cb = (esp_mesh_light_client_cb_t)btc_profile_cb_get(BTC_PID_LIGHT_CLIENT);
    if (btc_mesh_cb) {
        btc_mesh_cb(event, param);
    }
}

void btc_ble_mesh_light_client_arg_deep_copy(btc_msg_t *msg, void *p_dest, void *p_src)
{
    btc_ble_mesh_light_client_args_t *dst = (btc_ble_mesh_light_client_args_t *)p_dest;
    btc_ble_mesh_light_client_args_t *src = (btc_ble_mesh_light_client_args_t *)p_src;

    if (!msg || !dst || !src) {
        ALOGE("%s: parameter is NULL", __func__);
        return;
    }

    switch (msg->act) {
    case BTC_BLE_MESH_ACT_LIGHT_CLIENT_GET_STATE: {
        dst->light_client_get_state.params = (esp_ble_mesh_client_common_param_t *)malloc(sizeof(esp_ble_mesh_client_common_param_t));
        if (dst->light_client_get_state.params) {
            memcpy(dst->light_client_get_state.params, src->light_client_get_state.params,
                   sizeof(esp_ble_mesh_client_common_param_t));
        } else {
            ALOGE("%s %d no mem", __func__, msg->act);
        }
#if 0   /* Currently light get state is a empty union */
        dst->light_client_get_state.get_state = (esp_ble_mesh_light_client_get_state_t *)malloc(sizeof(esp_ble_mesh_light_client_get_state_t));
        if (dst->light_client_get_state.get_state) {
            memcpy(dst->light_client_get_state.get_state, src->light_client_get_state.get_state,
                   sizeof(esp_ble_mesh_light_client_get_state_t));
        } else {
            ALOGE("%s %d no mem", __func__, msg->act);
        }
#endif
        break;
    }
    case BTC_BLE_MESH_ACT_LIGHT_CLIENT_SET_STATE: {
        dst->light_client_set_state.params = (esp_ble_mesh_client_common_param_t *)malloc(sizeof(esp_ble_mesh_client_common_param_t));
        dst->light_client_set_state.set_state = (esp_ble_mesh_light_client_set_state_t *)malloc(sizeof(esp_ble_mesh_light_client_set_state_t));
        if (dst->light_client_set_state.params && dst->light_client_set_state.set_state) {
            memcpy(dst->light_client_set_state.params, src->light_client_set_state.params,
                   sizeof(esp_ble_mesh_client_common_param_t));
            memcpy(dst->light_client_set_state.set_state, src->light_client_set_state.set_state,
                   sizeof(esp_ble_mesh_light_client_set_state_t));
        } else {
            ALOGE("%s %d no mem", __func__, msg->act);
        }
        break;
    }
    default:
        ALOGE("%s Unhandled deep copy %d", __func__, msg->act);
        break;
    }
}

static void btc_ble_mesh_copy_req_data(btc_msg_t *msg, void *p_dest, void *p_src)
{
    esp_ble_mesh_light_client_cb_param_t *p_dest_data = (esp_ble_mesh_light_client_cb_param_t *)p_dest;
    esp_ble_mesh_light_client_cb_param_t *p_src_data = (esp_ble_mesh_light_client_cb_param_t *)p_src;

    if (!msg || !p_src_data || !p_dest_data) {
        ALOGE("%s: parameter is NULL", __func__);
        return;
    }

    switch (msg->act) {
    case ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT:
    case ESP_BLE_MESH_LIGHT_CLIENT_SET_STATE_EVT:
    case ESP_BLE_MESH_LIGHT_CLIENT_PUBLISH_EVT:
    case ESP_BLE_MESH_LIGHT_CLIENT_TIMEOUT_EVT:
        if (p_src_data->params) {
            p_dest_data->params = malloc(sizeof(esp_ble_mesh_client_common_param_t));
            if (p_dest_data->params) {
                memcpy(p_dest_data->params, p_src_data->params, sizeof(esp_ble_mesh_client_common_param_t));
            } else {
                ALOGE("%s %d no mem", __func__, msg->act);
            }
        }
        break;
    default:
        break;
    }
}

static void btc_ble_mesh_free_req_data(btc_msg_t *msg)
{
    esp_ble_mesh_light_client_cb_param_t *arg = NULL;

    if (!msg || !msg->arg) {
        ALOGE("%s: parameter is NULL", __func__);
        return;
    }

    arg = (esp_ble_mesh_light_client_cb_param_t *)(msg->arg);

    switch (msg->act) {
    case ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT:
    case ESP_BLE_MESH_LIGHT_CLIENT_SET_STATE_EVT:
    case ESP_BLE_MESH_LIGHT_CLIENT_PUBLISH_EVT:
    case ESP_BLE_MESH_LIGHT_CLIENT_TIMEOUT_EVT:
        if (arg->params) {
            free(arg->params);
        }
        break;
    default:
        break;
    }
}

void btc_ble_mesh_light_client_arg_deep_free(btc_msg_t *msg)
{
    btc_ble_mesh_light_client_args_t *arg = NULL;

    if (!msg || !msg->arg) {
        ALOGE("%s: parameter is NULL", __func__);
        return;
    }

    arg = (btc_ble_mesh_light_client_args_t *)(msg->arg);

    switch (msg->act) {
    case BTC_BLE_MESH_ACT_LIGHT_CLIENT_GET_STATE:
        if (arg->light_client_get_state.params) {
            free(arg->light_client_get_state.params);
        }
#if 0   /* Currently light get state is a empty union */
        if (arg->light_client_get_state.get_state) {
            free(arg->light_client_get_state.get_state);
        }
#endif
        break;
    case BTC_BLE_MESH_ACT_LIGHT_CLIENT_SET_STATE:
        if (arg->light_client_set_state.params) {
            free(arg->light_client_set_state.params);
        }
        if (arg->light_client_set_state.set_state) {
            free(arg->light_client_set_state.set_state);
        }
        break;
    default:
        break;
    }

    return;
}

static void btc_mesh_light_client_callback(esp_ble_mesh_light_client_cb_param_t *cb_params, uint8_t act)
{
    btc_msg_t msg = {0};

    ALOGE("%s", __func__);

    msg.sig = BTC_SIG_API_CB;
    msg.pid = BTC_PID_LIGHT_CLIENT;
    msg.act = act;

    btc_transfer_context(&msg, cb_params,
                         sizeof(esp_ble_mesh_light_client_cb_param_t), btc_ble_mesh_copy_req_data);
}

void bt_mesh_callback_light_status_to_btc(u32_t opcode, u8_t evt_type,
        struct bt_mesh_model *model,
        struct bt_mesh_msg_ctx *ctx,
        const u8_t *val, size_t len)
{
    esp_ble_mesh_light_client_cb_param_t cb_params = {0};
    esp_ble_mesh_client_common_param_t   params    = {0};
    size_t  length;
    uint8_t act;

    if (!model || !ctx) {
        ALOGE("%s: invalid parameter", __func__);
        return;
    }

    switch (evt_type) {
    case 0x00:
        act = ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT;
        break;
    case 0x01:
        act = ESP_BLE_MESH_LIGHT_CLIENT_SET_STATE_EVT;
        break;
    case 0x02:
        act = ESP_BLE_MESH_LIGHT_CLIENT_PUBLISH_EVT;
        break;
    case 0x03:
        act = ESP_BLE_MESH_LIGHT_CLIENT_TIMEOUT_EVT;
        break;
    default:
        ALOGE("%s: unknown light status event type", __func__);
        return;
    }

    params.opcode = opcode;
    params.model = (esp_ble_mesh_model_t *)model;
    params.ctx.net_idx = ctx->net_idx;
    params.ctx.app_idx = ctx->app_idx;
    params.ctx.addr = ctx->addr;
    params.ctx.recv_ttl = ctx->recv_ttl;
    params.ctx.recv_op = ctx->recv_op;
    params.ctx.recv_dst = ctx->recv_dst;

    cb_params.error_code = 0;
    cb_params.params = &params;

    if (val && len) {
        length = (len <= sizeof(cb_params.status_cb)) ? len : sizeof(cb_params.status_cb);
        memcpy(&cb_params.status_cb, val, length);
    }

    btc_mesh_light_client_callback(&cb_params, act);
}

void btc_mesh_light_client_publish_callback(u32_t opcode, struct bt_mesh_model *model,
        struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
    if (!model || !ctx || !buf) {
        ALOGE("%s: invalid parameter", __func__);
        return;
    }

    bt_mesh_callback_light_status_to_btc(opcode, 0x02, model, ctx, buf->data, buf->len);
}

void btc_mesh_light_client_call_handler(btc_msg_t *msg)
{
    esp_ble_mesh_light_client_cb_param_t light_client_cb = {0};
    esp_ble_mesh_client_common_param_t *params = NULL;
    btc_ble_mesh_light_client_args_t *arg = NULL;
    struct bt_mesh_common_param common = {0};
    bt_mesh_role_param_t role_param = {0};

    if (!msg || !msg->arg) {
        ALOGE("%s: parameter is NULL", __func__);
        return;
    }

    arg = (btc_ble_mesh_light_client_args_t *)(msg->arg);

    switch (msg->act) {
    case BTC_BLE_MESH_ACT_LIGHT_CLIENT_GET_STATE: {
        params = arg->light_client_get_state.params;
        role_param.model = (struct bt_mesh_model *)params->model;
        role_param.role = params->msg_role;
        if (bt_mesh_copy_msg_role(&role_param)) {
            ALOGE("%s: copy msg_role fail", __func__);
            return;
        }
        common.opcode = params->opcode;
        common.model = (struct bt_mesh_model *)params->model;
        common.ctx.net_idx = params->ctx.net_idx;
        common.ctx.app_idx = params->ctx.app_idx;
        common.ctx.addr = params->ctx.addr;
        common.ctx.send_rel = params->ctx.send_rel;
        common.ctx.send_ttl = params->ctx.send_ttl;
        common.msg_timeout = params->msg_timeout;

        light_client_cb.params = arg->light_client_get_state.params;
        light_client_cb.error_code =
            bt_mesh_light_client_get_state(&common,
                                           (void *)arg->light_client_get_state.get_state,
                                           (void *)&light_client_cb.status_cb);
        if (light_client_cb.error_code) {
            /* If send failed, callback error_code to app layer immediately */
            btc_mesh_light_client_callback(&light_client_cb,
                                           ESP_BLE_MESH_LIGHT_CLIENT_GET_STATE_EVT);
        }
        break;
    }
    case BTC_BLE_MESH_ACT_LIGHT_CLIENT_SET_STATE: {
        params = arg->light_client_set_state.params;
        role_param.model = (struct bt_mesh_model *)params->model;
        role_param.role = params->msg_role;
        if (bt_mesh_copy_msg_role(&role_param)) {
            ALOGE("%s: copy msg_role fail", __func__);
            return;
        }
        common.opcode = params->opcode;
        common.model = (struct bt_mesh_model *)params->model;
        common.ctx.net_idx = params->ctx.net_idx;
        common.ctx.app_idx = params->ctx.app_idx;
        common.ctx.addr = params->ctx.addr;
        common.ctx.send_rel = params->ctx.send_rel;
        common.ctx.send_ttl = params->ctx.send_ttl;
        common.msg_timeout = params->msg_timeout;

        light_client_cb.params = arg->light_client_set_state.params;
        light_client_cb.error_code =
            bt_mesh_light_client_set_state(&common,
                                           (void *)arg->light_client_set_state.set_state,
                                           (void *)&light_client_cb.status_cb);
        if (light_client_cb.error_code) {
            /* If send failed, callback error_code to app layer immediately */
            btc_mesh_light_client_callback(&light_client_cb,
                                           ESP_BLE_MESH_LIGHT_CLIENT_SET_STATE_EVT);
        }
        break;
    }
    default:
        break;
    }

    btc_ble_mesh_light_client_arg_deep_free(msg);
}

void btc_mesh_light_client_cb_handler(btc_msg_t *msg)
{
    esp_ble_mesh_light_client_cb_param_t *param = NULL;

    if (!msg || !msg->arg) {
        ALOGE("%s: parameter is NULL", __func__);
        return;
    }

    param = (esp_ble_mesh_light_client_cb_param_t *)(msg->arg);

    if (msg->act < ESP_BLE_MESH_LIGHT_CLIENT_EVT_MAX) {
        btc_ble_mesh_cb_to_app(msg->act, param);
    } else {
        ALOGE("%s, unknown msg->act = %d", __func__, msg->act);
    }

    btc_ble_mesh_free_req_data(msg);
}

