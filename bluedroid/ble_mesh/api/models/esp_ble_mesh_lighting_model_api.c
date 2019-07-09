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

#include <errno.h>

#include "btc_task.h"
#include "btc_manage.h"

#include "btc_ble_mesh_light_client.h"
#include "esp_ble_mesh_lighting_model_api.h"

esp_err_t esp_ble_mesh_register_light_client_callback(esp_mesh_light_client_cb_t callback)
{
    return (btc_profile_cb_set(BTC_PID_LIGHT_CLIENT, callback) == 0 ? ESP_OK : ESP_FAIL);
}

esp_err_t esp_ble_mesh_light_client_get_state(esp_ble_mesh_client_common_param_t *params,
        esp_ble_mesh_light_client_get_state_t *get_state)
{
    if (!params || !params->model || !params->ctx.addr || !get_state) {
        return ESP_ERR_INVALID_ARG;
    }

    btc_msg_t msg;
    btc_ble_mesh_light_client_args_t arg;

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_LIGHT_CLIENT;
    msg.act = BTC_BLE_MESH_ACT_LIGHT_CLIENT_GET_STATE;
    arg.light_client_get_state.params = params;
    arg.light_client_get_state.get_state = get_state;

    return (btc_transfer_context(&msg, &arg, sizeof(btc_ble_mesh_light_client_args_t), btc_ble_mesh_light_client_arg_deep_copy)
            == BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
}

esp_err_t esp_ble_mesh_light_client_set_state(esp_ble_mesh_client_common_param_t *params,
        esp_ble_mesh_light_client_set_state_t *set_state)
{
    if (!params || !params->model || !params->ctx.addr || !set_state) {
        return ESP_ERR_INVALID_ARG;
    }

    btc_msg_t msg;
    btc_ble_mesh_light_client_args_t arg;

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_LIGHT_CLIENT;
    msg.act = BTC_BLE_MESH_ACT_LIGHT_CLIENT_SET_STATE;
    arg.light_client_set_state.params = params;
    arg.light_client_set_state.set_state = set_state;

    return (btc_transfer_context(&msg, &arg, sizeof(btc_ble_mesh_light_client_args_t), btc_ble_mesh_light_client_arg_deep_copy)
            == BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
}

