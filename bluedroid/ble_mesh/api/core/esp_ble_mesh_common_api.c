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

#include <string.h>
#include <errno.h>

#include "btc_task.h"
#include "btc_manage.h"

#include "btc_ble_mesh_prov.h"

#include "mesh.h"
#include "mesh_buf.h"
#include "transport.h"
#include "esp_ble_mesh_common_api.h"

esp_err_t esp_ble_mesh_init(esp_ble_mesh_prov_t *prov, esp_ble_mesh_comp_t *comp)
{
    btc_msg_t msg;
    btc_ble_mesh_prov_args_t arg;

    msg.sig = BTC_SIG_API_CALL;
    msg.pid = BTC_PID_PROV;
    msg.act = BTC_BLE_MESH_ACT_APP_REGISTER;
    arg.mesh_reg.prov = prov;
    arg.mesh_reg.comp = comp;

    return (btc_transfer_context(&msg, &arg, sizeof(btc_ble_mesh_prov_args_t), NULL)
            == BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL);
}


