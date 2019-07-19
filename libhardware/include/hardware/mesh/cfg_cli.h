/** @file
 *  @brief Bluetooth Mesh Configuration Client Model APIs.
 */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __BT_MESH_CFG_CLI_H
#define __BT_MESH_CFG_CLI_H

#include "mesh_access.h"
//#include "mesh_kernel.h"
#include "bt_mesh_client_common.h"

/**
 * @brief Bluetooth Mesh
 * @defgroup bt_mesh_cfg_cli Bluetooth Mesh Configuration Client Model
 * @ingroup bt_mesh
 * @{
 */

/* Config client model common structure */
typedef bt_mesh_client_common_t bt_mesh_config_client_t;
typedef bt_mesh_internal_data_t config_internal_data_t;

extern const struct bt_mesh_model_op bt_mesh_cfg_cli_op[];

#define BT_MESH_MODEL_CFG_CLI(cli_data)                                      \
        BT_MESH_MODEL(BT_MESH_MODEL_ID_CFG_CLI,                      \
                  bt_mesh_cfg_cli_op, NULL, cli_data)

int bt_mesh_cfg_comp_data_get(struct bt_mesh_msg_ctx *ctx, u8_t page);

int bt_mesh_cfg_beacon_get(struct bt_mesh_msg_ctx *ctx);

int bt_mesh_cfg_beacon_set(struct bt_mesh_msg_ctx *ctx, u8_t val);

int bt_mesh_cfg_ttl_get(struct bt_mesh_msg_ctx *ctx);

int bt_mesh_cfg_ttl_set(struct bt_mesh_msg_ctx *ctx, u8_t val);

int bt_mesh_cfg_friend_get(struct bt_mesh_msg_ctx *ctx);

int bt_mesh_cfg_friend_set(struct bt_mesh_msg_ctx *ctx, u8_t val);

int bt_mesh_cfg_gatt_proxy_get(struct bt_mesh_msg_ctx *ctx);

int bt_mesh_cfg_gatt_proxy_set(struct bt_mesh_msg_ctx *ctx, u8_t val);

int bt_mesh_cfg_relay_get(struct bt_mesh_msg_ctx *ctx);

int bt_mesh_cfg_relay_set(struct bt_mesh_msg_ctx *ctx, u8_t new_relay, u8_t new_transmit);

int bt_mesh_cfg_net_key_add(struct bt_mesh_msg_ctx *ctx, u16_t key_net_idx,
                            const u8_t net_key[16]);

int bt_mesh_cfg_app_key_add(struct bt_mesh_msg_ctx *ctx, u16_t key_net_idx,
                            u16_t key_app_idx, const u8_t app_key[16]);

int bt_mesh_cfg_mod_app_bind(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                             u16_t mod_app_idx, u16_t mod_id);

int bt_mesh_cfg_mod_app_bind_vnd(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                                 u16_t mod_app_idx, u16_t mod_id, u16_t cid);

struct bt_mesh_cfg_mod_pub {
    u16_t  addr;
    u16_t  app_idx;
    bool   cred_flag;
    u8_t   ttl;
    u8_t   period;
    u8_t   transmit;
};

int bt_mesh_cfg_mod_pub_get(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr, u16_t mod_id);

int bt_mesh_cfg_mod_pub_get_vnd(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                                u16_t mod_id, u16_t cid);

int bt_mesh_cfg_mod_pub_set(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                            u16_t mod_id, struct bt_mesh_cfg_mod_pub *pub);

int bt_mesh_cfg_mod_pub_set_vnd(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                                u16_t mod_id, u16_t cid,
                                struct bt_mesh_cfg_mod_pub *pub);

int bt_mesh_cfg_mod_sub_add(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                            u16_t sub_addr, u16_t mod_id);

int bt_mesh_cfg_mod_sub_add_vnd(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                                u16_t sub_addr, u16_t mod_id, u16_t cid);

int bt_mesh_cfg_mod_sub_del(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                            u16_t sub_addr, u16_t mod_id);

int bt_mesh_cfg_mod_sub_del_vnd(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                                u16_t sub_addr, u16_t mod_id, u16_t cid);

int bt_mesh_cfg_mod_sub_overwrite(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                                  u16_t sub_addr, u16_t mod_id);

int bt_mesh_cfg_mod_sub_overwrite_vnd(struct bt_mesh_msg_ctx *ctx,
                                      u16_t elem_addr, u16_t sub_addr,
                                      u16_t mod_id, u16_t cid);

int bt_mesh_cfg_mod_sub_va_add(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                               const u8_t label[16], u16_t mod_id);

int bt_mesh_cfg_mod_sub_va_add_vnd(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                                   const u8_t label[16], u16_t mod_id, u16_t cid);

int bt_mesh_cfg_mod_sub_va_del(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                               const u8_t label[16], u16_t mod_id);

int bt_mesh_cfg_mod_sub_va_del_vnd(struct bt_mesh_msg_ctx *ctx, u16_t elem_addr,
                                   const u8_t label[16], u16_t mod_id, u16_t cid);

int bt_mesh_cfg_mod_sub_va_overwrite(struct bt_mesh_msg_ctx *ctx,
                                     u16_t elem_addr, const u8_t label[16],
                                     u16_t mod_id);

int bt_mesh_cfg_mod_sub_va_overwrite_vnd(struct bt_mesh_msg_ctx *ctx,
        u16_t elem_addr, const u8_t label[16],
        u16_t mod_id, u16_t cid);

struct bt_mesh_cfg_hb_sub {
    u16_t src;
    u16_t dst;
    u8_t  period;
};

int bt_mesh_cfg_hb_sub_set(struct bt_mesh_msg_ctx *ctx,
                           struct bt_mesh_cfg_hb_sub *sub);

int bt_mesh_cfg_hb_sub_get(struct bt_mesh_msg_ctx *ctx);

struct bt_mesh_cfg_hb_pub {
    u16_t dst;
    u8_t  count;
    u8_t  period;
    u8_t  ttl;
    u16_t feat;
    u16_t net_idx;
};

int bt_mesh_cfg_hb_pub_set(struct bt_mesh_msg_ctx *ctx,
                           const struct bt_mesh_cfg_hb_pub *pub);

int bt_mesh_cfg_hb_pub_get(struct bt_mesh_msg_ctx *ctx);

int bt_mesh_cfg_node_reset(struct bt_mesh_msg_ctx *ctx);

s32_t bt_mesh_cfg_cli_timeout_get(void);
void bt_mesh_cfg_cli_timeout_set(s32_t timeout);

/* Configuration Client Status Message Context */

struct bt_mesh_cfg_comp_data_status {
    u8_t page;
    struct net_buf_simple *comp_data;
};

struct bt_mesh_cfg_relay_status {
    u8_t relay;
    u8_t retransmit;
};

struct bt_mesh_cfg_netkey_status {
    u8_t  status;
    u16_t net_idx;
};

struct bt_mesh_cfg_appkey_status {
    u8_t  status;
    u16_t net_idx;
    u16_t app_idx;
};

struct bt_mesh_cfg_mod_app_status {
    u8_t  status;
    u16_t elem_addr;
    u16_t app_idx;
    u16_t cid;
    u16_t mod_id;
};

struct bt_mesh_cfg_mod_pub_status {
    u8_t  status;
    u16_t elem_addr;
    u16_t addr;
    u16_t app_idx;
    bool  cred_flag;
    u8_t  ttl;
    u8_t  period;
    u8_t  transmit;
    u16_t cid;
    u16_t mod_id;
};

struct bt_mesh_cfg_mod_sub_status {
    u8_t  status;
    u16_t elem_addr;
    u16_t sub_addr;
    u16_t cid;
    u16_t mod_id;
};

struct bt_mesh_cfg_hb_sub_status {
    u8_t  status;
    u16_t src;
    u16_t dst;
    u8_t  period;
    u8_t  count;
    u8_t  min;
    u8_t  max;
};

struct bt_mesh_cfg_hb_pub_status {
    u8_t  status;
    u16_t dst;
    u8_t  count;
    u8_t  period;
    u8_t  ttl;
    u16_t feat;
    u16_t net_idx;
};

/**
 * @}
 */

#endif /* __BT_MESH_CFG_CLI_H */
