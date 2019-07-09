/*  Bluetooth Mesh */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 * Additional Copyright (c) 2019 Borderless Incoporated
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "access"
#include <utils/Log.h>
#include <assert.h>

#include "errno.h"
#include "mesh_util.h"

#include "mesh_buf.h"
#if CONFIG_BT_MESH
#include "mesh_kernel.h"
#include "mesh_access.h"
#include "mesh_main.h"

#define ALOGE_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_ACCESS)

#include "mesh.h"
#include "adv.h"
#include "net.h"
#include "lpn.h"
#include "transport.h"
#include "access.h"
#include "foundation.h"

#include "generic_client.h"
#include "sensor_client.h"
#include "time_scene_client.h"
#include "light_client.h"

#include "common.h"

#define CODE_UNREACHABLE ALOGE("The op_code not define.");

#define BT_MESH_SDU_MAX_LEN     384

static const struct bt_mesh_comp *dev_comp;
static u16_t dev_primary_addr;

static const struct {
    const u16_t id;
    int (*const init)(struct bt_mesh_model *model, bool primary);
} model_init[] = {
    { BT_MESH_MODEL_ID_CFG_SRV, bt_mesh_cfg_srv_init },
    { BT_MESH_MODEL_ID_HEALTH_SRV, bt_mesh_health_srv_init },
#if defined(CONFIG_BT_MESH_CFG_CLI)
    { BT_MESH_MODEL_ID_CFG_CLI, bt_mesh_cfg_cli_init },
#endif
#if defined(CONFIG_BT_MESH_HEALTH_CLI)
    { BT_MESH_MODEL_ID_HEALTH_CLI, bt_mesh_health_cli_init },
#endif
#if defined(CONFIG_BT_MESH_GENERIC_ONOFF_CLI)
    { BT_MESH_MODEL_ID_GEN_ONOFF_CLI, bt_mesh_gen_onoff_cli_init },
#endif
#if defined(CONFIG_BT_MESH_GENERIC_LEVEL_CLI)
    { BT_MESH_MODEL_ID_GEN_LEVEL_CLI, bt_mesh_gen_level_cli_init },
#endif
#if defined(CONFIG_BT_MESH_GENERIC_DEF_TRANS_TIME_CLI)
    { BT_MESH_MODEL_ID_GEN_DEF_TRANS_TIME_CLI, bt_mesh_gen_def_trans_time_cli_init },
#endif
#if defined(CONFIG_BT_MESH_GENERIC_POWER_ONOFF_CLI)
    { BT_MESH_MODEL_ID_GEN_POWER_ONOFF_CLI, bt_mesh_gen_pwr_onoff_cli_init },
#endif
#if defined(CONFIG_BT_MESH_GENERIC_POWER_LEVEL_CLI)
    { BT_MESH_MODEL_ID_GEN_POWER_LEVEL_CLI, bt_mesh_gen_pwr_level_cli_init },
#endif
#if defined(CONFIG_BT_MESH_GENERIC_BATTERY_CLI)
    { BT_MESH_MODEL_ID_GEN_BATTERY_CLI, bt_mesh_gen_battery_cli_init },
#endif
#if defined(CONFIG_BT_MESH_GENERIC_LOCATION_CLI)
    { BT_MESH_MODEL_ID_GEN_LOCATION_CLI, bt_mesh_gen_location_cli_init },
#endif
#if defined(CONFIG_BT_MESH_GENERIC_PROPERTY_CLI)
    { BT_MESH_MODEL_ID_GEN_PROP_CLI, bt_mesh_gen_property_cli_init },
#endif
#if defined(CONFIG_BT_MESH_SENSOR_CLI)
    { BT_MESH_MODEL_ID_SENSOR_CLI, bt_mesh_sensor_cli_init },
#endif
#if defined(CONFIG_BT_MESH_SCENE_CLI)
    { BT_MESH_MODEL_ID_SCENE_CLI, bt_mesh_scene_cli_init },
#endif
#if defined(CONFIG_BT_MESH_LIGHT_LIGHTNESS_CLI)
    { BT_MESH_MODEL_ID_LIGHT_LIGHTNESS_CLI, bt_mesh_light_lightness_cli_init },
#endif
#if defined(CONFIG_BT_MESH_LIGHT_CTL_CLI)
    { BT_MESH_MODEL_ID_LIGHT_CTL_CLI, bt_mesh_light_ctl_cli_init },
#endif
#if defined(CONFIG_BT_MESH_LIGHT_HSL_CLI)
    { BT_MESH_MODEL_ID_LIGHT_HSL_CLI, bt_mesh_light_hsl_cli_init },
#endif
};

void bt_mesh_model_foreach(void (*func)(struct bt_mesh_model *mod,
                                        struct bt_mesh_elem *elem,
                                        bool vnd, bool primary,
                                        void *user_data),
                           void *user_data)
{
    int i, j;

    for (i = 0; i < (int)dev_comp->elem_count; i++) {
        struct bt_mesh_elem *elem = &dev_comp->elem[i];
        for (j = 0; j < elem->model_count; j++) {
            struct bt_mesh_model *model = &elem->models[j];

            func(model, elem, false, i == 0, user_data);
        }
        for (j = 0; j < elem->vnd_model_count; j++) {
            struct bt_mesh_model *model = &elem->vnd_models[j];

            func(model, elem, true, i == 0, user_data);
        }
    }
}

s32_t bt_mesh_model_pub_period_get(struct bt_mesh_model *mod)
{
    int period = 0;

    if (!mod->pub) {
        return 0;
    }

    switch (mod->pub->period >> 6) {
    case 0x00:
        /* 1 step is 100 ms */
        period = K_MSEC((mod->pub->period & BIT_MASK(6)) * 100);
        break;
    case 0x01:
        /* 1 step is 1 second */
        period = K_SECONDS(mod->pub->period & BIT_MASK(6));
        break;
    case 0x02:
        /* 1 step is 10 seconds */
        period = K_SECONDS((mod->pub->period & BIT_MASK(6)) * 10);
        break;
    case 0x03:
        /* 1 step is 10 minutes */
        period = K_MINUTES((mod->pub->period & BIT_MASK(6)) * 10);
        break;
    default:
        CODE_UNREACHABLE;
    }

    return period >> mod->pub->period_div;
}

static s32_t next_period(struct bt_mesh_model *mod)
{
    struct bt_mesh_model_pub *pub = mod->pub;
    u32_t elapsed, period;

    period = bt_mesh_model_pub_period_get(mod);
    if (!period) {
        return 0;
    }

    elapsed = k_uptime_get_32() - pub->period_start;

    ALOGE("Publishing took %ums", elapsed);

    if (elapsed > period) {
        ALOGE("Publication sending took longer than the period");
        /* Return smallest positive number since 0 means disabled */
        return K_MSEC(1);
    }

    return period - elapsed;
}

static void publish_sent(int err, void *user_data)
{
    struct bt_mesh_model *mod = user_data;
    s32_t delay;

    ALOGE("err %d", err);

    if (mod->pub->count) {
        delay = BT_MESH_PUB_TRANSMIT_INT(mod->pub->retransmit);
    } else {
        delay = next_period(mod);
    }

    if (delay) {
        ALOGE("Publishing next time in %dms", delay);
        k_delayed_work_submit(&mod->pub->timer, delay);
    }
}

static const struct bt_mesh_send_cb pub_sent_cb = {
    .end = publish_sent,
};

static int publish_retransmit(struct bt_mesh_model *mod)
{
    struct net_buf_simple *sdu = NET_BUF_SIMPLE(min(BT_MESH_TX_SDU_MAX, BT_MESH_SDU_MAX_LEN));
    struct bt_mesh_model_pub *pub = mod->pub;
    struct bt_mesh_app_key *key;
    struct bt_mesh_msg_ctx ctx = {
        .addr = pub->addr,
        .send_ttl = pub->ttl,
        .model = mod,
        .srv_send = pub->dev_role == NODE ? true : false,
    };
    struct bt_mesh_net_tx tx = {
        .ctx = &ctx,
        .src = mod->elem->addr,
        .xmit = bt_mesh_net_transmit_get(),
        .friend_cred = pub->cred,
    };

    key = bt_mesh_app_key_find(pub->key);
    if (!key) {
        return -EADDRNOTAVAIL;
    }

    tx.sub = bt_mesh_subnet_get(key->net_idx);

    ctx.net_idx = key->net_idx;
    ctx.app_idx = key->app_idx;

    net_buf_simple_init(sdu, 0);
    net_buf_simple_add_mem(sdu, pub->msg->data, pub->msg->len);

    pub->count--;

    return bt_mesh_trans_send(&tx, sdu, &pub_sent_cb, mod);
}

static void mod_publish(struct k_work *work)
{
    struct bt_mesh_model_pub *pub = CONTAINER_OF(work,
                                    struct bt_mesh_model_pub,
                                    timer.work);
    s32_t period_ms;
    int err;

    ALOGE("%s", __func__);

    period_ms = bt_mesh_model_pub_period_get(pub->mod);
    ALOGE("period %u ms", period_ms);

    if (pub->count) {
        err = publish_retransmit(pub->mod);
        if (err) {
            ALOGE("Failed to retransmit (err %d)", err);

            pub->count = 0;

            /* Continue with normal publication */
            if (period_ms) {
                k_delayed_work_submit(&pub->timer, period_ms);
            }
        }

        return;
    }

    if (!period_ms) {
        return;
    }

    assert(pub->update != NULL);

    pub->period_start = k_uptime_get_32();

    err = pub->update(pub->mod);
    if (err) {
        ALOGE("Failed to update publication message");
        return;
    }

    err = bt_mesh_model_publish(pub->mod);
    if (err) {
        ALOGE("Publishing failed (err %d)", err);
    }

    if (pub->count) {
        /* Retransmissions also control the timer */
        k_delayed_work_cancel(&pub->timer);
    }
}

static void mod_init(struct bt_mesh_model *mod, struct bt_mesh_elem *elem,
                     bool vnd, bool primary, void *user_data)
{
    int i;

    mod->elem = elem;
    if (mod->pub) {
        mod->pub->mod = mod;
        k_delayed_work_init(&mod->pub->timer, mod_publish);
    }
    
    for (i = 0; i < (int)MESH_ARRAY_SIZE(mod->keys); i++) {
        mod->keys[i] = BT_MESH_KEY_UNUSED;
    }
    
    if (vnd) {
        return;
    }
    
    for (i = 0; i < (int)MESH_ARRAY_SIZE(model_init); i++) {
        ALOGE("%s, model_init[i].id = %d, mod->id = %d", __func__, model_init[i].id, mod->id);
        if (model_init[i].id == mod->id) {
            model_init[i].init(mod, primary);
        }
    }
}

int bt_mesh_comp_register(const struct bt_mesh_comp *comp)
{
   /* There must be at least one element */
    if (!comp->elem_count) {
        return -EINVAL;
    }
    
    dev_comp = comp;

    for (int i = 0; i < (int)dev_comp->elem_count; i++) {
        struct bt_mesh_elem *elem = &dev_comp->elem[i];
    
    	for (int j = 0; j < (int)elem->model_count; j++) {
            struct bt_mesh_model *model = &elem->models[j];
            model->elem = elem;
        }
    
    	for (int k = 0; k < (int)elem->vnd_model_count; k++) {
            struct bt_mesh_model *vnd_model = &elem->vnd_models[k];
            vnd_model->elem = elem;
        }
    }
    
    bt_mesh_model_foreach(mod_init, NULL);
    return 0;
}

void bt_mesh_comp_provision(u16_t addr)
{
    int i;

    dev_primary_addr = addr;

    ALOGE("addr 0x%04x elem_count %u", addr, dev_comp->elem_count);

    for (i = 0; i < (int)dev_comp->elem_count; i++) {
        struct bt_mesh_elem *elem = &dev_comp->elem[i];

        elem->addr = addr++;

        ALOGE("addr 0x%04x mod_count %u vnd_mod_count %u",
               elem->addr, elem->model_count, elem->vnd_model_count);
    }
}

void bt_mesh_comp_unprovision(void)
{
    ALOGE("mesh unprovision complete, %s", __func__);

    dev_primary_addr = BT_MESH_ADDR_UNASSIGNED;

    bt_mesh_model_foreach(mod_init, NULL);
}

u16_t bt_mesh_primary_addr(void)
{
    return dev_primary_addr;
}

u16_t *bt_mesh_model_find_group(struct bt_mesh_model *mod, u16_t addr)
{
    int i;

    for (i = 0; i < (int)MESH_ARRAY_SIZE(mod->groups); i++) {
        if (mod->groups[i] == addr) {
            return &mod->groups[i];
        }
    }

    return NULL;
}

static struct bt_mesh_model *bt_mesh_elem_find_group(struct bt_mesh_elem *elem,
        u16_t group_addr)
{
    struct bt_mesh_model *model;
    u16_t *match;
    int i;

    for (i = 0; i < (int)elem->model_count; i++) {
        model = &elem->models[i];

        match = bt_mesh_model_find_group(model, group_addr);
        if (match) {
            return model;
        }
    }

    for (i = 0; i < (int)elem->vnd_model_count; i++) {
        model = &elem->vnd_models[i];

        match = bt_mesh_model_find_group(model, group_addr);
        if (match) {
            return model;
        }
    }

    return NULL;
}

struct bt_mesh_elem *bt_mesh_elem_find(u16_t addr)
{
    int i;

    for (i = 0; i < (int)dev_comp->elem_count; i++) {
        struct bt_mesh_elem *elem = &dev_comp->elem[i];

        if (BT_MESH_ADDR_IS_GROUP(addr) ||
                BT_MESH_ADDR_IS_VIRTUAL(addr)) {
            if (bt_mesh_elem_find_group(elem, addr)) {
                return elem;
            }
        } else if (elem->addr == addr) {
            return elem;
        }
    }

    return NULL;
}

u8_t bt_mesh_elem_count(void)
{
    return dev_comp->elem_count;
}

static bool model_has_key(struct bt_mesh_model *mod, u16_t key)
{
    int i;

    for (i = 0; i < (int)MESH_ARRAY_SIZE(mod->keys); i++) {
        if (mod->keys[i] == key) {
            return true;
        }
    }

    return false;
}

static const struct bt_mesh_model_op *find_op(struct bt_mesh_model *models,
        u8_t model_count,
        u16_t app_idx, u32_t opcode,
        struct bt_mesh_model **model)
{
    u8_t i;

    for (i = 0; i < model_count; i++) {
        const struct bt_mesh_model_op *op;

        *model = &models[i];

        if (!model_has_key(*model, app_idx)) {
            continue;
        }

        for (op = (*model)->op; op->func; op++) {
            if (op->opcode == opcode) {
                return op;
            }
        }
    }

    *model = NULL;
    return NULL;
}

static int get_opcode(struct net_buf_simple *buf, u32_t *opcode)
{
    switch (buf->data[0] >> 6) {
    case 0x00:
    case 0x01:
        if (buf->data[0] == 0x7f) {
            ALOGE("Ignoring RFU OpCode");
            return -EINVAL;
        }

        *opcode = net_buf_simple_pull_u8(buf);
        return 0;
    case 0x02:
        if (buf->len < 2) {
            ALOGE("Too short payload for 2-octet OpCode");
            return -EINVAL;
        }

        *opcode = net_buf_simple_pull_be16(buf);
        return 0;
    case 0x03:
        if (buf->len < 3) {
            ALOGE("Too short payload for 3-octet OpCode");
            return -EINVAL;
        }

        *opcode = net_buf_simple_pull_u8(buf) << 16;
        *opcode |= net_buf_simple_pull_le16(buf);
        return 0;
    }

    /* Change by Espressif, we don't support the CODE_UNREACHABLE macro in ESP-IDF architecture */
    return 0xffff;
    //CODE_UNREACHABLE;
}

bool bt_mesh_fixed_group_match(u16_t addr)
{
    /* Check for fixed group addresses */
    switch (addr) {
    case BT_MESH_ADDR_ALL_NODES:
        return true;
    case BT_MESH_ADDR_PROXIES:
        /* TODO: Proxy not yet supported */
        return false;
    case BT_MESH_ADDR_FRIENDS:
        return (bt_mesh_friend_get() == BT_MESH_FRIEND_ENABLED);
    case BT_MESH_ADDR_RELAYS:
        return (bt_mesh_relay_get() == BT_MESH_RELAY_ENABLED);
    default:
        return false;
    }
}

u32_t mesh_opcode;

void bt_mesh_model_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf)
{
    struct bt_mesh_model *models, *model;
    const struct bt_mesh_model_op *op;
    u32_t opcode;
    u8_t count;
    int i;

    ALOGE("app_idx 0x%04x src 0x%04x dst 0x%04x", rx->ctx.app_idx,
           rx->ctx.addr, rx->dst);
    ALOGE("len %u: %s", buf->len, bt_hex(buf->data, buf->len));

    if (get_opcode(buf, &opcode) < 0) {
        ALOGE("Unable to decode OpCode");
        return;
    }

    ALOGE("OpCode 0x%08x", opcode);

    mesh_opcode = opcode;
    for (i = 0; i < (int)dev_comp->elem_count; i++) {
        struct bt_mesh_elem *elem = &dev_comp->elem[i];

        if (BT_MESH_ADDR_IS_UNICAST(rx->dst)) {
            if (elem->addr != rx->dst) {
                continue;
            }
        } else if (BT_MESH_ADDR_IS_GROUP(rx->dst) ||
                   BT_MESH_ADDR_IS_VIRTUAL(rx->dst)) {
            if (!bt_mesh_elem_find_group(elem, rx->dst)) {
                continue;
            }
        } else if (i != 0 || !bt_mesh_fixed_group_match(rx->dst)) {
            continue;
        }

        /* SIG models cannot contain 3-byte (vendor) OpCodes, and
         * vendor models cannot contain SIG (1- or 2-byte) OpCodes, so
         * we only need to do the lookup in one of the model lists.
         */
        if (opcode < 0x10000) {
            models = elem->models;
            count = elem->model_count;
        } else {
            models = elem->vnd_models;
            count = elem->vnd_model_count;
        }

        op = find_op(models, count, rx->ctx.app_idx, opcode, &model);
        if (op) {
            struct net_buf_simple_state state;

            if (buf->len < op->min_len) {
                ALOGE("Too short message for OpCode 0x%08x",
                       opcode);
                continue;
            }

            /* The callback will likely parse the buffer, so
             * store the parsing state in case multiple models
             * receive the message.
             */
            net_buf_simple_save(buf, &state);
            /** Change by Espressif, here we update recv_op with the
             *  value opcode got from the buf.
             */
            rx->ctx.recv_op = opcode;
            /** Change by Espressif, Zephyr add 'recv_dst' in ctx in
             *  the lastest master branch, here we just update this
             *  value with rx->dst.
             */
            rx->ctx.recv_dst = rx->dst;
            /** Change by Espressif, we update the model pointer to the
             *  found model when we received a message.
             */
            rx->ctx.model = model;
            /** Change by Espressif, we update the srv_send flag to be
             *  true when we received a message. This flag will be used
             *  when a server model sends a status message, and will
             *  have no impact on the client sent messages.
             */
            rx->ctx.srv_send = true;
            op->func(model, &rx->ctx, buf);
            net_buf_simple_restore(buf, &state);
        } else {
            ALOGE("No OpCode 0x%08x for elem %d", opcode, i);
        }
    }
}

void bt_mesh_model_msg_init(struct net_buf_simple *msg, u32_t opcode)
{
    net_buf_simple_init(msg, 0);

    if (opcode < 0x100) {
        /* 1-byte OpCode */
        net_buf_simple_add_u8(msg, opcode);
        return;
    }

    if (opcode < 0x10000) {
        /* 2-byte OpCode */
        net_buf_simple_add_be16(msg, opcode);
        return;
    }

    /* 3-byte OpCode */
    net_buf_simple_add_u8(msg, ((opcode >> 16) & 0xff));
    net_buf_simple_add_le16(msg, opcode & 0xffff);
}

static int model_send(struct bt_mesh_model *model,
                      struct bt_mesh_net_tx *tx, bool implicit_bind,
                      struct net_buf_simple *msg,
                      const struct bt_mesh_send_cb *cb, void *cb_data)
{
    ALOGE("net_idx 0x%04x app_idx 0x%04x dst 0x%04x", tx->ctx->net_idx,
           tx->ctx->app_idx, tx->ctx->addr);
    ALOGE("len %u: %s", msg->len, bt_hex(msg->data, msg->len));

    bool check = false;
    u8_t role;

    role = bt_mesh_get_msg_role(model, tx->ctx->srv_send);
    if (role == ROLE_NVAL) {
        ALOGE("%s: get role fail", __func__);
        return -EINVAL;
    }

#if CONFIG_BT_MESH_NODE && !CONFIG_BT_MESH_PROVISIONER
    if (role == NODE) {
        if (!bt_mesh_is_provisioned()) {
            ALOGE("%s: Local node is not yet provisioned", __func__);
            return -EAGAIN;
        }
        if (!bt_mesh_is_provisioner_en()) {
            check = true;
        }
    }
#endif

#if !CONFIG_BT_MESH_NODE && CONFIG_BT_MESH_PROVISIONER
    if (role == PROVISIONER) {
        if (!provisioner_check_msg_dst_addr(tx->ctx->addr)) {
            ALOGE("%s: check msg dst_addr fail", __func__);
            return -EINVAL;
        }
        if (bt_mesh_is_provisioner_en()) {
            check = true;
        }
    }
#endif

#if CONFIG_BT_MESH_NODE && CONFIG_BT_MESH_PROVISIONER
    if (role == PROVISIONER) {
        if (!provisioner_check_msg_dst_addr(tx->ctx->addr)) {
            ALOGE("%s: check msg dst_addr fail", __func__);
            return -EINVAL;
        }
        if (bt_mesh_is_provisioner_en()) {
            check = true;
        }
    } else {
        if (!bt_mesh_is_provisioned()) {
            ALOGE("Local node is not yet provisioned");
            return -EAGAIN;
        }
        check = true;
    }
#endif

    if (!check) {
        ALOGE("%s: check fail", __func__);
        return -EINVAL;
    }

    if (net_buf_simple_tailroom(msg) < 4) {
        ALOGE("Not enough tailroom for TransMIC");
        return -EINVAL;
    }

    if (msg->len > min(BT_MESH_TX_SDU_MAX, BT_MESH_SDU_MAX_LEN) - 4) {
        ALOGE("Too big message");
        return -EMSGSIZE;
    }

    if (!implicit_bind && !model_has_key(model, tx->ctx->app_idx)) {
        ALOGE("Model not bound to AppKey 0x%04x", tx->ctx->app_idx);
        return -EINVAL;
    }

    return bt_mesh_trans_send(tx, msg, cb, cb_data);
}

int bt_mesh_model_send(struct bt_mesh_model *model,
                       struct bt_mesh_msg_ctx *ctx,
                       struct net_buf_simple *msg,
                       const struct bt_mesh_send_cb *cb, void *cb_data)
{
    struct bt_mesh_subnet *sub = NULL;
    u8_t role;

    role = bt_mesh_get_msg_role(model, ctx->srv_send);
    if (role == ROLE_NVAL) {
        ALOGE("%s: get role fail", __func__);
        return -EINVAL;
    }

#if CONFIG_BT_MESH_NODE && !CONFIG_BT_MESH_PROVISIONER
    if (role == NODE) {
        if (!bt_mesh_is_provisioner_en()) {
            sub = bt_mesh_subnet_get(ctx->net_idx);
        }
    }
#endif

#if !CONFIG_BT_MESH_NODE && CONFIG_BT_MESH_PROVISIONER
    if (role == PROVISIONER) {
        if (bt_mesh_is_provisioner_en()) {
            sub = provisioner_subnet_get(ctx->net_idx);
        }
    }
#endif

#if CONFIG_BT_MESH_NODE && CONFIG_BT_MESH_PROVISIONER
    if (role == NODE) {
        sub = bt_mesh_subnet_get(ctx->net_idx);
    } else if (role == PROVISIONER) {
        if (bt_mesh_is_provisioner_en()) {
            sub = provisioner_subnet_get(ctx->net_idx);
        }
    }
#endif

    if (!sub) {
        ALOGE("%s: get sub fail", __func__);
        return -EINVAL;
    }

    ctx->model = model;

    struct bt_mesh_net_tx tx = {
        .sub  = sub,
        .ctx  = ctx,
        .src  = model->elem->addr,
        .xmit = bt_mesh_net_transmit_get(),
        .friend_cred = 0,
    };

    return model_send(model, &tx, false, msg, cb, cb_data);
}

int bt_mesh_model_publish(struct bt_mesh_model *model)
{
    struct net_buf_simple *sdu = NET_BUF_SIMPLE(min(BT_MESH_TX_SDU_MAX, BT_MESH_SDU_MAX_LEN));
    struct bt_mesh_model_pub *pub = model->pub;
    struct bt_mesh_app_key *key = NULL;
    struct bt_mesh_msg_ctx ctx = {0};
    struct bt_mesh_net_tx tx = {
        .sub = NULL,
        .ctx = &ctx,
        .src = model->elem->addr,
        .xmit = bt_mesh_net_transmit_get(),
    };
    int err;

    ALOGE("%s", __func__);

    if (!pub) {
        return -ENOTSUP;
    }

    if (pub->addr == BT_MESH_ADDR_UNASSIGNED) {
        return -EADDRNOTAVAIL;
    }

#if CONFIG_BT_MESH_NODE && !CONFIG_BT_MESH_PROVISIONER
    if (pub->dev_role == NODE) {
        if (bt_mesh_is_provisioned()) {
            key = bt_mesh_app_key_find(pub->key);
        }
    }
#endif

#if !CONFIG_BT_MESH_NODE && CONFIG_BT_MESH_PROVISIONER
    if (pub->dev_role == PROVISIONER) {
        if (bt_mesh_is_provisioner_en()) {
            key = provisioner_app_key_find(pub->key);
        }
    }
#endif

#if CONFIG_BT_MESH_NODE && CONFIG_BT_MESH_PROVISIONER
    if (pub->dev_role == NODE) {
        if (bt_mesh_is_provisioned()) {
            key = bt_mesh_app_key_find(pub->key);
        }
    } else if (pub->dev_role == PROVISIONER) {
        if (bt_mesh_is_provisioner_en()) {
            key = provisioner_app_key_find(pub->key);
        }
    }
#endif

    if (!key) {
        ALOGE("%s: get app key fail", __func__);
        return -EADDRNOTAVAIL;
    }

    if (pub->msg->len + 4 > min(BT_MESH_TX_SDU_MAX, BT_MESH_SDU_MAX_LEN)) {
        ALOGE("Message does not fit maximum SDU size");
        return -EMSGSIZE;
    }

    if (pub->count) {
        ALOGE("Clearing publish retransmit timer");
        k_delayed_work_cancel(&pub->timer);
    }

    net_buf_simple_init(sdu, 0);
    net_buf_simple_add_mem(sdu, pub->msg->data, pub->msg->len);

    ctx.addr     = pub->addr;
    ctx.send_ttl = pub->ttl;
    ctx.net_idx  = key->net_idx;
    ctx.app_idx  = key->app_idx;
    ctx.srv_send = pub->dev_role == NODE ? true : false;

    tx.friend_cred = pub->cred;

#if CONFIG_BT_MESH_NODE && !CONFIG_BT_MESH_PROVISIONER
    if (pub->dev_role == NODE) {
        if (bt_mesh_is_provisioned()) {
            tx.sub = bt_mesh_subnet_get(ctx.net_idx);
        }
    }
#endif

#if !CONFIG_BT_MESH_NODE && CONFIG_BT_MESH_PROVISIONER
    if (pub->dev_role == PROVISIONER) {
        if (bt_mesh_is_provisioner_en()) {
            tx.sub = provisioner_subnet_get(ctx.net_idx);
        }
    }
#endif

#if CONFIG_BT_MESH_NODE && CONFIG_BT_MESH_PROVISIONER
    if (pub->dev_role == NODE) {
        if (bt_mesh_is_provisioned()) {
            tx.sub = bt_mesh_subnet_get(ctx.net_idx);
        }
    } else if (pub->dev_role == PROVISIONER) {
        if (bt_mesh_is_provisioner_en()) {
            tx.sub = provisioner_subnet_get(ctx.net_idx);
        }
    }
#endif

    if (!tx.sub) {
        ALOGE("%s: Get subnet failed", __func__);
        return -EADDRNOTAVAIL;
    }

    pub->count = BT_MESH_PUB_TRANSMIT_COUNT(pub->retransmit);

    ALOGE("Publish Retransmit Count %u Interval %ums", pub->count,
           BT_MESH_PUB_TRANSMIT_INT(pub->retransmit));

    err = model_send(model, &tx, true, sdu, &pub_sent_cb, model);
    if (err) {
        pub->count = 0;
        return err;
    }

    return 0;
}

struct bt_mesh_model *bt_mesh_model_find_vnd(struct bt_mesh_elem *elem,
        u16_t company, u16_t id)
{
    u8_t i;

    for (i = 0; i < elem->vnd_model_count; i++) {
        if (elem->vnd_models[i].vnd.company == company &&
                elem->vnd_models[i].vnd.id == id) {
            return &elem->vnd_models[i];
        }
    }

    return NULL;
}

struct bt_mesh_model *bt_mesh_model_find(struct bt_mesh_elem *elem,
        u16_t id)
{
    u8_t i;

    for (i = 0; i < elem->model_count; i++) {
        if (elem->models[i].id == id) {
            return &elem->models[i];
        }
    }

    return NULL;
}

const struct bt_mesh_comp *bt_mesh_comp_get(void)
{
    return dev_comp;
}
#endif /* CONFIG_BT_MESH */
