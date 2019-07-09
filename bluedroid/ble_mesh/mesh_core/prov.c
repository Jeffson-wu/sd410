/*  Bluetooth Mesh */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_TAG "prov"
#include <utils/Log.h>

#include <errno.h>
#include <string.h>

#include "mesh_util.h"

#if CONFIG_BT_MESH

#include "mesh_main.h"
#include "mesh_bt_uuid.h"

#define ALOGE_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_PROV)

#include "crypto.h"
#include "adv.h"
#include "mesh.h"
#include "net.h"
#include "access.h"
#include "foundation.h"
#include "proxy.h"
#include "prov.h"

#if CONFIG_BT_MESH_NODE

/* 3 transmissions, 20ms interval */
#define PROV_XMIT_COUNT        2
#define PROV_XMIT_INT          20

#define AUTH_METHOD_NO_OOB     0x00
#define AUTH_METHOD_STATIC     0x01
#define AUTH_METHOD_OUTPUT     0x02
#define AUTH_METHOD_INPUT      0x03

#define OUTPUT_OOB_BLINK       0x00
#define OUTPUT_OOB_BEEP        0x01
#define OUTPUT_OOB_VIBRATE     0x02
#define OUTPUT_OOB_NUMBER      0x03
#define OUTPUT_OOB_STRING      0x04

#define INPUT_OOB_PUSH         0x00
#define INPUT_OOB_TWIST        0x01
#define INPUT_OOB_NUMBER       0x02
#define INPUT_OOB_STRING       0x03

#define PROV_ERR_NONE          0x00
#define PROV_ERR_NVAL_PDU      0x01
#define PROV_ERR_NVAL_FMT      0x02
#define PROV_ERR_UNEXP_PDU     0x03
#define PROV_ERR_CFM_FAILED    0x04
#define PROV_ERR_RESOURCES     0x05
#define PROV_ERR_DECRYPT       0x06
#define PROV_ERR_UNEXP_ERR     0x07
#define PROV_ERR_ADDR          0x08

#define PROV_INVITE            0x00
#define PROV_CAPABILITIES      0x01
#define PROV_START             0x02
#define PROV_PUB_KEY           0x03
#define PROV_INPUT_COMPLETE    0x04
#define PROV_CONFIRM           0x05
#define PROV_RANDOM            0x06
#define PROV_DATA              0x07
#define PROV_COMPLETE          0x08
#define PROV_FAILED            0x09

#define PROV_ALG_P256          0x00

#define GPCF(gpc)              (gpc & 0x03)
#define GPC_START(last_seg)    (((last_seg) << 2) | 0x00)
#define GPC_ACK                0x01
#define GPC_CONT(seg_id)       (((seg_id) << 2) | 0x02)
#define GPC_CTL(op)            (((op) << 2) | 0x03)

#define START_PAYLOAD_MAX      20
#define CONT_PAYLOAD_MAX       23

#define START_LAST_SEG(gpc)    (gpc >> 2)
#define CONT_SEG_INDEX(gpc)    (gpc >> 2)

#define BEARER_CTL(gpc)        (gpc >> 2)
#define LINK_OPEN              0x00
#define LINK_ACK               0x01
#define LINK_CLOSE             0x02

#define CLOSE_REASON_SUCCESS   0x00
#define CLOSE_REASON_TIMEOUT   0x01
#define CLOSE_REASON_FAILED    0x02

#define XACT_SEG_DATA(_seg) (&link1.rx.buf->data[20 + ((_seg - 1) * 23)])
#define XACT_SEG_RECV(_seg) (link1.rx.seg &= ~(1 << (_seg)))

#define XACT_NVAL              0xff

enum {
    REMOTE_PUB_KEY,        /* Remote key has been received */
    LOCAL_PUB_KEY,         /* Local public key is available */
    LINK_ACTIVE,           /* Link has been opened */
    HAVE_DHKEY,            /* DHKey has been calcualted */
    SEND_CONFIRM,          /* Waiting to send Confirm value */
    WAIT_NUMBER,           /* Waiting for number input from user */
    WAIT_STRING,           /* Waiting for string input from user */
    TIMEOUT_START,         /* Provision timeout timer has started */

    NUM_FLAGS,
};

struct prov_link1 {
    ATOMIC_DEFINE(flags, NUM_FLAGS);
#if defined(CONFIG_BT_MESH_PB_GATT)
    struct bt_conn *conn;    /* GATT connection */
#endif
    u8_t  dhkey[32];         /* Calculated DHKey */
    u8_t  expect;            /* Next expected PDU */

    u8_t  oob_method;
    u8_t  oob_action;
    u8_t  oob_size;

    u8_t  conf[16];          /* Remote Confirmation */
    u8_t  rand[16];          /* Local Random */
    u8_t  auth[16];          /* Authentication Value */

    u8_t  conf_salt[16];     /* ConfirmationSalt */
    u8_t  conf_key[16];      /* ConfirmationKey */
    u8_t  conf_inputs[145];  /* ConfirmationInputs */
    u8_t  prov_salt[16];     /* Provisioning Salt */

#if defined(CONFIG_BT_MESH_PB_ADV)
    u32_t id;                /* Link ID */

    struct {
        u8_t  id;        /* Transaction ID */
        u8_t  prev_id;   /* Previous Transaction ID */
        u8_t  seg;       /* Bit-field of unreceived segments */
        u8_t  last_seg;  /* Last segment (to check length) */
        u8_t  fcs;       /* Expected FCS value */
        struct net_buf_simple *buf;
    } rx;

    struct {
        /* Start timestamp of the transaction */
        s64_t start;

        /* Transaction id*/
        u8_t id;

        /* Pending outgoing buffer(s) */
        struct net_buf *buf[3];

        /* Retransmit timer */
        struct k_delayed_work retransmit;
    } tx;
#endif

    /* Provision timeout timer */
    struct k_delayed_work timeout;
};

struct prov_rx {
    u32_t link1_id;
    u8_t  xact_id;
    u8_t  gpc;
};

#define RETRANSMIT_TIMEOUT   K_MSEC(500)
#define BUF_TIMEOUT          K_MSEC(400)
#define TRANSACTION_TIMEOUT  K_SECONDS(30)
#define PROVISION_TIMEOUT    K_SECONDS(60)

#if defined(CONFIG_BT_MESH_PB_GATT)
#define PROV_BUF_HEADROOM 5
#else
#define PROV_BUF_HEADROOM 0
static struct net_buf_simple *rx_buf = NET_BUF_SIMPLE(65);
#endif

#define PROV_BUF(len) NET_BUF_SIMPLE(PROV_BUF_HEADROOM + len)

static struct prov_link1 link1;

static const struct bt_mesh_prov *prov;

static void close_link1(u8_t err, u8_t reason);

#if defined(CONFIG_BT_MESH_PB_ADV)
static void buf_sent(int err, void *user_data)
{
    if (!link1.tx.buf[0]) {
        return;
    }

    k_delayed_work_submit(&link1.tx.retransmit, RETRANSMIT_TIMEOUT);
}

static struct bt_mesh_send_cb buf_sent_cb = {
    .end = buf_sent,
};

static void free_segments(void)
{
    int i;

    for (i = 0; i < MESH_ARRAY_SIZE(link1.tx.buf); i++) {
        struct net_buf *buf = link1.tx.buf[i];

        if (!buf) {
            break;
        }

        link1.tx.buf[i] = NULL;
        /* Mark as canceled */
        BT_MESH_ADV(buf)->busy = 0;
        /** Change by Espressif. Add this to avoid buf->ref is 2 which will
         *  cause lack of buf.
         */
        if (buf->ref > 1) {
            buf->ref = 1;
        }
        net_buf_unref(buf);
    }
}

static void prov_clear_tx(void)
{
    ALOGE("%s", __func__);

    k_delayed_work_cancel(&link1.tx.retransmit);

    free_segments();
}

static void reset_link1(void)
{
    prov_clear_tx();

    /* Disable Attention Timer if it was set */
    if (link1.conf_inputs[0]) {
        bt_mesh_attention(NULL, 0);
    }

    if (prov->link_close) {
        prov->link_close(BT_MESH_PROV_ADV);
    }

    /* Clear everything except the retransmit delayed work config */
    memset(&link1, 0, offsetof(struct prov_link1, tx.retransmit));

    link1.rx.prev_id = XACT_NVAL;

    if (bt_pub_key_get()) {
        atomic_set_bit(link1.flags, LOCAL_PUB_KEY);
    }

#if defined(CONFIG_BT_MESH_PB_GATT)
    link1.rx.buf = bt_mesh_proxy_get_buf();
#else
    net_buf_simple_init(rx_buf, 0);
    link1.rx.buf = rx_buf;
#endif
}

static struct net_buf *adv_buf_create(void)
{
    struct net_buf *buf;

    buf = bt_mesh_adv_create(BT_MESH_ADV_PROV, PROV_XMIT_COUNT,
                             PROV_XMIT_INT, BUF_TIMEOUT);
    if (!buf) {
        ALOGE("Out of provisioning buffers");
        return NULL;
    }

    return buf;
}

static u8_t pending_ack = XACT_NVAL;

static void ack_complete(u16_t duration, int err, void *user_data)
{
    ALOGE("xact %u complete", (u8_t)pending_ack);
    pending_ack = XACT_NVAL;
}

static void gen_prov_ack_send(u8_t xact_id)
{
    static const struct bt_mesh_send_cb cb = {
        .start = ack_complete,
    };
    const struct bt_mesh_send_cb *complete;
    struct net_buf *buf;

    ALOGE("xact_id %u", xact_id);

    if (pending_ack == xact_id) {
        ALOGE("Not sending duplicate ack");
        return;
    }

    buf = adv_buf_create();
    if (!buf) {
        return;
    }

    if (pending_ack == XACT_NVAL) {
        pending_ack = xact_id;
        complete = &cb;
    } else {
        complete = NULL;
    }

    net_buf_add_be32(buf, link1.id);
    net_buf_add_u8(buf, xact_id);
    net_buf_add_u8(buf, GPC_ACK);

    bt_mesh_adv_send(buf, complete, NULL);
    net_buf_unref(buf);
}

static void send_reliable(void)
{
    int i;

    link1.tx.start = k_uptime_get();

    for (i = 0; i < MESH_ARRAY_SIZE(link1.tx.buf); i++) {
        struct net_buf *buf = link1.tx.buf[i];

        if (!buf) {
            break;
        }

        if (i + 1 < MESH_ARRAY_SIZE(link1.tx.buf) && link1.tx.buf[i + 1]) {
            bt_mesh_adv_send(buf, NULL, NULL);
        } else {
            bt_mesh_adv_send(buf, &buf_sent_cb, NULL);
        }
    }
}

static int bearer_ctl_send(u8_t op, void *data, u8_t data_len)
{
    struct net_buf *buf;

    ALOGE("op 0x%02x data_len %u", op, data_len);

    prov_clear_tx();

    buf = adv_buf_create();
    if (!buf) {
        return -ENOBUFS;
    }

    net_buf_add_be32(buf, link1.id);
    /* Transaction ID, always 0 for Bearer messages */
    net_buf_add_u8(buf, 0x00);
    net_buf_add_u8(buf, GPC_CTL(op));
    net_buf_add_mem(buf, data, data_len);

    link1.tx.buf[0] = buf;
    send_reliable();

    return 0;
}

static u8_t last_seg(u8_t len)
{
    if (len <= START_PAYLOAD_MAX) {
        return 0;
    }

    len -= START_PAYLOAD_MAX;

    return 1 + (len / CONT_PAYLOAD_MAX);
}

static inline u8_t next_transaction_id(void)
{
    if (link1.tx.id != 0 && link1.tx.id != 0xFF) {
        return ++link1.tx.id;
    }

    link1.tx.id = 0x80;
    return link1.tx.id;
}

static int prov_send_adv(struct net_buf_simple *msg)
{
    struct net_buf *start, *buf;
    u8_t seg_len, seg_id;
    u8_t xact_id;
    u8_t type;

    ALOGE("%s, len %u: %s", __func__, msg->len, bt_hex(msg->data, msg->len));

    prov_clear_tx();

    start = adv_buf_create();
    if (!start) {
        return -ENOBUFS;
    }

    xact_id = next_transaction_id();
    net_buf_add_be32(start, link1.id);
    net_buf_add_u8(start, xact_id);

    net_buf_add_u8(start, GPC_START(last_seg(msg->len)));
    net_buf_add_be16(start, msg->len);
    net_buf_add_u8(start, bt_mesh_fcs_calc(msg->data, msg->len));

    link1.tx.buf[0] = start;
    /* Change by Espressif, get message type */
    type = msg->data[0];

    seg_len = min(msg->len, START_PAYLOAD_MAX);
    ALOGE("seg 0 len %u: %s", seg_len, bt_hex(msg->data, seg_len));
    net_buf_add_mem(start, msg->data, seg_len);
    net_buf_simple_pull(msg, seg_len);

    buf = start;
    for (seg_id = 1; msg->len > 0; seg_id++) {
        if (seg_id >= MESH_ARRAY_SIZE(link1.tx.buf)) {
            ALOGE("Too big message");
            free_segments();
            return -E2BIG;
        }

        buf = adv_buf_create();
        if (!buf) {
            free_segments();
            return -ENOBUFS;
        }

        link1.tx.buf[seg_id] = buf;

        seg_len = min(msg->len, CONT_PAYLOAD_MAX);

        ALOGE("seg_id %u len %u: %s", seg_id, seg_len,
               bt_hex(msg->data, seg_len));

        net_buf_add_be32(buf, link1.id);
        net_buf_add_u8(buf, xact_id);
        net_buf_add_u8(buf, GPC_CONT(seg_id));
        net_buf_add_mem(buf, msg->data, seg_len);
        net_buf_simple_pull(msg, seg_len);
    }

    send_reliable();

    /* Change by Espressif, add provisioning timeout timer operations.
     * When sending a provisioning PDU successfully, restart the 60s timer.
     */
    if (atomic_test_and_clear_bit(link1.flags, TIMEOUT_START)) {
        k_delayed_work_cancel(&link1.timeout);
    }
    if (type != PROV_COMPLETE && type != PROV_FAILED) {
        if (!atomic_test_and_set_bit(link1.flags, TIMEOUT_START)) {
            k_delayed_work_submit(&link1.timeout, PROVISION_TIMEOUT);
        }
    }

    return 0;
}

#endif /* CONFIG_BT_MESH_PB_ADV */

#if defined(CONFIG_BT_MESH_PB_GATT)
static int prov_send_gatt(struct net_buf_simple *msg)
{
    int err = 0;

    if (!link1.conn) {
        return -ENOTCONN;
    }

    /* Change by Espressif, add provisioning timeout timer operations.
     * When sending a provisioning PDU successfully, restart the 60s timer.
     */
    err = bt_mesh_proxy_send(link1.conn, BT_MESH_PROXY_PROV, msg);
    if (err) {
        ALOGE("%s: fail to send provisioning PDU", __func__);
        return err;
    }

    if (atomic_test_and_clear_bit(link1.flags, TIMEOUT_START)) {
        k_delayed_work_cancel(&link1.timeout);
    }
    if (msg->data[1] != PROV_COMPLETE && msg->data[1] != PROV_FAILED) {
        if (!atomic_test_and_set_bit(link1.flags, TIMEOUT_START)) {
            k_delayed_work_submit(&link1.timeout, PROVISION_TIMEOUT);
        }
    }

    return 0;
}
#endif /* CONFIG_BT_MESH_PB_GATT */

static inline int prov_send(struct net_buf_simple *buf)
{
#if defined(CONFIG_BT_MESH_PB_GATT)
    if (link1.conn) {
        return prov_send_gatt(buf);
    }
#endif
#if defined(CONFIG_BT_MESH_PB_ADV)
    return prov_send_adv(buf);
#else
    return 0;
#endif
}

static void prov_buf_init(struct net_buf_simple *buf, u8_t type)
{
    net_buf_simple_init(buf, PROV_BUF_HEADROOM);
    net_buf_simple_add_u8(buf, type);
}

static void prov_send_fail_msg(u8_t err)
{
    struct net_buf_simple *buf = PROV_BUF(2);

    prov_buf_init(buf, PROV_FAILED);
    net_buf_simple_add_u8(buf, err);
    prov_send(buf);
}

static void prov_invite(const u8_t *data)
{
    struct net_buf_simple *buf = PROV_BUF(12);

    ALOGE("Attention Duration: %u seconds", data[0]);

    if (data[0]) {
        bt_mesh_attention(NULL, data[0]);
    }

    link1.conf_inputs[0] = data[0];

    prov_buf_init(buf, PROV_CAPABILITIES);

    /* Number of Elements supported */
    net_buf_simple_add_u8(buf, bt_mesh_elem_count());

    /* Supported algorithms - FIPS P-256 Eliptic Curve */
    net_buf_simple_add_be16(buf, BIT(PROV_ALG_P256));

    /* Public Key Type */
    net_buf_simple_add_u8(buf, 0x00);

    /* Static OOB Type */
    net_buf_simple_add_u8(buf, prov->static_val ? BIT(0) : 0x00);

    /* Output OOB Size */
    net_buf_simple_add_u8(buf, prov->output_size);

    /* Output OOB Action */
    net_buf_simple_add_be16(buf, prov->output_actions);

    /* Input OOB Size */
    net_buf_simple_add_u8(buf, prov->input_size);

    /* Input OOB Action */
    net_buf_simple_add_be16(buf, prov->input_actions);

    memcpy(&link1.conf_inputs[1], &buf->data[1], 11);

    if (prov_send(buf)) {
        ALOGE("Failed to send capabilities");
        close_link1(PROV_ERR_RESOURCES, CLOSE_REASON_FAILED);
        return;
    }

    link1.expect = PROV_START;
}

static void prov_capabilities(const u8_t *data)
{
    u16_t algorithms, output_action, input_action;

    ALOGE("Elements: %u", data[0]);

    algorithms = sys_get_be16(&data[1]);
    ALOGE("Algorithms:        %u", algorithms);

    ALOGE("Public Key Type:   0x%02x", data[3]);
    ALOGE("Static OOB Type:   0x%02x", data[4]);
    ALOGE("Output OOB Size:   %u", data[5]);

    output_action = sys_get_be16(&data[6]);
    ALOGE("Output OOB Action: 0x%04x", output_action);

    ALOGE("Input OOB Size:    %u", data[8]);

    input_action = sys_get_be16(&data[9]);
    ALOGE("Input OOB Action:  0x%04x", input_action);
}

static bt_mesh_output_action_t output_action(u8_t action)
{
    switch (action) {
    case OUTPUT_OOB_BLINK:
        return BT_MESH_BLINK;
    case OUTPUT_OOB_BEEP:
        return BT_MESH_BEEP;
    case OUTPUT_OOB_VIBRATE:
        return BT_MESH_VIBRATE;
    case OUTPUT_OOB_NUMBER:
        return BT_MESH_DISPLAY_NUMBER;
    case OUTPUT_OOB_STRING:
        return BT_MESH_DISPLAY_STRING;
    default:
        return BT_MESH_NO_OUTPUT;
    }
}

static bt_mesh_input_action_t input_action(u8_t action)
{
    switch (action) {
    case INPUT_OOB_PUSH:
        return BT_MESH_PUSH;
    case INPUT_OOB_TWIST:
        return BT_MESH_TWIST;
    case INPUT_OOB_NUMBER:
        return BT_MESH_ENTER_NUMBER;
    case INPUT_OOB_STRING:
        return BT_MESH_ENTER_STRING;
    default:
        return BT_MESH_NO_INPUT;
    }
}

static int prov_auth(u8_t method, u8_t action, u8_t size)
{
    bt_mesh_output_action_t output;
    bt_mesh_input_action_t input;

    switch (method) {
    case AUTH_METHOD_NO_OOB:
        if (action || size) {
            return -EINVAL;
        }

        memset(link1.auth, 0, sizeof(link1.auth));
        return 0;
    case AUTH_METHOD_STATIC:
        if (action || size) {
            return -EINVAL;
        }

        memcpy(link1.auth + 16 - prov->static_val_len,
               prov->static_val, prov->static_val_len);
        memset(link1.auth, 0, sizeof(link1.auth) - prov->static_val_len);
        return 0;

    case AUTH_METHOD_OUTPUT:
        output = output_action(action);
        if (!output) {
            return -EINVAL;
        }

        if (!(prov->output_actions & output)) {
            return -EINVAL;
        }

        if (size > prov->output_size) {
            return -EINVAL;
        }

        if (output == BT_MESH_DISPLAY_STRING) {
            unsigned char str[9];
            u8_t i;

            bt_rand(str, size);

            /* Normalize to '0' .. '9' & 'A' .. 'Z' */
            for (i = 0; i < size; i++) {
                str[i] %= 36;
                if (str[i] < 10) {
                    str[i] += '0';
                } else {
                    str[i] += 'A' - 10;
                }
            }
            str[size] = '\0';

            memcpy(link1.auth, str, size);
            memset(link1.auth + size, 0, sizeof(link1.auth) - size);

            return prov->output_string((char *)str);
        } else {
            u32_t div[8] = { 10, 100, 1000, 10000, 100000,
                             1000000, 10000000, 100000000
                           };
            u32_t num;

            bt_rand(&num, sizeof(num));
            num %= div[size - 1];

            sys_put_be32(num, &link1.auth[12]);
            memset(link1.auth, 0, 12);

            return prov->output_number(output, num);
        }

    case AUTH_METHOD_INPUT:
        input = input_action(action);
        if (!input) {
            return -EINVAL;
        }

        if (!(prov->input_actions & input)) {
            return -EINVAL;
        }

        if (size > prov->input_size) {
            return -EINVAL;
        }

        if (input == BT_MESH_ENTER_STRING) {
            atomic_set_bit(link1.flags, WAIT_STRING);
        } else {
            atomic_set_bit(link1.flags, WAIT_NUMBER);
        }

        return prov->input(input, size);

    default:
        return -EINVAL;
    }
}

static void prov_start(const u8_t *data)
{
    ALOGE("Algorithm:   0x%02x", data[0]);
    ALOGE("Public Key:  0x%02x", data[1]);
    ALOGE("Auth Method: 0x%02x", data[2]);
    ALOGE("Auth Action: 0x%02x", data[3]);
    ALOGE("Auth Size:   0x%02x", data[4]);

    if (data[0] != PROV_ALG_P256) {
        ALOGE("Unknown algorithm 0x%02x", data[0]);
        prov_send_fail_msg(PROV_ERR_NVAL_FMT);
        return;
    }

    if (data[1] > 0x01) {
        ALOGE("Invalid public key value: 0x%02x", data[1]);
        prov_send_fail_msg(PROV_ERR_NVAL_FMT);
        return;
    }

    memcpy(&link1.conf_inputs[12], data, 5);

    link1.expect = PROV_PUB_KEY;

    if (prov_auth(data[2], data[3], data[4]) < 0) {
        ALOGE("Invalid authentication method: 0x%02x; "
               "action: 0x%02x; size: 0x%02x", data[2], data[3],
               data[4]);
        prov_send_fail_msg(PROV_ERR_NVAL_FMT);
    }
}

static void send_confirm(void)
{
    struct net_buf_simple *cfm = PROV_BUF(17);

    ALOGE("ConfInputs[0]   %s", bt_hex(link1.conf_inputs, 64));
    ALOGE("ConfInputs[64]  %s", bt_hex(&link1.conf_inputs[64], 64));
    ALOGE("ConfInputs[128] %s", bt_hex(&link1.conf_inputs[128], 17));

    if (bt_mesh_prov_conf_salt(link1.conf_inputs, link1.conf_salt)) {
        ALOGE("Unable to generate confirmation salt");
        close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_FAILED);
        return;
    }

    ALOGE("ConfirmationSalt: %s", bt_hex(link1.conf_salt, 16));

    if (bt_mesh_prov_conf_key(link1.dhkey, link1.conf_salt, link1.conf_key)) {
        ALOGE("Unable to generate confirmation key");
        close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_FAILED);
        return;
    }

    ALOGE("ConfirmationKey: %s", bt_hex(link1.conf_key, 16));

    if (bt_rand(link1.rand, 16)) {
        ALOGE("Unable to generate random number");
        close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_FAILED);
        return;
    }

    ALOGE("LocalRandom: %s", bt_hex(link1.rand, 16));

    prov_buf_init(cfm, PROV_CONFIRM);

    if (bt_mesh_prov_conf(link1.conf_key, link1.rand, link1.auth,
                          net_buf_simple_add(cfm, 16))) {
        ALOGE("Unable to generate confirmation value");
        close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_FAILED);
        return;
    }

    if (prov_send(cfm)) {
        ALOGE("Failed to send Provisioning Confirm");
        close_link1(PROV_ERR_RESOURCES, CLOSE_REASON_FAILED);
        return;
    }

    link1.expect = PROV_RANDOM;
}

static void send_input_complete(void)
{
    struct net_buf_simple *buf = PROV_BUF(1);

    prov_buf_init(buf, PROV_INPUT_COMPLETE);
    prov_send(buf);
}

int bt_mesh_input_number(u32_t num)
{
    ALOGE("%u", num);

    if (!atomic_test_and_clear_bit(link1.flags, WAIT_NUMBER)) {
        return -EINVAL;
    }

    sys_put_be32(num, &link1.auth[12]);

    send_input_complete();

    if (!atomic_test_bit(link1.flags, HAVE_DHKEY)) {
        return 0;
    }

    if (atomic_test_and_clear_bit(link1.flags, SEND_CONFIRM)) {
        send_confirm();
    }

    return 0;
}

int bt_mesh_input_string(const char *str)
{
    ALOGE("%s", str);

    if (!atomic_test_and_clear_bit(link1.flags, WAIT_STRING)) {
        return -EINVAL;
    }

    memcpy(link1.auth, str, prov->input_size);

    send_input_complete();

    if (!atomic_test_bit(link1.flags, HAVE_DHKEY)) {
        return 0;
    }

    if (atomic_test_and_clear_bit(link1.flags, SEND_CONFIRM)) {
        send_confirm();
    }

    return 0;
}

static void prov_dh_key_cb(const u8_t key[32])
{
    ALOGE("%p", key);

    if (!key) {
        ALOGE("DHKey generation failed");
        close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_FAILED);
        return;
    }

    sys_memcpy_swap(link1.dhkey, key, 32);

    ALOGE("DHkey: %s", bt_hex(link1.dhkey, 32));

    atomic_set_bit(link1.flags, HAVE_DHKEY);

    if (atomic_test_bit(link1.flags, WAIT_NUMBER) ||
            atomic_test_bit(link1.flags, WAIT_STRING)) {
        return;
    }

    if (atomic_test_and_clear_bit(link1.flags, SEND_CONFIRM)) {
        send_confirm();
    }
}

static void send_pub_key(void)
{
    struct net_buf_simple *buf = PROV_BUF(65);
    const u8_t *key = NULL;

    key = bt_pub_key_get();
    if (!key) {
        ALOGE("No public key available");
        close_link1(PROV_ERR_RESOURCES, CLOSE_REASON_FAILED);
        return;
    }

    ALOGE("Local Public Key: %s", bt_hex(key, 64));

    prov_buf_init(buf, PROV_PUB_KEY);

    /* Swap X and Y halves independently to big-endian */
    sys_memcpy_swap(net_buf_simple_add(buf, 32), key, 32);
    sys_memcpy_swap(net_buf_simple_add(buf, 32), &key[32], 32);

    memcpy(&link1.conf_inputs[81], &buf->data[1], 64);

    prov_send(buf);

    /* Copy remote key in little-endian for bt_dh_key_gen().
     * X and Y halves are swapped independently.
     */
    net_buf_simple_init(buf, 0);
    sys_memcpy_swap(buf->data, &link1.conf_inputs[17], 32);
    sys_memcpy_swap(&buf->data[32], &link1.conf_inputs[49], 32);

    if (bt_dh_key_gen(buf->data, prov_dh_key_cb)) {
        ALOGE("Failed to generate DHKey");
        close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_FAILED);
        return;
    }

    link1.expect = PROV_CONFIRM;
}

static void prov_pub_key(const u8_t *data)
{
    ALOGE("Remote Public Key: %s", bt_hex(data, 64));

    memcpy(&link1.conf_inputs[17], data, 64);
    /* TODO: Change by Espressif.
     Temporary modification. We don't support generate the publish key by the controller now */
#if 0
    if (!atomic_test_bit(link1.flags, LOCAL_PUB_KEY)) {
        /* Clear retransmit timer */
#if defined(CONFIG_BT_MESH_PB_ADV)
        prov_clear_tx();
#endif
        atomic_set_bit(link1.flags, REMOTE_PUB_KEY);
        ALOGE("Waiting for local public key");
        return;
    }
#endif /* #if 0 */
    send_pub_key();
}

#if 0
/* Change by Espressif, currently mask this function */
static void pub_key_ready(const u8_t *pkey)
{
    if (!pkey) {
        ALOGE("Public key not available");
        return;
    }

    ALOGE("Local public key ready");

    atomic_set_bit(link1.flags, LOCAL_PUB_KEY);

    if (atomic_test_and_clear_bit(link1.flags, REMOTE_PUB_KEY)) {
        send_pub_key();
    }
}
#endif

static void prov_input_complete(const u8_t *data)
{
    ALOGE("%s", __func__);
}

static void prov_confirm(const u8_t *data)
{
    ALOGE("Remote Confirm: %s", bt_hex(data, 16));

    memcpy(link1.conf, data, 16);

    if (!atomic_test_bit(link1.flags, HAVE_DHKEY)) {
#if defined(CONFIG_BT_MESH_PB_ADV)
        prov_clear_tx();
#endif
        atomic_set_bit(link1.flags, SEND_CONFIRM);
    } else {
        send_confirm();
    }
}

static void prov_random(const u8_t *data)
{
    struct net_buf_simple *rnd = PROV_BUF(17);
    u8_t conf_verify[16];

    ALOGE("Remote Random: %s", bt_hex(data, 16));

    if (bt_mesh_prov_conf(link1.conf_key, data, link1.auth, conf_verify)) {
        ALOGE("Unable to calculate confirmation verification");
        close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_FAILED);
        return;
    }

    if (memcmp(conf_verify, link1.conf, 16)) {
        ALOGE("Invalid confirmation value");
        ALOGE("Received:   %s", bt_hex(link1.conf, 16));
        ALOGE("Calculated: %s",  bt_hex(conf_verify, 16));
        close_link1(PROV_ERR_CFM_FAILED, CLOSE_REASON_FAILED);
        return;
    }

    prov_buf_init(rnd, PROV_RANDOM);
    net_buf_simple_add_mem(rnd, link1.rand, 16);

    if (prov_send(rnd)) {
        ALOGE("Failed to send Provisioning Random");
        close_link1(PROV_ERR_RESOURCES, CLOSE_REASON_FAILED);
        return;
    }

    if (bt_mesh_prov_salt(link1.conf_salt, data, link1.rand,
                          link1.prov_salt)) {
        ALOGE("Failed to generate provisioning salt");
        close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_FAILED);
        return;
    }

    ALOGE("ProvisioningSalt: %s", bt_hex(link1.prov_salt, 16));

    link1.expect = PROV_DATA;
}

static void prov_data(const u8_t *data)
{
    struct net_buf_simple *msg = PROV_BUF(1);
    u8_t session_key[16];
    u8_t nonce[13];
    u8_t dev_key[16];
    u8_t pdu[25];
    u8_t flags;
    u32_t iv_index;
    u16_t addr;
    u16_t net_idx;
    int err;

    ALOGE("%s", __func__);

    err = bt_mesh_session_key(link1.dhkey, link1.prov_salt, session_key);
    if (err) {
        ALOGE("Unable to generate session key");
        close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_FAILED);
        return;
    }

    ALOGE("SessionKey: %s", bt_hex(session_key, 16));

    err = bt_mesh_prov_nonce(link1.dhkey, link1.prov_salt, nonce);
    if (err) {
        ALOGE("Unable to generate session nonce");
        close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_FAILED);
        return;
    }

    ALOGE("Nonce: %s", bt_hex(nonce, 13));

    err = bt_mesh_prov_decrypt(session_key, nonce, data, pdu);
    if (err) {
        ALOGE("Unable to decrypt provisioning data");
        close_link1(PROV_ERR_DECRYPT, CLOSE_REASON_FAILED);
        return;
    }

    err = bt_mesh_dev_key(link1.dhkey, link1.prov_salt, dev_key);
    if (err) {
        ALOGE("Unable to generate device key");
        close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_FAILED);
        return;
    }

    ALOGE("DevKey: %s", bt_hex(dev_key, 16));

    net_idx = sys_get_be16(&pdu[16]);
    flags = pdu[18];
    iv_index = sys_get_be32(&pdu[19]);
    addr = sys_get_be16(&pdu[23]);

    ALOGE("net_idx %u iv_index 0x%08x, addr 0x%04x",
           net_idx, iv_index, addr);

    prov_buf_init(msg, PROV_COMPLETE);
    prov_send(msg);

    /* Ignore any further PDUs on this link1 */
    link1.expect = 0;

    bt_mesh_provision(pdu, net_idx, flags, iv_index, 0, addr, dev_key);
}

static void prov_complete(const u8_t *data)
{
    ALOGE("%s", __func__);
}

static void prov_failed(const u8_t *data)
{
    ALOGE("Error: 0x%02x", data[0]);
}

static const struct {
    void (*func)(const u8_t *data);
    u16_t len;
} prov_handlers[] = {
    { prov_invite, 1 },
    { prov_capabilities, 11 },
    { prov_start, 5, },
    { prov_pub_key, 64 },
    { prov_input_complete, 0 },
    { prov_confirm, 16 },
    { prov_random, 16 },
    { prov_data, 33 },
    { prov_complete, 0 },
    { prov_failed, 1 },
};

static void close_link1(u8_t err, u8_t reason)
{
#if defined(CONFIG_BT_MESH_PB_GATT)
    if (link1.conn) {
        bt_mesh_pb_gatt_close(link1.conn);
        return;
    }
#endif

#if defined(CONFIG_BT_MESH_PB_ADV)
    if (err) {
        prov_send_fail_msg(err);
    }

    link1.rx.seg = 0;
    bearer_ctl_send(LINK_CLOSE, &reason, sizeof(reason));
#endif

    atomic_clear_bit(link1.flags, LINK_ACTIVE);

    /* Disable Attention Timer if it was set */
    if (link1.conf_inputs[0]) {
        bt_mesh_attention(NULL, 0);
    }
}

/* Change by Espressif, add provisioning timeout timer callback */
static void prov_timeout(struct k_work *work)
{
    ALOGE("%s", __func__);

    close_link1(PROV_ERR_UNEXP_ERR, CLOSE_REASON_TIMEOUT);
}

#if defined(CONFIG_BT_MESH_PB_ADV)
static void prov_retransmit(struct k_work *work)
{
    int i;

    ALOGE("%s", __func__);

    if (!atomic_test_bit(link1.flags, LINK_ACTIVE)) {
        ALOGE("Link not active");
        return;
    }

    if (k_uptime_get() - link1.tx.start > TRANSACTION_TIMEOUT) {
        ALOGE("Giving up transaction");
        reset_link1();
        return;
    }

    for (i = 0; i < MESH_ARRAY_SIZE(link1.tx.buf); i++) {
        struct net_buf *buf = link1.tx.buf[i];

        if (!buf) {
            break;
        }

        if (BT_MESH_ADV(buf)->busy) {
            continue;
        }

        ALOGE("%u bytes: %s", buf->len, bt_hex(buf->data, buf->len));

        if (i + 1 < MESH_ARRAY_SIZE(link1.tx.buf) && link1.tx.buf[i + 1]) {
            bt_mesh_adv_send(buf, NULL, NULL);
        } else {
            bt_mesh_adv_send(buf, &buf_sent_cb, NULL);
        }

    }
}

static void link1_open(struct prov_rx *rx, struct net_buf_simple *buf)
{
    ALOGE("len %u", buf->len);

    if (buf->len < 16) {
        ALOGE("Too short bearer open message (len %u)", buf->len);
        return;
    }

    if (atomic_test_bit(link1.flags, LINK_ACTIVE)) {
        ALOGE("Ignoring bearer open: link1 already active");
        return;
    }

    if (memcmp(buf->data, prov->uuid, 16)) {
        ALOGE("Bearer open message not for us");
        return;
    }

    if (prov->link_open) {
        prov->link_open(BT_MESH_PROV_ADV);
    }

    link1.id = rx->link1_id;
    atomic_set_bit(link1.flags, LINK_ACTIVE);
    net_buf_simple_init(link1.rx.buf, 0);

    bearer_ctl_send(LINK_ACK, NULL, 0);

    link1.expect = PROV_INVITE;
}

static void link1_ack(struct prov_rx *rx, struct net_buf_simple *buf)
{
    ALOGE("len %u", buf->len);
}

static void link1_close(struct prov_rx *rx, struct net_buf_simple *buf)
{
    ALOGE("len %u", buf->len);

    reset_link1();
}

static void gen_prov_ctl(struct prov_rx *rx, struct net_buf_simple *buf)
{
    ALOGE("op 0x%02x len %u", BEARER_CTL(rx->gpc), buf->len);

    switch (BEARER_CTL(rx->gpc)) {
    case LINK_OPEN:
        link1_open(rx, buf);
        break;
    case LINK_ACK:
        if (!atomic_test_bit(link1.flags, LINK_ACTIVE)) {
            return;
        }

        link1_ack(rx, buf);
        break;
    case LINK_CLOSE:
        if (!atomic_test_bit(link1.flags, LINK_ACTIVE)) {
            return;
        }

        link1_close(rx, buf);
        break;
    default:
        ALOGE("Unknown bearer opcode: 0x%02x", BEARER_CTL(rx->gpc));
        return;
    }
}

static void prov_msg_recv(void)
{
    u8_t type = link1.rx.buf->data[0];

    ALOGE("type 0x%02x len %u", type, link1.rx.buf->len);

    if (!bt_mesh_fcs_check(link1.rx.buf, link1.rx.fcs)) {
        ALOGE("Incorrect FCS");
        return;
    }

    gen_prov_ack_send(link1.rx.id);
    link1.rx.prev_id = link1.rx.id;
    link1.rx.id = 0;

    if (type != PROV_FAILED && type != link1.expect) {
        ALOGE("Unexpected msg 0x%02x != 0x%02x", type, link1.expect);
        prov_send_fail_msg(PROV_ERR_UNEXP_PDU);
        return;
    }

    if (type >= MESH_ARRAY_SIZE(prov_handlers)) {
        ALOGE("Unknown provisioning PDU type 0x%02x", type);
        close_link1(PROV_ERR_NVAL_PDU, CLOSE_REASON_FAILED);
        return;
    }

    if (1 + prov_handlers[type].len != link1.rx.buf->len) {
        ALOGE("Invalid length %u for type 0x%02x",
               link1.rx.buf->len, type);
        close_link1(PROV_ERR_NVAL_FMT, CLOSE_REASON_FAILED);
        return;
    }

    /* Change by Espressif, add provisioning timeout timer operations.
     * When received a provisioning PDU, restart the 60s timer.
     */
    if (atomic_test_and_clear_bit(link1.flags, TIMEOUT_START)) {
        k_delayed_work_cancel(&link1.timeout);
    }
    if (!atomic_test_and_set_bit(link1.flags, TIMEOUT_START)) {
        k_delayed_work_submit(&link1.timeout, PROVISION_TIMEOUT);
    }

    prov_handlers[type].func(&link1.rx.buf->data[1]);
}

static void gen_prov_cont(struct prov_rx *rx, struct net_buf_simple *buf)
{
    u8_t seg = CONT_SEG_INDEX(rx->gpc);

    ALOGE("len %u, seg_index %u", buf->len, seg);

    if (!link1.rx.seg && link1.rx.prev_id == rx->xact_id) {
        ALOGE("Resending ack");
        gen_prov_ack_send(rx->xact_id);
        return;
    }

    if (rx->xact_id != link1.rx.id) {
        ALOGE("Data for unknown transaction (%u != %u)",
                rx->xact_id, link1.rx.id);
        return;
    }

    if (seg > link1.rx.last_seg) {
        ALOGE("Invalid segment index %u", seg);
        close_link1(PROV_ERR_NVAL_FMT, CLOSE_REASON_FAILED);
        return;
    } else if (seg == link1.rx.last_seg) {
        u8_t expect_len;

        expect_len = (link1.rx.buf->len - 20 -
                      (23 * (link1.rx.last_seg - 1)));
        if (expect_len != buf->len) {
            ALOGE("Incorrect last seg len: %u != %u",
                   expect_len, buf->len);
            close_link1(PROV_ERR_NVAL_FMT, CLOSE_REASON_FAILED);
            return;
        }
    }

    if (!(link1.rx.seg & BIT(seg))) {
        ALOGE("Ignoring already received segment");
        return;
    }

    memcpy(XACT_SEG_DATA(seg), buf->data, buf->len);
    XACT_SEG_RECV(seg);

    if (!link1.rx.seg) {
        prov_msg_recv();
    }
}

static void gen_prov_ack(struct prov_rx *rx, struct net_buf_simple *buf)
{
    ALOGE("len %u", buf->len);

    if (!link1.tx.buf[0]) {
        return;
    }

    if (rx->xact_id == link1.tx.id) {
        prov_clear_tx();
    }
}

static void gen_prov_start(struct prov_rx *rx, struct net_buf_simple *buf)
{
    if (link1.rx.seg) {
        ALOGE("Got Start while there are unreceived segments");
        return;
    }

    if (link1.rx.prev_id == rx->xact_id) {
        ALOGE("Resending ack");
        gen_prov_ack_send(rx->xact_id);
        return;
    }

    link1.rx.buf->len = net_buf_simple_pull_be16(buf);
    link1.rx.id  = rx->xact_id;
    link1.rx.fcs = net_buf_simple_pull_u8(buf);

    ALOGE("len %u last_seg %u total_len %u fcs 0x%02x", buf->len,
           START_LAST_SEG(rx->gpc), link1.rx.buf->len, link1.rx.fcs);

    if (link1.rx.buf->len < 1) {
        ALOGE("Ignoring zero-length provisioning PDU");
        close_link1(PROV_ERR_NVAL_FMT, CLOSE_REASON_FAILED);
        return;
    }

    if (link1.rx.buf->len > link1.rx.buf->size) {
        ALOGE("Too large provisioning PDU (%u bytes)",
               link1.rx.buf->len);
        close_link1(PROV_ERR_NVAL_FMT, CLOSE_REASON_FAILED);
        return;
    }

    if (START_LAST_SEG(rx->gpc) > 0 && link1.rx.buf->len <= 20) {
        ALOGE("Too small total length for multi-segment PDU");
        close_link1(PROV_ERR_NVAL_FMT, CLOSE_REASON_FAILED);
        return;
    }

    link1.rx.seg = (1 << (START_LAST_SEG(rx->gpc) + 1)) - 1;
    link1.rx.last_seg = START_LAST_SEG(rx->gpc);
    memcpy(link1.rx.buf->data, buf->data, buf->len);
    XACT_SEG_RECV(0);

    if (!link1.rx.seg) {
        prov_msg_recv();
    }
}

static const struct {
    void (*const func)(struct prov_rx *rx, struct net_buf_simple *buf);
    const u8_t require_link1;
    const u8_t min_len;
} gen_prov[] = {
    { gen_prov_start, true, 3 },
    { gen_prov_ack, true, 0 },
    { gen_prov_cont, true, 0 },
    { gen_prov_ctl, false, 0 },
};

static void gen_prov_recv(struct prov_rx *rx, struct net_buf_simple *buf)
{
    if (buf->len < gen_prov[GPCF(rx->gpc)].min_len) {
        ALOGE("Too short GPC message type %u", GPCF(rx->gpc));
        return;
    }

    if (!atomic_test_bit(link1.flags, LINK_ACTIVE) &&
            gen_prov[GPCF(rx->gpc)].require_link1) {
        ALOGE("Ignoring message that requires active link1");
        return;
    }

    gen_prov[GPCF(rx->gpc)].func(rx, buf);
}

void bt_mesh_pb_adv_recv(struct net_buf_simple *buf)
{
    struct prov_rx rx;

    if (!bt_prov_active() && bt_mesh_is_provisioned()) {
        ALOGE("Ignoring provisioning PDU - already provisioned");
        return;
    }

    if (buf->len < 6) {
        ALOGE("Too short provisioning packet (len %u)", buf->len);
        return;
    }

    rx.link1_id = net_buf_simple_pull_be32(buf);
    rx.xact_id = net_buf_simple_pull_u8(buf);
    rx.gpc = net_buf_simple_pull_u8(buf);

    ALOGE("link1_id 0x%08x xact_id %u", rx.link1_id, rx.xact_id);

    if (atomic_test_bit(link1.flags, LINK_ACTIVE) && link1.id != rx.link1_id) {
        ALOGE("Ignoring mesh beacon for unknown link1");
        return;
    }

    gen_prov_recv(&rx, buf);
}
#endif /* CONFIG_BT_MESH_PB_ADV */

#if defined(CONFIG_BT_MESH_PB_GATT)
int bt_mesh_pb_gatt_recv(struct bt_conn *conn, struct net_buf_simple *buf)
{
    u8_t type;

    ALOGE("%u bytes: %s", buf->len, bt_hex(buf->data, buf->len));

    if (link1.conn != conn) {
        ALOGE("Data for unexpected connection");
        return -ENOTCONN;
    }

    if (buf->len < 1) {
        ALOGE("Too short provisioning packet (len %u)", buf->len);
        return -EINVAL;
    }

    type = net_buf_simple_pull_u8(buf);
    if (type != PROV_FAILED && type != link1.expect) {
        ALOGE("Unexpected msg 0x%02x != 0x%02x", type, link1.expect);
        return -EINVAL;
    }

    if (type >= MESH_ARRAY_SIZE(prov_handlers)) {
        ALOGE("Unknown provisioning PDU type 0x%02x", type);
        return -EINVAL;
    }

    if (prov_handlers[type].len != buf->len) {
        ALOGE("Invalid length %u for type 0x%02x", buf->len, type);
        return -EINVAL;
    }

    /* Change by Espressif, add provisioning timeout timer operations.
     * When received a provisioning PDU, restart the 60s timer.
     */
    if (atomic_test_and_clear_bit(link1.flags, TIMEOUT_START)) {
        k_delayed_work_cancel(&link1.timeout);
    }
    if (!atomic_test_and_set_bit(link1.flags, TIMEOUT_START)) {
        k_delayed_work_submit(&link1.timeout, PROVISION_TIMEOUT);
    }

    prov_handlers[type].func(buf->data);

    return 0;
}

int bt_mesh_pb_gatt_open(struct bt_conn *conn)
{
    ALOGE("conn %p", conn);

    if (atomic_test_and_set_bit(link1.flags, LINK_ACTIVE)) {
        return -EBUSY;
    }

    link1.conn = bt_conn_ref(conn);
    link1.expect = PROV_INVITE;

    if (prov->link_open) {
        prov->link_open(BT_MESH_PROV_GATT);
    }

    return 0;
}

int bt_mesh_pb_gatt_close(struct bt_conn *conn)
{
    bool pub_key;

    ALOGE("conn %p", conn);

    if (link1.conn != conn) {
        ALOGE("Not connected");
        return -ENOTCONN;
    }

    /* Disable Attention Timer if it was set */
    if (link1.conf_inputs[0]) {
        bt_mesh_attention(NULL, 0);
    }

    if (prov->link_close) {
        prov->link_close(BT_MESH_PROV_GATT);
    }

    // Should reset the atomic flag after disconneted
    bt_conn_unref(link1.conn);

    pub_key = atomic_test_bit(link1.flags, LOCAL_PUB_KEY);

    /* Change by Espressif, use offsetof when clear link1 */
#if defined(CONFIG_BT_MESH_PB_ADV)
    memset(&link1, 0, offsetof(struct prov_link1, tx.retransmit));
    link1.rx.prev_id = XACT_NVAL;
#if defined(CONFIG_BT_MESH_PB_GATT)
    link1.rx.buf = bt_mesh_proxy_get_buf();
#else
    net_buf_simple_init(rx_buf, 0);
    link1.rx.buf = rx_buf;
#endif
#else
    memset(&link1, 0, offsetof(struct prov_link1, timeout));
#endif /* CONFIG_BT_MESH_PB_ADV */

    if (pub_key) {
        atomic_set_bit(link1.flags, LOCAL_PUB_KEY);
    }

    return 0;
}
#endif /* CONFIG_BT_MESH_PB_GATT */

const u8_t *bt_mesh_prov_get_uuid(void)
{
    ALOGE("bt_mesh_prov_get_uuid1"); 
    return prov->uuid;

}

bool bt_prov_active(void)
{
    return atomic_test_bit(link1.flags, LINK_ACTIVE);
}

int bt_mesh_prov_init(const struct bt_mesh_prov *prov_info)
{
    const u8_t *key = NULL;

    if (!prov_info) {
        ALOGE("No provisioning context provided");
        return -EINVAL;
    }

    /* Change by Espressif. Use micro-ecc to generate public key now. */
    key = bt_pub_key_get();
    if (!key) {
        ALOGE("%s: Failed to generate public key", __func__);
        return -EIO;
    }

    prov = prov_info;

#if defined(CONFIG_BT_MESH_PB_ADV)
    k_delayed_work_init(&link1.tx.retransmit, prov_retransmit);
    link1.rx.prev_id = XACT_NVAL;

#if defined(CONFIG_BT_MESH_PB_GATT)
    link1.rx.buf = bt_mesh_proxy_get_buf();
#else
    net_buf_simple_init(rx_buf, 0);
    link1.rx.buf = rx_buf;
#endif

#endif /* CONFIG_BT_MESH_PB_ADV */

    
    struct bt_uuid_128 uuid = { .uuid.type = BT_UUID_TYPE_128 };
    memcpy(uuid.val, prov->uuid, 16);
    ALOGE("Device UUID: %s", bt_uuid_str(&uuid.uuid));

    /* Change by Espressif, add provisioning timeout timer init */
    k_delayed_work_init(&link1.timeout, prov_timeout);

    return 0;
}

void bt_mesh_prov_complete(u16_t net_idx, u16_t addr)
{
    if (prov->complete) {
        prov->complete(net_idx, addr);
    }
}

void bt_mesh_prov_reset(void)
{
    if (prov->reset) {
        prov->reset();
    }
}

#endif /* CONFIG_BT_MESH_NODE */

#endif /* #if CONFIG_BT_MESH */
