/*  Bluetooth Mesh */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_TAG "proxy"
#include <utils/Log.h>

#include <string.h>
#include <errno.h>

#include "mesh_buf.h"
#include "mesh_util.h"
#if CONFIG_BT_MESH

#include "mesh_bearer_adapt.h"

#define ALOGE_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_PROXY)

#include "mesh.h"
#include "adv.h"
#include "net.h"
#include "prov.h"
#include "beacon.h"
#include "foundation.h"
#include "access.h"
#include "proxy.h"

#if CONFIG_BT_MESH_NODE

#define PDU_TYPE(data)     (data[0] & BIT_MASK(6))
#define PDU_SAR(data)      (data[0] >> 6)

#define SAR_COMPLETE       0x00
#define SAR_FIRST          0x01
#define SAR_CONT           0x02
#define SAR_LAST           0x03

#define CFG_FILTER_SET     0x00
#define CFG_FILTER_ADD     0x01
#define CFG_FILTER_REMOVE  0x02
#define CFG_FILTER_STATUS  0x03

#define CONFIG_BT_MAX_CONN CONFIG_BT_ACL_CONNECTIONS

#define PDU_HDR(sar, type) (sar << 6 | (type & BIT_MASK(6)))

#define CLIENT_BUF_SIZE 68

static const struct bt_le_adv_param slow_adv_param = {
    .options = (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME),
    .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
    .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
};

static const struct bt_le_adv_param fast_adv_param = {
    .options = (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME),
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_0,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_0,
};

static bool proxy_adv_enabled;

#if defined(CONFIG_BT_MESH_GATT_PROXY)
static void proxy_send_beacons(struct k_work *work);
static u16_t proxy_ccc_val;
#endif

#if defined(CONFIG_BT_MESH_PB_GATT)
static u16_t prov_ccc_val;
static bool prov_fast_adv;
#endif

static struct bt_mesh_proxy_client {
    struct bt_conn *conn;
    u16_t filter[CONFIG_BT_MESH_PROXY_FILTER_SIZE];
    enum __packed {
        NONE,
        WHITELIST,
        BLACKLIST,
        PROV,
    } filter_type;
    u8_t msg_type;
#if defined(CONFIG_BT_MESH_GATT_PROXY)
    struct k_work send_beacons;
#endif
    struct net_buf_simple    buf;
    u8_t                     buf_data[CLIENT_BUF_SIZE];
} clients[CONFIG_BT_MAX_CONN] = {
    [0 ... (CONFIG_BT_MAX_CONN - 1)] = {
#if defined(CONFIG_BT_MESH_GATT_PROXY)
        .send_beacons = _K_WORK_INITIALIZER(proxy_send_beacons),
#endif
        .buf.size = CLIENT_BUF_SIZE,
    },
};

/* Track which service is enabled */
static enum {
    MESH_GATT_NONE,
    MESH_GATT_PROV,
    MESH_GATT_PROXY,
} gatt_svc = MESH_GATT_NONE;

static char device_name[DEVICE_NAME_SIZE] = "ESP-BLE-MESH";

int bt_mesh_set_device_name(const char *name)
{
    if (!name) {
        ALOGE("%s: name is NULL", __func__);
        return -EINVAL;
    }

    if (strlen(name) > DEVICE_NAME_SIZE) {
        ALOGE("%s: too long name", __func__);
        return -EINVAL;
    }

    memcpy(device_name, name, strlen(name));

    return 0;
}

static struct bt_mesh_proxy_client *find_client(struct bt_conn *conn)
{
    int i;

    for (i = 0; i < (int)MESH_ARRAY_SIZE(clients); i++) {
        if (clients[i].conn == conn) {
            return &clients[i];
        }
    }

    return NULL;
}

#if defined(CONFIG_BT_MESH_GATT_PROXY)
/* Next subnet in queue to be advertised */
static int next_idx;

static int proxy_segment_and_send(struct bt_conn *conn, u8_t type,
                                  struct net_buf_simple *msg);

static int filter_set(struct bt_mesh_proxy_client *client,
                      struct net_buf_simple *buf)
{
    u8_t type;

    if (buf->len < 1) {
        ALOGE("Too short Filter Set message");
        return -EINVAL;
    }

    type = net_buf_simple_pull_u8(buf);
    ALOGE("type 0x%02x", type);

    switch (type) {
    case 0x00:
        memset(client->filter, 0, sizeof(client->filter));
        client->filter_type = WHITELIST;
        break;
    case 0x01:
        memset(client->filter, 0, sizeof(client->filter));
        client->filter_type = BLACKLIST;
        break;
    default:
        ALOGE("Prohibited Filter Type 0x%02x", type);
        return -EINVAL;
    }

    return 0;
}

static void filter_add(struct bt_mesh_proxy_client *client, u16_t addr)
{
    int i;

    ALOGE("addr 0x%04x", addr);

    if (addr == BT_MESH_ADDR_UNASSIGNED) {
        return;
    }

    for (i = 0; i < (int)MESH_ARRAY_SIZE(client->filter); i++) {
        if (client->filter[i] == addr) {
            return;
        }
    }

    for (i = 0; i < (int)MESH_ARRAY_SIZE(client->filter); i++) {
        if (client->filter[i] == BT_MESH_ADDR_UNASSIGNED) {
            client->filter[i] = addr;
            return;
        }
    }
}

static void filter_remove(struct bt_mesh_proxy_client *client, u16_t addr)
{
    int i;

    ALOGE("addr 0x%04x", addr);

    if (addr == BT_MESH_ADDR_UNASSIGNED) {
        return;
    }

    for (i = 0; i < (int)MESH_ARRAY_SIZE(client->filter); i++) {
        if (client->filter[i] == addr) {
            client->filter[i] = BT_MESH_ADDR_UNASSIGNED;
            return;
        }
    }
}

static void send_filter_status(struct bt_mesh_proxy_client *client,
                               struct bt_mesh_net_rx *rx,
                               struct net_buf_simple *buf)
{
    struct bt_mesh_net_tx tx = {
        .sub = rx->sub,
        .ctx = &rx->ctx,
        .src = bt_mesh_primary_addr(),
    };
    u16_t filter_size;
    int i, err;

    /* Configuration messages always have dst unassigned */
    tx.ctx->addr = BT_MESH_ADDR_UNASSIGNED;

    net_buf_simple_init(buf, 10);

    net_buf_simple_add_u8(buf, CFG_FILTER_STATUS);

    if (client->filter_type == WHITELIST) {
        net_buf_simple_add_u8(buf, 0x00);
    } else {
        net_buf_simple_add_u8(buf, 0x01);
    }

    for (filter_size = 0, i = 0; i < (int)MESH_ARRAY_SIZE(client->filter); i++) {
        if (client->filter[i] != BT_MESH_ADDR_UNASSIGNED) {
            filter_size++;
        }
    }

    net_buf_simple_add_be16(buf, filter_size);

    ALOGE("%u bytes: %s", buf->len, bt_hex(buf->data, buf->len));

    err = bt_mesh_net_encode(&tx, buf, true);
    if (err) {
        ALOGE("Encoding Proxy cfg message failed (err %d)", err);
        return;
    }

    err = proxy_segment_and_send(client->conn, BT_MESH_PROXY_CONFIG, buf);
    if (err) {
        ALOGE("Failed to send proxy cfg message (err %d)", err);
    }
}

static void proxy_cfg(struct bt_mesh_proxy_client *client)
{
    struct net_buf_simple *buf = NET_BUF_SIMPLE(29);
    struct bt_mesh_net_rx rx;
    u8_t opcode;
    int err;

    err = bt_mesh_net_decode(&client->buf, BT_MESH_NET_IF_PROXY_CFG,
                             &rx, buf);
    if (err) {
        ALOGE("Failed to decode Proxy Configuration (err %d)", err);
        return;
    }

    /* Remove network headers */
    net_buf_simple_pull(buf, BT_MESH_NET_HDR_LEN);

    ALOGE("%u bytes: %s", buf->len, bt_hex(buf->data, buf->len));

    if (buf->len < 1) {
        ALOGE("Too short proxy configuration PDU");
        return;
    }

    opcode = net_buf_simple_pull_u8(buf);
    switch (opcode) {
    case CFG_FILTER_SET:
        filter_set(client, buf);
        send_filter_status(client, &rx, buf);
        break;
    case CFG_FILTER_ADD:
        while (buf->len >= 2) {
            u16_t addr;

            addr = net_buf_simple_pull_be16(buf);
            filter_add(client, addr);
        }
        send_filter_status(client, &rx, buf);
        break;
    case CFG_FILTER_REMOVE:
        while (buf->len >= 2) {
            u16_t addr;

            addr = net_buf_simple_pull_be16(buf);
            filter_remove(client, addr);
        }
        send_filter_status(client, &rx, buf);
        break;
    default:
        ALOGE("Unhandled configuration OpCode 0x%02x", opcode);
        break;
    }
}

static int beacon_send(struct bt_conn *conn, struct bt_mesh_subnet *sub)
{
    struct net_buf_simple *buf = NET_BUF_SIMPLE(23);

    net_buf_simple_init(buf, 1);
    bt_mesh_beacon_create(sub, buf);

    return proxy_segment_and_send(conn, BT_MESH_PROXY_BEACON, buf);
}

static void proxy_send_beacons(struct k_work *work)
{
    struct bt_mesh_proxy_client *client;
    int i;

    client = CONTAINER_OF(work, struct bt_mesh_proxy_client, send_beacons);

    for (i = 0; i < (int)MESH_ARRAY_SIZE(bt_mesh.sub); i++) {
        struct bt_mesh_subnet *sub = &bt_mesh.sub[i];

        if (sub->net_idx != BT_MESH_KEY_UNUSED) {
            beacon_send(client->conn, sub);
        }
    }
}

void bt_mesh_proxy_beacon_send(struct bt_mesh_subnet *sub)
{
    int i;

    if (!sub) {
        /* NULL means we send on all subnets */
        for (i = 0; i < (int)MESH_ARRAY_SIZE(bt_mesh.sub); i++) {
            if (bt_mesh.sub[i].net_idx != BT_MESH_KEY_UNUSED) {
                bt_mesh_proxy_beacon_send(&bt_mesh.sub[i]);
            }
        }

        return;
    }

    for (i = 0; i < (int)MESH_ARRAY_SIZE(clients); i++) {
        if (clients[i].conn) {
            beacon_send(clients[i].conn, sub);
        }
    }
}

void bt_mesh_proxy_identity_start(struct bt_mesh_subnet *sub)
{
    sub->node_id = BT_MESH_NODE_IDENTITY_RUNNING;
    sub->node_id_start = k_uptime_get_32();

    /* Prioritize the recently enabled subnet */
    next_idx = sub - bt_mesh.sub;
}

void bt_mesh_proxy_identity_stop(struct bt_mesh_subnet *sub)
{
    sub->node_id = BT_MESH_NODE_IDENTITY_STOPPED;
    sub->node_id_start = 0;
}

int bt_mesh_proxy_identity_enable(void)
{
    int i, count = 0;

    ALOGE("%s", __func__);

    if (!bt_mesh_is_provisioned()) {
        return -EAGAIN;
    }

    for (i = 0; i < (int)MESH_ARRAY_SIZE(bt_mesh.sub); i++) {
        struct bt_mesh_subnet *sub = &bt_mesh.sub[i];

        if (sub->net_idx == BT_MESH_KEY_UNUSED) {
            continue;
        }

        if (sub->node_id == BT_MESH_NODE_IDENTITY_NOT_SUPPORTED) {
            continue;
        }

        bt_mesh_proxy_identity_start(sub);
        count++;
    }

    if (count) {
        bt_mesh_adv_update();
    }

    return 0;
}

#endif /* GATT_PROXY */

static void proxy_complete_pdu(struct bt_mesh_proxy_client *client)
{
    switch (client->msg_type) {
#if defined(CONFIG_BT_MESH_GATT_PROXY)
    case BT_MESH_PROXY_NET_PDU:
        ALOGE("Mesh Network PDU");
        bt_mesh_net_recv(&client->buf, 0, BT_MESH_NET_IF_PROXY);
        break;
    case BT_MESH_PROXY_BEACON:
        ALOGE("Mesh Beacon PDU");
        bt_mesh_beacon_recv(&client->buf);
        break;
    case BT_MESH_PROXY_CONFIG:
        ALOGE("Mesh Configuration PDU");
        proxy_cfg(client);
        break;
#endif
#if defined(CONFIG_BT_MESH_PB_GATT)
    case BT_MESH_PROXY_PROV:
        ALOGE("Mesh Provisioning PDU");
        bt_mesh_pb_gatt_recv(client->conn, &client->buf);
        break;
#endif
    default:
        ALOGE("Unhandled Message Type 0x%02x", client->msg_type);
        break;
    }

    net_buf_simple_init(&client->buf, 0);
}

#define ATTR_IS_PROV(attr) (attr->user_data != NULL)

static size_t proxy_recv(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr, const void *buf,
                          u16_t len, u16_t offset, u8_t flags)
{
    struct bt_mesh_proxy_client *client = find_client(conn);
    const u8_t *data = buf;

    if (!client) {
        return -ENOTCONN;
    }

    if (len < 1) {
        ALOGE("Too small Proxy PDU");
        return -EINVAL;
    }

    if (ATTR_IS_PROV(attr) != (PDU_TYPE(data) == BT_MESH_PROXY_PROV)) {
        ALOGE("Proxy PDU type doesn't match GATT service");
        return -EINVAL;
    }

    if (len - 1 > (int)net_buf_simple_tailroom(&client->buf)) {
        ALOGE("Too big proxy PDU");
        return -EINVAL;
    }

    switch (PDU_SAR(data)) {
    case SAR_COMPLETE:
        if (client->buf.len) {
            ALOGE("Complete PDU while a pending incomplete one");
            return -EINVAL;
        }

        client->msg_type = PDU_TYPE(data);
        net_buf_simple_add_mem(&client->buf, data + 1, len - 1);
        proxy_complete_pdu(client);
        break;

    case SAR_FIRST:
        if (client->buf.len) {
            ALOGE("First PDU while a pending incomplete one");
            return -EINVAL;
        }

        client->msg_type = PDU_TYPE(data);
        net_buf_simple_add_mem(&client->buf, data + 1, len - 1);
        break;

    case SAR_CONT:
        if (!client->buf.len) {
            ALOGE("Continuation with no prior data");
            return -EINVAL;
        }

        if (client->msg_type != PDU_TYPE(data)) {
            ALOGE("Unexpected message type in continuation");
            return -EINVAL;
        }

        net_buf_simple_add_mem(&client->buf, data + 1, len - 1);
        break;

    case SAR_LAST:
        if (!client->buf.len) {
            ALOGE("Last SAR PDU with no prior data");
            return -EINVAL;
        }

        if (client->msg_type != PDU_TYPE(data)) {
            ALOGE("Unexpected message type in last SAR PDU");
            return -EINVAL;
        }

        net_buf_simple_add_mem(&client->buf, data + 1, len - 1);
        proxy_complete_pdu(client);
        break;
    }

    return len;
}

static int conn_count;

static void proxy_connected(struct bt_conn *conn, u8_t err)
{
    struct bt_mesh_proxy_client *client;
    int i;

    ALOGE("conn %p err 0x%02x", conn, err);

    conn_count++;

    /* Since we use ADV_OPT_ONE_TIME */
    proxy_adv_enabled = false;

    /* Try to re-enable advertising in case it's possible */
    if (conn_count < CONFIG_BT_MAX_CONN) {
        bt_mesh_adv_update();
    }

    for (client = NULL, i = 0; i < (int)MESH_ARRAY_SIZE(clients); i++) {
        if (!clients[i].conn) {
            client = &clients[i];
            break;
        }
    }

    if (!client) {
        ALOGE("No free Proxy Client objects");
        return;
    }

    client->conn = bt_conn_ref(conn);
    client->filter_type = NONE;
    memset(client->filter, 0, sizeof(client->filter));
    net_buf_simple_init(&client->buf, 0);
}

static void proxy_disconnected(struct bt_conn *conn, u8_t reason)
{
    int i;

    ALOGE("conn %p reason 0x%02x", conn, reason);

    conn_count--;

    for (i = 0; i < (int)MESH_ARRAY_SIZE(clients); i++) {
        struct bt_mesh_proxy_client *client = &clients[i];

        if (client->conn == conn) {
            if (IS_ENABLED(CONFIG_BT_MESH_PB_GATT) &&
                    client->filter_type == PROV) {
                bt_mesh_pb_gatt_close(conn);
            }

            // Should reset the atomic flag after disconneted
            bt_conn_unref(client->conn);
            client->conn = NULL;
            break;
        }
    }

    bt_mesh_adv_update();
}

struct net_buf_simple *bt_mesh_proxy_get_buf(void)
{
    struct net_buf_simple *buf = &clients[0].buf;

    net_buf_simple_init(buf, 0);

    return buf;
}

#if defined(CONFIG_BT_MESH_PB_GATT)
static size_t prov_ccc_write(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, u16_t len,
                              u16_t offset, u8_t flags)
{
    struct bt_mesh_proxy_client *client;
    u16_t *value = attr->user_data;

    ALOGE("len %u: %s", len, bt_hex(buf, len));

    if (len != sizeof(*value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    *value = sys_get_le16(buf);
    if (*value != BT_GATT_CCC_NOTIFY) {
        ALOGE("Client wrote 0x%04x instead enabling notify", *value);
        return len;
    }

    /* If a connection exists there must be a client */
    client = find_client(conn);
    __ASSERT(client, "No client for connection");

    if (client->filter_type == NONE) {
        client->filter_type = PROV;
        bt_mesh_pb_gatt_open(conn);
    }

    return len;
}

static size_t prov_ccc_read(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, u16_t len, u16_t offset)
{
    u16_t *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(*value));
}

/* Mesh Provisioning Service Declaration */
static struct bt_gatt_attr prov_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(BT_UUID_MESH_PROV),

    BT_GATT_CHARACTERISTIC(BT_UUID_MESH_PROV_DATA_IN,
    BT_GATT_CHRC_WRITE_WITHOUT_RESP),
    BT_GATT_DESCRIPTOR(BT_UUID_MESH_PROV_DATA_IN, BT_GATT_PERM_WRITE,
    NULL, proxy_recv, (void *)1),

    BT_GATT_CHARACTERISTIC(BT_UUID_MESH_PROV_DATA_OUT,
    BT_GATT_CHRC_NOTIFY),
    BT_GATT_DESCRIPTOR(BT_UUID_MESH_PROV_DATA_OUT, BT_GATT_PERM_NONE,
    NULL, NULL, NULL),
    /* Add custom CCC as clients need to be tracked individually */
    BT_GATT_DESCRIPTOR(BT_UUID_GATT_CCC,
    BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
    prov_ccc_read, prov_ccc_write, &prov_ccc_val),
};

struct bt_gatt_service prov_svc = BT_GATT_SERVICE(prov_attrs);

int bt_mesh_proxy_prov_enable(void)
{
    int i;

    ALOGE("%s", __func__);

    /**TODO: Change by Espressif. We should not register service here, otherwise will
     * block btu task, so currently just start it. Need to further check*/
    bt_gatt_service_start(&prov_svc);
    //bt_gatt_service_register(&prov_svc);
    gatt_svc = MESH_GATT_PROV;
    prov_fast_adv = true;

    for (i = 0; i < (int)MESH_ARRAY_SIZE(clients); i++) {
        if (clients[i].conn) {
            clients[i].filter_type = PROV;
        }
    }


    return 0;
}

int bt_mesh_proxy_prov_disable(void)
{
    int i;

    ALOGE("%s", __func__);

    /**TODO: Change by Espressif. Please refer to bt_mesh_proxy_prov_enable for
     * more info */
    bt_gatt_service_stop(&prov_svc);
    //bt_gatt_service_unregister(&prov_svc);
    gatt_svc = MESH_GATT_NONE;

    for (i = 0; i < (int)MESH_ARRAY_SIZE(clients); i++) {
        struct bt_mesh_proxy_client *client = &clients[i];

        if (client->conn && client->filter_type == PROV) {
            bt_mesh_pb_gatt_close(client->conn);
            client->filter_type = NONE;
        }
    }

    return 0;
}

#endif /* CONFIG_BT_MESH_PB_GATT */

#if defined(CONFIG_BT_MESH_GATT_PROXY)
static size_t proxy_ccc_write(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               const void *buf, u16_t len,
                               u16_t offset, u8_t flags)
{
    struct bt_mesh_proxy_client *client;
    u16_t value;

    ALOGE("len %u: %s", len, bt_hex(buf, len));

    if (len != sizeof(value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    value = sys_get_le16(buf);
    if (value != BT_GATT_CCC_NOTIFY) {
        ALOGE("Client wrote 0x%04x instead enabling notify", value);
        return len;
    }

    /* If a connection exists there must be a client */
    client = find_client(conn);
    __ASSERT(client, "No client for connection");

    if (client->filter_type == NONE) {
        client->filter_type = WHITELIST;
        k_work_submit(&client->send_beacons);
    }

    return len;
}

static size_t proxy_ccc_read(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, u16_t len, u16_t offset)
{
    u16_t *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(*value));
}

/* Mesh Proxy Service Declaration */
static struct bt_gatt_attr proxy_attrs[] = {
    BT_GATT_PRIMARY_SERVICE(BT_UUID_MESH_PROXY),

    BT_GATT_CHARACTERISTIC(BT_UUID_MESH_PROXY_DATA_IN,
    BT_GATT_CHRC_WRITE_WITHOUT_RESP),
    BT_GATT_DESCRIPTOR(BT_UUID_MESH_PROXY_DATA_IN, BT_GATT_PERM_WRITE,
    NULL, proxy_recv, NULL),

    BT_GATT_CHARACTERISTIC(BT_UUID_MESH_PROXY_DATA_OUT,
    BT_GATT_CHRC_NOTIFY),
    BT_GATT_DESCRIPTOR(BT_UUID_MESH_PROXY_DATA_OUT, BT_GATT_PERM_NONE,
    NULL, NULL, NULL),
    /* Add custom CCC as clients need to be tracked individually */
    BT_GATT_DESCRIPTOR(BT_UUID_GATT_CCC,
    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
    proxy_ccc_read, proxy_ccc_write, &proxy_ccc_val),
};

struct bt_gatt_service proxy_svc = BT_GATT_SERVICE(proxy_attrs);

int bt_mesh_proxy_gatt_enable(void)
{
    int i;

    ALOGE("%s", __func__);

    /**TODO: We should not register service here, otherwise will
     * block btu task, so currently just start it. Need to further check*/
    bt_gatt_service_start(&proxy_svc);
    //bt_gatt_service_register(&proxy_svc);
    gatt_svc = MESH_GATT_PROXY;

    for (i = 0; i < (int)MESH_ARRAY_SIZE(clients); i++) {
        if (clients[i].conn) {
            clients[i].filter_type = WHITELIST;
        }
    }

    return 0;
}

void bt_mesh_proxy_gatt_disconnect(void)
{
    int i;

    ALOGE("%s", __func__);

    for (i = 0; i < (int)MESH_ARRAY_SIZE(clients); i++) {
        struct bt_mesh_proxy_client *client = &clients[i];

        if (client->conn && (client->filter_type == WHITELIST ||
                             client->filter_type == BLACKLIST)) {
            client->filter_type = NONE;
            bt_conn_disconnect(client->conn, 0x13);
            //BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        }
    }
}

int bt_mesh_proxy_gatt_disable(void)
{
    ALOGE("%s", __func__);

    bt_mesh_proxy_gatt_disconnect();

    /**TODO: please refer to comment of bt_mesh_proxy_gatt_enable,
     * We not de-register, but only stop service*/
    bt_gatt_service_stop(&proxy_svc);
    //bt_gatt_service_unregister(&proxy_svc);
    gatt_svc = MESH_GATT_NONE;

    return 0;
}

void bt_mesh_proxy_addr_add(struct net_buf_simple *buf, u16_t addr)
{
    struct bt_mesh_proxy_client *client =
        CONTAINER_OF(buf, struct bt_mesh_proxy_client, buf);

    ALOGE("filter_type %u addr 0x%04x", client->filter_type, addr);

    if (client->filter_type == WHITELIST) {
        filter_add(client, addr);
    } else if (client->filter_type == BLACKLIST) {
        filter_remove(client, addr);
    }
}

static bool client_filter_match(struct bt_mesh_proxy_client *client,
                                u16_t addr)
{
    int i;

    ALOGE("filter_type %u addr 0x%04x", client->filter_type, addr);

    if (client->filter_type == WHITELIST) {
        for (i = 0; i < (int)MESH_ARRAY_SIZE(client->filter); i++) {
            if (client->filter[i] == addr) {
                return true;
            }
        }

        return false;
    }

    if (client->filter_type == BLACKLIST) {
        for (i = 0; i < (int)MESH_ARRAY_SIZE(client->filter); i++) {
            if (client->filter[i] == addr) {
                return false;
            }
        }

        return true;
    }

    return false;
}

bool bt_mesh_proxy_relay(struct net_buf_simple *buf, u16_t dst)
{
    bool relayed = false;
    int i;

    ALOGE("%u bytes to dst 0x%04x", buf->len, dst);

    for (i = 0; i < (int)MESH_ARRAY_SIZE(clients); i++) {
        struct bt_mesh_proxy_client *client = &clients[i];
        struct net_buf_simple *msg = NET_BUF_SIMPLE(32);

        if (!client->conn) {
            continue;
        }

        if (!client_filter_match(client, dst)) {
            continue;
        }

        /* Proxy PDU sending modifies the original buffer,
         * so we need to make a copy.
         */
        net_buf_simple_init(msg, 1);
        net_buf_simple_add_mem(msg, buf->data, buf->len);

        bt_mesh_proxy_send(client->conn, BT_MESH_PROXY_NET_PDU, msg);
        relayed = true;
    }

    return relayed;
}

#endif /* CONFIG_BT_MESH_GATT_PROXY */

static int proxy_send(struct bt_conn *conn, const void *data, u16_t len)
{
    ALOGE("%u bytes: %s", len, bt_hex(data, len));

#if defined(CONFIG_BT_MESH_GATT_PROXY)
    if (gatt_svc == MESH_GATT_PROXY) {
        return bt_gatt_notify(conn, &proxy_attrs[4], data, len);
    }
#endif

#if defined(CONFIG_BT_MESH_PB_GATT)
    if (gatt_svc == MESH_GATT_PROV) {
        return bt_gatt_notify(conn, &prov_attrs[4], data, len);
    }
#endif

    return 0;
}

static int proxy_segment_and_send(struct bt_conn *conn, u8_t type,
                                  struct net_buf_simple *msg)
{
    u16_t mtu;

    ALOGE("conn %p type 0x%02x len %u: %s", conn, type, msg->len,
           bt_hex(msg->data, msg->len));

    /* ATT_MTU - OpCode (1 byte) - Handle (2 bytes) */
    mtu = bt_gatt_get_mtu(conn) - 3;
    if (mtu > msg->len) {
        net_buf_simple_push_u8(msg, PDU_HDR(SAR_COMPLETE, type));
        return proxy_send(conn, msg->data, msg->len);
    }

    net_buf_simple_push_u8(msg, PDU_HDR(SAR_FIRST, type));
    proxy_send(conn, msg->data, mtu);
    net_buf_simple_pull(msg, mtu);

    while (msg->len) {
        if (msg->len + 1 < mtu) {
            net_buf_simple_push_u8(msg, PDU_HDR(SAR_LAST, type));
            proxy_send(conn, msg->data, msg->len);
            break;
        }

        net_buf_simple_push_u8(msg, PDU_HDR(SAR_CONT, type));
        proxy_send(conn, msg->data, mtu);
        net_buf_simple_pull(msg, mtu);
    }

    return 0;
}

int bt_mesh_proxy_send(struct bt_conn *conn, u8_t type,
                       struct net_buf_simple *msg)
{
    struct bt_mesh_proxy_client *client = find_client(conn);

    if (!client) {
        ALOGE("No Proxy Client found");
        return -ENOTCONN;
    }

    if ((client->filter_type == PROV) != (type == BT_MESH_PROXY_PROV)) {
        ALOGE("Invalid PDU type for Proxy Client");
        return -EINVAL;
    }

    return proxy_segment_and_send(conn, type, msg);
}

#if defined(CONFIG_BT_MESH_PB_GATT)
static u8_t prov_svc_data[20] = { 0x27, 0x18, };

static const struct bt_data prov_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x27, 0x18),
    BT_DATA(BT_DATA_SVC_DATA16, prov_svc_data, sizeof(prov_svc_data)),
};

static const struct bt_data prov_sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, device_name,
    (sizeof(device_name) - 1)),
};
#endif /* PB_GATT */

#if defined(CONFIG_BT_MESH_GATT_PROXY)

#define ID_TYPE_NET  0x00
#define ID_TYPE_NODE 0x01

#define NODE_ID_LEN  19
#define NET_ID_LEN   11

#define NODE_ID_TIMEOUT K_SECONDS(CONFIG_BT_MESH_NODE_ID_TIMEOUT)

static u8_t proxy_svc_data[NODE_ID_LEN] = { 0x28, 0x18, };

static const struct bt_data node_id_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x28, 0x18),
    BT_DATA(BT_DATA_SVC_DATA16, proxy_svc_data, NODE_ID_LEN),
};

static const struct bt_data net_id_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x28, 0x18),
    BT_DATA(BT_DATA_SVC_DATA16, proxy_svc_data, NET_ID_LEN),
};

static int node_id_adv(struct bt_mesh_subnet *sub)
{
    u8_t tmp[16];
    int err;

    ALOGE("%s", __func__);

    proxy_svc_data[2] = ID_TYPE_NODE;

    err = bt_rand(proxy_svc_data + 11, 8);
    if (err) {
        return err;
    }

    memset(tmp, 0, 6);
    memcpy(tmp + 6, proxy_svc_data + 11, 8);
    sys_put_be16(bt_mesh_primary_addr(), tmp + 14);

    err = bt_encrypt_be(sub->keys[sub->kr_flag].identity, tmp, tmp);
    if (err) {
        return err;
    }

    memcpy(proxy_svc_data + 3, tmp + 8, 8);

    err = bt_le_adv_start(&fast_adv_param, node_id_ad,
                          MESH_ARRAY_SIZE(node_id_ad), NULL, 0);
    if (err) {
        ALOGE("Failed to advertise using Node ID (err %d)", err);
        return err;
    }

    proxy_adv_enabled = true;

    return 0;
}

static int net_id_adv(struct bt_mesh_subnet *sub)
{
    int err;

    ALOGE("%s", __func__);

    proxy_svc_data[2] = ID_TYPE_NET;

    ALOGE("Advertising with NetId %s",
           bt_hex(sub->keys[sub->kr_flag].net_id, 8));

    memcpy(proxy_svc_data + 3, sub->keys[sub->kr_flag].net_id, 8);

    err = bt_le_adv_start(&slow_adv_param, net_id_ad,
                          MESH_ARRAY_SIZE(net_id_ad), NULL, 0);
    if (err) {
        ALOGE("Failed to advertise using Network ID (err %d)", err);
        return err;
    }

    proxy_adv_enabled = true;

    return 0;
}

static bool advertise_subnet(struct bt_mesh_subnet *sub)
{
    if (sub->net_idx == BT_MESH_KEY_UNUSED) {
        return false;
    }

    return (sub->node_id == BT_MESH_NODE_IDENTITY_RUNNING ||
            bt_mesh_gatt_proxy_get() == BT_MESH_GATT_PROXY_ENABLED);
}

static struct bt_mesh_subnet *next_sub(void)
{
    int i;

    for (i = 0; i < (int)MESH_ARRAY_SIZE(bt_mesh.sub); i++) {
        struct bt_mesh_subnet *sub;

        sub = &bt_mesh.sub[(i + next_idx) % MESH_ARRAY_SIZE(bt_mesh.sub)];
        if (advertise_subnet(sub)) {
            next_idx = (next_idx + 1) % MESH_ARRAY_SIZE(bt_mesh.sub);
            return sub;
        }
    }

    return NULL;
}

static int sub_count(void)
{
    int i, count = 0;

    for (i = 0; i < (int)MESH_ARRAY_SIZE(bt_mesh.sub); i++) {
        struct bt_mesh_subnet *sub = &bt_mesh.sub[i];

        if (advertise_subnet(sub)) {
            count++;
        }
    }

    return count;
}

static s32_t gatt_proxy_advertise(struct bt_mesh_subnet *sub)
{
    s32_t remaining = K_FOREVER;
    int subnet_count;

    ALOGE("%s", __func__);

    if (conn_count == CONFIG_BT_MAX_CONN) {
        ALOGE("Connectable advertising deferred (max connections)");
        return remaining;
    }

    if (!sub) {
        ALOGE("No subnets to advertise on");
        return remaining;
    }

    if (sub->node_id == BT_MESH_NODE_IDENTITY_RUNNING) {
        u32_t active = k_uptime_get_32() - sub->node_id_start;

        if (active < NODE_ID_TIMEOUT) {
            remaining = NODE_ID_TIMEOUT - active;
            ALOGE("Node ID active for %u ms, %d ms remaining",
                   active, remaining);
            node_id_adv(sub);
        } else {
            bt_mesh_proxy_identity_stop(sub);
            ALOGE("Node ID stopped");
        }
    }

    if (sub->node_id == BT_MESH_NODE_IDENTITY_STOPPED) {
        if (bt_mesh_gatt_proxy_get() == BT_MESH_GATT_PROXY_ENABLED) {
            net_id_adv(sub);
        } else {
            return gatt_proxy_advertise(next_sub());
        }
    }

    subnet_count = sub_count();
    ALOGE("sub_count %u", subnet_count);
    if (subnet_count > 1) {
        s32_t max_timeout;

        /* We use NODE_ID_TIMEOUT as a starting point since it may
         * be less than 60 seconds. Divide this period into at least
         * 6 slices, but make sure that a slice is at least one
         * second long (to avoid excessive rotation).
         */
        max_timeout = NODE_ID_TIMEOUT / max(subnet_count, 6);
        max_timeout = max(max_timeout, K_SECONDS(1));

        if (remaining > max_timeout || remaining < 0) {
            remaining = max_timeout;
        }
    }

    ALOGE("Advertising %d ms for net_idx 0x%04x", remaining, sub->net_idx);

    return remaining;
}
#endif /* GATT_PROXY */

s32_t bt_mesh_proxy_adv_start(void)
{
    ALOGE("%s", __func__);

    if (gatt_svc == MESH_GATT_NONE) {
        ALOGE("%s, MESH_GATT_NONE", __func__);
        return K_FOREVER;
    }

#if defined(CONFIG_BT_MESH_PB_GATT)
    if (!bt_mesh_is_provisioned()) {
        const struct bt_le_adv_param *param;

        if (prov_fast_adv) {
            param = &fast_adv_param;
        } else {
            param = &slow_adv_param;
        }

        if (bt_le_adv_start(param, prov_ad, MESH_ARRAY_SIZE(prov_ad),
                            prov_sd, MESH_ARRAY_SIZE(prov_sd)) == 0) {
            proxy_adv_enabled = true;

            /* Advertise 60 seconds using fast interval */
            if (prov_fast_adv) {
                prov_fast_adv = false;
                return K_SECONDS(60);
            }
        }
    }
#endif /* PB_GATT */

#if defined(CONFIG_BT_MESH_GATT_PROXY)
    if (bt_mesh_is_provisioned()) {
        return gatt_proxy_advertise(next_sub());
    }
#endif /* GATT_PROXY */

    return K_FOREVER;
}

void bt_mesh_proxy_adv_stop(void)
{
    int err;

    ALOGE("adv_enabled %u", proxy_adv_enabled);

    if (!proxy_adv_enabled) {
        return;
    }

    err = bt_le_adv_stop();
    if (err) {
        ALOGE("Failed to stop advertising (err %d)", err);
    } else {
        proxy_adv_enabled = false;
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = proxy_connected,
    .disconnected = proxy_disconnected,
};

int bt_mesh_proxy_init(void)
{
    ALOGE("%s: bt_conn_cb_register1", __func__);
    bt_conn_cb_register(&conn_callbacks);

    ALOGE("%s: bt_conn_cb_register2", __func__);
#if defined(CONFIG_BT_MESH_PB_GATT)
    if (!bt_mesh_prov_get_uuid()) {
        ALOGE("%s: prov->uuid not initialized", __func__);
        return -EINVAL;
    }
    ALOGE("%s: bt_conn_cb_register3", __func__);
    memcpy(prov_svc_data + 2, bt_mesh_prov_get_uuid(), 16);
    ALOGE("%s: bt_conn_cb_register4", __func__);
#endif

    return 0;
}

#endif /* CONFIG_BT_MESH_NODE */

#endif /* #if CONFIG_BT_MESH */
