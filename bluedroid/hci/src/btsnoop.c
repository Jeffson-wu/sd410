/******************************************************************************
 *
 *  Copyright (C) 2009-2012 Broadcom Corporation
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

#define LOG_TAG "btsnoop"

#include <arpa/inet.h>
#include <assert.h>
#include <ctype.h>
#include <cutils/log.h>
#include <cutils/properties.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/poll.h>
#include <unistd.h>

#include "bt_hci_bdroid.h"
#include "bt_utils.h"
#include "utils.h"

typedef enum {
  kCommandPacket = 1,
  kAclPacket = 2,
  kScoPacket = 3,
  kEventPacket = 4
} packet_type_t;

// Epoch in microseconds since 01/01/0000.
static const uint64_t BTSNOOP_EPOCH_DELTA = 0x00dcddb30f2f8000ULL;

// File descriptor for btsnoop file.

static int hci_btsnoop_fd = -1;
extern int client_socket_btsnoop;
static long int gmt_offset;
#define USEC_PER_SEC 1000000L
bool hci_ext_dump_enabled = FALSE;  /* External BT snoop */

void btsnoop_net_open();
void btsnoop_net_close();
void btsnoop_net_write(const void *data, size_t length);

static uint64_t btsnoop_timestamp(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  tv.tv_sec += gmt_offset;

  // Timestamp is in microseconds.
  uint64_t timestamp = tv.tv_sec * 1000 * 1000LL;
  timestamp += tv.tv_usec;
  timestamp += BTSNOOP_EPOCH_DELTA;
  return timestamp;
}

static void btsnoop_write(const void *data, size_t length) {
  if (client_socket_btsnoop != -1) {
    btsnoop_net_write(data, length);
    /* skip writing to file if external client is connected*/
    return;
  }

  if (hci_btsnoop_fd != -1) {
    write(hci_btsnoop_fd, data, length);
  }
}

static uint64_t time_now_us() {
    struct timespec ts_now;
    clock_gettime(CLOCK_BOOTTIME, &ts_now);
    return ((uint64_t)ts_now.tv_sec * USEC_PER_SEC) + ((uint64_t)ts_now.tv_nsec / 1000);
}

static void btsnoop_write_packet(packet_type_t type, const uint8_t *packet, bool is_received) {
  int length_he = 0;
  int length;
  int flags;
  int drops = 0;
  struct pollfd pfd;
  uint64_t ts_begin;
  uint64_t ts_end, ts_diff;
  uint8_t snoop_buf[1200] = {0};
  uint32_t offset = 0;

  switch (type) {
    case kCommandPacket:
      length_he = packet[2] + 4;
      flags = 2;
      break;
    case kAclPacket:
      length_he = (packet[3] << 8) + packet[2] + 5;
      flags = is_received;
      break;
    case kScoPacket:
      length_he = packet[2] + 4;
      flags = is_received;
      break;
    case kEventPacket:
      length_he = packet[1] + 3;
      flags = 3;
      break;
  }

  uint64_t timestamp = btsnoop_timestamp();
  uint32_t time_hi = timestamp >> 32;
  uint32_t time_lo = timestamp & 0xFFFFFFFF;

  length = htonl(length_he);
  flags = htonl(flags);
  drops = htonl(drops);
  time_hi = htonl(time_hi);
  time_lo = htonl(time_lo);

  /* store the length in both original and included fields */
  memcpy(snoop_buf + offset, &length, 4);
  offset += 4;
  memcpy(snoop_buf + offset, &length, 4);
  offset += 4;

  /* flags:  */
  memcpy(snoop_buf + offset, &flags, 4);
  offset += 4;

  /* drops: none */
  memcpy(snoop_buf + offset, &drops, 4);
  offset += 4;

  /* time */
  memcpy(snoop_buf + offset, &time_hi, 4);
  offset += 4;
  memcpy(snoop_buf + offset, &time_lo, 4);
  offset = offset + 4;

  snoop_buf[offset] = type;
  offset += 1;
  memcpy(snoop_buf + offset, packet, length_he - 1);

  // This function is called from different contexts.
  utils_lock();
  if (client_socket_btsnoop != -1) {
    pfd.fd = client_socket_btsnoop;
    pfd.events = POLLOUT;

    ts_begin = time_now_us();

    if (poll(&pfd, 1, 10) == 0) {
      ALOGE("btsnoop poll : Taking more than 10 ms : skip dump");
      ts_end = time_now_us();
      utils_unlock();
      ts_diff = ts_end - ts_begin;
      if (ts_diff > 10000) {
        ALOGE("btsnoop poll T/O : took more time %08lld us", ts_diff);
      }
      return;
    }

    ts_end = time_now_us();
    ts_diff = ts_end - ts_begin;
    if (ts_diff > 10000) {
      ALOGE("btsnoop poll : took more time %08lld us", ts_diff);
    }
  }
  ts_begin = time_now_us();

  btsnoop_write(snoop_buf, offset + length_he - 1);

  ts_end = time_now_us();
  ts_diff = ts_end - ts_begin;
  utils_unlock();
  if (ts_diff > 10000) {
    ALOGE("btsnoop write : Write took more time %08lld us", ts_diff);
  }
}

void btsnoop_open(const char *p_path, const bool save_existing) {
  time_t t = time(NULL);
  struct tm tm_cur = {0};

  assert(p_path != NULL);
  assert(*p_path != '\0');

  localtime_r (&t, &tm_cur);
  ALOGI("Time GMT offset %ld\n", tm_cur.tm_gmtoff);
  gmt_offset = tm_cur.tm_gmtoff;

  btsnoop_net_open();
  if (hci_ext_dump_enabled == true) {
    property_set("bluetooth.startbtsnoop", "true");
  }

  if (hci_btsnoop_fd != -1) {
    ALOGE("%s btsnoop log file is already open.", __func__);
    return;
  }

  if (save_existing)
  {
    char fname_backup[266] = {0};
    strncat(fname_backup, p_path, 255);
    strcat(fname_backup, ".last");
    rename(p_path, fname_backup);
  }

  hci_btsnoop_fd = open(p_path,
                        O_WRONLY | O_CREAT | O_TRUNC,
                        S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH);

  if (hci_btsnoop_fd == -1) {
    ALOGE("%s unable to open '%s': %s", __func__, p_path, strerror(errno));
    return;
  }

  write(hci_btsnoop_fd, "btsnoop\0\0\0\0\1\0\0\x3\xea", 16);
}

void btsnoop_close(void) {
  if (hci_btsnoop_fd != -1)
    close(hci_btsnoop_fd);
  hci_btsnoop_fd = -1;

  if (hci_ext_dump_enabled == true) {
    property_set("bluetooth.startbtsnoop", "false");
  }

  btsnoop_net_close();
}

void btsnoop_capture(const HC_BT_HDR *p_buf, bool is_rcvd) {
  const uint8_t *p = (const uint8_t *)(p_buf + 1) + p_buf->offset;

  if (hci_btsnoop_fd == -1)
    return;

  switch (p_buf->event & MSG_EVT_MASK) {
    case MSG_HC_TO_STACK_HCI_EVT:
      btsnoop_write_packet(kEventPacket, p, false);
      break;
    case MSG_HC_TO_STACK_HCI_ACL:
    case MSG_STACK_TO_HC_HCI_ACL:
      btsnoop_write_packet(kAclPacket, p, is_rcvd);
      break;
    case MSG_HC_TO_STACK_HCI_SCO:
    case MSG_STACK_TO_HC_HCI_SCO:
      btsnoop_write_packet(kScoPacket, p, is_rcvd);
      break;
    case MSG_STACK_TO_HC_HCI_CMD:
      btsnoop_write_packet(kCommandPacket, p, true);
      break;
  }
}
