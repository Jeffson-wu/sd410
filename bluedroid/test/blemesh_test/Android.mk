#
#  Copyright (C) 2009-2012 Broadcom Corporation
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at:
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:=     \
    blemeshtest.c

LOCAL_C_INCLUDES :=
LOCAL_CFLAGS := -Wno-unused-parameter

LOCAL_CFLAGS += -std=c99

LOCAL_CFLAGS += -std=c99

LOCAL_CFLAGS += -DBUILDCFG \
                -DCONFIG_BT_MESH \
                -DCONFIG_BT_MESH_PROXY \
                -DCONFIG_BT_MESH_NODE \
                -DCONFIG_BT_MESH_PB_ADV \
                -DCONFIG_NET_BUF_POOL_USAGE \
                -DCONFIG_NET_BUF_LOG \
                -DCONFIG_BT_MESH_PB_GATT \
                -DCONFIG_BT_MESH_GATT_PROXY \
                -DCONFIG_BT_MESH_RELAY \
                -DCONFIG_BT_MESH_CFG_CLI \
                -DCONFIG_BT_MESH_HEALTH_CLI \

LOCAL_MODULE_TAGS := debug optional

LOCAL_MODULE:= blt

LOCAL_SHARED_LIBRARIES += libcutils   \
                          libutils    \
                          libhardware \
			  libhardware_legacy 
LOCAL_MULTILIB := 32

include $(BUILD_EXECUTABLE)
