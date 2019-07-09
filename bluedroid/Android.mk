LOCAL_PATH := $(call my-dir)

bdroid_CFLAGS := -Wno-unused-parameter

# Setup bdroid local make variables for handling configuration
ifneq ($(BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR),)
  bdroid_C_INCLUDES := $(BOARD_BLUETOOTH_BDROID_BUILDCFG_INCLUDE_DIR)
  bdroid_CFLAGS += -DHAS_BDROID_BUILDCFG
else
  bdroid_C_INCLUDES :=
  bdroid_CFLAGS += -DHAS_NO_BDROID_BUILDCFG
endif

bdroid_CFLAGS += -Wall -Werror
bdroid_CFLAGS += -DAVK_BACKPORT

bdroid_CFLAGS += -DCONFIG_BT_MESH \
                  -DCONFIG_BT_MESH_PROXY \
                  -DCONFIG_BT_MESH_PROV \
                  -DCONFIG_BT_MESH_NODE \
                  -DCONFIG_NET_BUF_POOL_USAGE \
                  -DCONFIG_NET_BUF_LOG \
                  -DCONFIG_BT_MESH_PB_GATT \
                  -DCONFIG_BT_MESH_GATT_PROXY \
                  -DCONFIG_BT_MESH_RELAY \
                  -DCONFIG_BT_MESH_CFG_CLI \
                  -DCONFIG_BT_MESH_HEALTH_CLI \

ifneq ($(BOARD_BLUETOOTH_BDROID_HCILP_INCLUDED),)
  bdroid_CFLAGS += -DHCILP_INCLUDED=$(BOARD_BLUETOOTH_BDROID_HCILP_INCLUDED)
endif

include $(call all-subdir-makefiles)

# Cleanup our locals
bdroid_C_INCLUDES :=
bdroid_CFLAGS :=
