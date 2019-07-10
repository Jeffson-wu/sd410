/*
 *
 * Quicklogic vx5b3a dsi convertor driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
//#include "msm_fb.h"
#include "mdss_dsi.h"
//#include "mipi_quickvx.h"
//#include "mdp4.h"

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

//#include "ql_debug.h"
// local debug message.
#if 0
#define QL_DBG(x...) 
#define QL_DBGL(x...) 
#else
#define QL_DBG(f, x...) \
	printk("[QLVX] %s: " f, __func__,## x)
#define QL_DBGL(lvl, f, x...) do {if (lvl) printk("[QLVX] %s: " f, __func__,## x); }while(0)
#endif
#define QL_ERR(f, x...) \
	printk("[QLVX] ERROR %s: " f, __func__,## x)

//#define QL_TEST_NO_HW
//#define QL_MIPI_READ_AFTER_WRITE_DEBUG
//#define QL_I2C_READ_AFTER_WRITE_DEBUG
//if use i2c for sysfs to access vx registers
#define QL_VX_SYSFS_I2C_ACCESS_READ
//if use i2c direct access for all VX register i2c access
//#define QL_VX_REG_I2C_DIRECT_ACCESS
//#define QL_VX_LCD_I2C_for_MIPI_ACCESS
#ifdef QL_VX_LCD_I2C_for_MIPI_ACCESS
#define QL_VX_LCD_VC 0
#endif


//if use i2c direct access for all VX register i2c access
//#define QL_VX_REG_I2C_DIRECT_ACCESS
//#define QL_VX_LCD_I2C_for_MIPI_ACCESS

#define LVDS_1024x768P60	0
#define LVDS_800x480P60		0
#define LVDS_640x360P60		0
#define RGB_640x360P60		0
#define LVDS_640x360P60_90M	0
#define RGB_640x360P60_90M	1
#define RGB_640x360P50		0

struct vx5b3a_data {
	struct i2c_client *client;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
};

//register.

struct chip_init_data {
	int reg_addr; 
	int data;
};

void ql_chip_init(struct msm_fb_data_type *mfd);
int ql_i2c_read(u32 addr, u32 *val, u32 data_size);
int ql_i2c_write(long addr, long val, int data_size);
int ql_i2c_release(void);

//extern void panel_hud_disp_on(void);

struct chip_init_data vx3_init_data[] = {
#if LVDS_1024x768P60
{0x700,	0x34900040},
{0x704,	0x101BD   },
{0x70C,	0x00004604},
{0x710,	0x004D000F},
{0x714,	0x20      },
{0x718,	0x00000102},
{0x71C,	0xA8002F  },
{0x720,	0x0       },
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x700,	0x34900040},
{0x70C,	0x00005E66},
{0x718,	0x00000202},
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x120,	0x5       },
{0x124,	0x21580400},
{0x128,	0x100C1D  },
{0x12C,	0x9D      },
{0x130,	0x3C10    },
{0x134,	0x00000005},
{0x138,	0xFF8000  },
{0x13C,	0x0       },
{0x140,	0x10000   },
{0x20C,	0x24      },
{0x21C,	0x7D0     },
{0x224,	0x0       },
{0x228,	0xA00000  },
{0x22C,	0xFF08    },
{0x230,	0x1       },
{0x234,	0xCA033E10},
{0x238,	0x00000060},
{0x23C,	0x82E86030},
{0x244,	0x001E0285},
{0x258,	0x60085   },
{0x158,	0x0       },
{0x158,	0x1       },
{0x37C,	0x00001063},
{0x380,	0x82A86030},
{0x384,	0x2861408B},
{0x388,	0x00130285},
{0x38C,	0x10630009},
{0x394,	0x400B82A8},
{0x600,	0x16CC78C },
{0x604,	0x3FFF83E8},
{0x608,	0xD0E     },
{0x154,	0x00000000},
{0x154,	0x80000000}
#elif LVDS_800x480P60
{0x700,	0x88900040},
{0x704,	0x401F4   },
{0x70C,	0x00004604},
{0x710,	0x004D000F},
{0x714,	0x20      },
{0x718,	0x00000102},
{0x71C,	0xA8002F  },
{0x720,	0x0       },
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x700,	0x88900040},
{0x70C,	0x00005E66},
{0x718,	0x00000202},
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x120,	0x5       },
{0x124,	0xBCF0320 },
{0x128,	0x10341D  },
{0x12C,	0x27      },
{0x130,	0x3C10    },
{0x134,	0x00000005},
{0x138,	0xFF8000  },
{0x13C,	0x0       },
{0x140,	0x10000   },
{0x20C,	0x24      },
{0x21C,	0x7D0     },
{0x224,	0x0       },
{0x228,	0xA00000  },
{0x22C,	0xFF01    },
{0x230,	0x1       },
{0x234,	0xCA033E10},
{0x238,	0x00000060},
{0x23C,	0x82E86030},
{0x244,	0x001E0285},
{0x258,	0x3002F   },
{0x158,	0x0       },
{0x158,	0x1       },
{0x37C,	0x00001063},
{0x380,	0x82A86030},
{0x384,	0x2861408B},
{0x388,	0x00130285},
{0x38C,	0x10630009},
{0x394,	0x400B82A8},
{0x600,	0x16CC78C },
{0x604,	0x3FFF83E8},
{0x608,	0xD0E     },
{0x154,	0x00000000},
{0x154,	0x80000000}
#elif LVDS_640x360P60
{0x700,	0x34900040},
{0x704,	0x100B8   },
{0x70C,	0x00004604},
{0x710,	0x004D000F},
{0x714,	0x20      },
{0x718,	0x00000102},
{0x71C,	0xA8002F  },
{0x720,	0x0       },
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x700,	0x34900040},
{0x70C,	0x00005E66},
{0x718,	0x00000202},
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x120,	0x5       },
{0x124,	0xFCB4280 },
{0x128,	0x11FC20  },
{0x12C,	0x39      },
{0x130,	0x3C10    },
{0x134,	0x00000005},
{0x138,	0xFF8000  },
{0x13C,	0x0       },
{0x140,	0x10000   },
{0x20C,	0x24      },
{0x21C,	0x7D0     },
{0x224,	0x0       },
{0x228,	0xA00000  },
{0x22C,	0xFF01    },
{0x230,	0x1       },
{0x234,	0xCA033E10},
{0x238,	0x00000060},
{0x23C,	0x82E86030},
{0x244,	0x001E0285},
{0x258,	0x6003F   },
{0x158,	0x0       },
{0x158,	0x1       },
{0x37C,	0x00001063},
{0x380,	0x82A86030},
{0x384,	0x2861408B},
{0x388,	0x00130285},
{0x38C,	0x10630009},
{0x394,	0x400B82A8},
{0x600,	0x16CC78C },
{0x604,	0x3FFF83E8},
{0x608,	0xD0E     },
{0x154,	0x00000000},
{0x154,	0x80000000}
#elif RGB_640x360P60
{0x700,	0x780000A0},
{0x704,	0x11F0028 },
{0x70C,	0x00004604},
{0x710,	0x07CD1027},
{0x714,	0x1       },
{0x718,	0x00000102},
{0x71C,	0xA8000B  },
{0x720,	0x0       },
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x700,	0x780000A0},
{0x70C,	0x00004636},
{0x718,	0x00000002},
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x120,	0x5       },
{0x124,	0xFCB4280 },
{0x128,	0x11FC20  },
{0x12C,	0x39      },
{0x130,	0x3CF0    },
{0x134,	0x00000005},
{0x138,	0xFF8000  },
{0x13C,	0x0       },
{0x140,	0x10000   },
{0x174,	0xff      },
{0x20C,	0x24      },
{0x21C,	0x7D0     },
{0x224,	0x0       },
{0x228,	0x50000   },
{0x22C,	0xFF01    },
{0x230,	0x1       },
{0x234,	0xCA033E10},
{0x238,	0x00000060},
{0x244,	0x00120285},
{0x258,	0x6003F   },
{0x158,	0x0       },
{0x158,	0x1       },
{0x37C,	0x00001063},
{0x380,	0x82A86030},
{0x384,	0x2861408B},
{0x388,	0x00130285},
{0x38C,	0x10630009},
{0x394,	0x400B82A8},
{0x608,	0x50F     },
{0x154,	0x00000000},
{0x154,	0x80000000}
#elif LVDS_640x360P60_90M
{0x700,	0xDC900040},
{0x704,	0x702E5   },
{0x70C,	0x00004604},
{0x710,	0x004D000F},
{0x714,	0x20      },
{0x718,	0x00000102},
{0x71C,	0xA8002F  },
{0x720,	0x0       },
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x700,	0xDC900040},
{0x70C,	0x00005E66},
{0x718,	0x00000202},
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x120,	0x5       },
{0x124,	0xFCB4280 },
{0x128,	0x11FC20  },
{0x12C,	0x39      },
{0x130,	0x3C10    },
{0x134,	0x00000005},
{0x138,	0xFF8000  },
{0x13C,	0x0       },
{0x140,	0x10000   },
{0x20C,	0x24      },
{0x21C,	0x7D0     },
{0x224,	0x0       },
{0x228,	0xA00000  },
{0x22C,	0xFF01    },
{0x230,	0x1       },
{0x234,	0xCA033E10},
{0x238,	0x00000060},
{0x23C,	0x82E86030},
{0x244,	0x001E0285},
{0x258,	0x6003F   },
{0x158,	0x0       },
{0x158,	0x1       },
{0x37C,	0x00001063},
{0x380,	0x82A86030},
{0x384,	0x2861408B},
{0x388,	0x00130285},
{0x38C,	0x10630009},
{0x394,	0x400B82A8},
{0x600,	0x16CC78C },
{0x604,	0x3FFF83E8},
{0x608,	0xD0E     },
{0x154,	0x00000000},
{0x154,	0x80000000}
#elif RGB_640x360P60_90M
#if 1
// Generated by Vx_MIPIin_RGBo_borderless_20181105_8.xlsm (2018/11/08 18:32:18)
// Address	Value
{0x700,	0x780000A0},
{0x704,	0x1F0029  },
{0x70C,	0x00004604},
{0x710,	0x07CD1027},
{0x714,	0x0       },
{0x718,	0x00000102},
{0x71C,	0xA8000B  },
{0x720,	0x0       },
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x700,	0x780000A0},
{0x70C,	0x00004636},
{0x718,	0x00000202},
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x120,	0x5       },
{0x124,	0x104B4280},
{0x128,	0x11FC20  },
{0x12C,	0x3B      },
{0x130,	0x3CF8    },// org 0x3CF0
{0x134,	0x00000005},
{0x138,	0xFF8000  },
{0x13C,	0x0       },
{0x140,	0x10000   },
{0x174,	0xff      },
{0x20C,	0x24      },
{0x21C,	0x7D0     },
{0x224,	0x0       },
{0x228,	0x50000   },
{0x22C,	0xFF01    },
{0x230,	0x1       },
{0x234,	0xCA033E10},
{0x238,	0x00000060},
{0x244,	0x00120285},
{0x258,	0x60041   },
{0x158,	0x0       },
{0x158,	0x1       },
{0x37C,	0x00001063},
{0x380,	0x82A86030},
{0x384,	0x2861408B},
{0x388,	0x00130285},
{0x38C,	0x10630009},
{0x394,	0x400B82A8},
{0x608,	0x50F     },
{0x154,	0x00000000},
{0x154,	0x80000000}
// QL test
#else
// Generated by Vx_MIPIin_RGBo_Borderless_register22remake_20190218.xlsm (2019/02/18 9:49:58)
// Address	Value
{0x700,	0x8DF000A0},
{0x704,	0x1F0603  },
{0x70C,	0x00004604},
{0x710,	0x07CD1027},
{0x714,	0x0       },
{0x718,	0x00000102},
{0x71C,	0x1A8000B },
{0x720,	0x0       },
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x700,	0x8DF000A0},
{0x70C,	0x00004636},
{0x718,	0x00000202},
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x120,	0x5       },
{0x124,	0x104B4280},
{0x128,	0x11FC20  },
{0x12C,	0x3B      },
{0x130,	0x3CF0    },
{0x134,	0x00000005},
{0x138,	0xFF8000  },
{0x13C,	0x0       },
{0x140,	0x10000   },
{0x174,	0xff      },
{0x20C,	0x24      },
{0x21C,	0x7D0     },
{0x224,	0x0       },
{0x228,	0x50000   },
{0x22C,	0xFF01    },
{0x230,	0x1       },
{0x234,	0xCA033E10},
{0x238,	0x00000060},
{0x244,	0x00120285},
{0x258,	0x50041   },
{0x158,	0x0       },
{0x158,	0x1       },
{0x37C,	0x00001063},
{0x380,	0x82A86030},
{0x384,	0x2861408B},
{0x388,	0x00130285},
{0x38C,	0x10630009},
{0x394,	0x400B82A8},
{0x608,	0x50F     },
{0x154,	0x00000000},
{0x154,	0x80000000}
#endif
#elif RGB_640x360P50
// Generated by Vx_MIPIin_RGBo_borderless_20181211_19.xlsm (2018/12/11 20:21:00)
//Address	Value
{0x700,	0x9C0000A0},
{0x704,	0x1F002C  },
{0x70C,	0x00004604},
{0x710,	0x07CD1027},
{0x714,	0x0       },
{0x718,	0x00000102},
{0x71C,	0xA8000B  },
{0x720,	0x0       },
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x700,	0x9C0000A0},
{0x70C,	0x00004636},
{0x718,	0x00000202},
{0x154,	0x00000000},
{0x154,	0x80000000},
{0x120,	0x5       },
{0x124,	0x104B4280},
{0x128,	0x11FC20  },
{0x12C,	0x3B      },
{0x130,	0x3CF0    },
{0x134,	0x00000005},
{0x138,	0xFF8000  },
{0x13C,	0x0       },
{0x140,	0x10000   },
{0x174,	0xff      },
{0x20C,	0x24      },
{0x21C,	0x7D0     },
{0x224,	0x0       },
{0x228,	0x50000   },
{0x22C,	0xFF01    },
{0x230,	0x1       },
{0x234,	0xCA033E10},
{0x238,	0x00000060},
{0x244,	0x00120285},
{0x258,	0x60041   },
{0x158,	0x0       },
{0x158,	0x1       },
{0x37C,	0x00001063},
{0x380,	0x82A86030},
{0x384,	0x2861408B},
{0x388,	0x00130285},
{0x38C,	0x10630009},
{0x394,	0x400B82A8},
{0x608,	0x50F     },
{0x154,	0x00000000},
{0x154,	0x80000000}
#endif
};

void ql_chip_init(struct msm_fb_data_type *mfd)
{
  //add your init code from the script here.
	int i;
	int reg_addr;
	int data;
	printk(KERN_ERR "ql_chip_init\n");
	for (i=0; i<(sizeof(vx3_init_data)/sizeof(struct chip_init_data)); i++) {
		reg_addr = vx3_init_data[i].reg_addr;
		data = vx3_init_data[i].data;
//		if (mfd == NULL) {
			ql_i2c_write(reg_addr, data, 4); 
//		} else {
//			ql_mipi_write(mfd, reg_addr, data, 4);
//  	    }

		if(reg_addr == 0x130)
			printk("eztest QL------------->addr:%x data:%x\n",reg_addr,data);
		if ((reg_addr == 0x154) && (data == 0x80000000)) {
			//doing init done.
			usleep(1000*1);
		}
	}

    ql_i2c_release();

	return;
}

///i2c access
struct i2c_client *i2c_quick_client = NULL;

/* quickvx i2c address is 0x64 (depending on gpio pin)
 * The platform has to define i2c_board_info and call i2c_register_board_info()

static struct i2c_board_info i2c_quickvx_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("i2c_quickvx", 0x64),
	},
};
*/

#define CONTROL_BYTE_GEN       (0x09u)
#define CONTROL_BYTE_DCS       (0x08u)
#define CONTROL_BYTE_I2C_RELEASE (0x0u)

#define QL_I2C_RELEASE  {\
		CONTROL_BYTE_I2C_RELEASE, \
}


int ql_i2c_release(void)
{
	  int write_size;
	  int ret = -1;
	  char buf[] = QL_I2C_RELEASE;

	write_size = 1;
    if ((ret = i2c_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}
	  return 0;

}


#ifdef QL_VX_REG_I2C_DIRECT_ACCESS
#define CONTROL_BYTE_DA_WR     (0x0Au)
#define CONTROL_BYTE_DA_RD     (0x0Eu)

#define DA_QL_WRITE  {\
		CONTROL_BYTE_DA_WR, \
        0x00,  /* Address MS */\
        0x00,  /* Address LS */\
        0x00,  /* data LS */\
		0x00, \
	    0x00, \
        0x00,  /* data MS */\
    }

#define DA_QL_READ  {\
		CONTROL_BYTE_DA_RD, \
        0x00,  /* Address MS */\
        0x00,  /* Address LS */\
        0x00,  /* len MS */\
        0x00,  /* len LS */\
    }

int ql_i2c_read(u32 addr, u32 *val, u32 data_size) 
{
	u32 data;
    char buf[] = DA_QL_READ;
	char rx[10];
	int ret = -1;
	int write_size;

  	  buf[1] = (addr >> 8) & 0xff;
  	  buf[2] = addr & 0xff;
  	  buf[3] = (data_size >> 8) & 0xff;
  	  buf[4] = data_size & 0xff;

	  write_size = 5;

    /* Read register */
    if ((ret = i2c_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}
	//return number of bytes or error
    if ((ret = i2c_master_recv( i2c_quick_client,
                     (char*)(&rx[0]),
                     data_size )) != data_size) {
		printk(KERN_ERR
		  "%s: i2c_master_recv failed (%d)!\n", __func__, ret);
		return -1;
	}

		data = rx[0];
		if (data_size > 1) 
			data |= (rx[1] << 8);
		if (data_size > 2)
			data |= (rx[2] << 16) | (rx[3] << 24);

		*val = data;

    	QL_DBG("a r0x%x=0x%x\n",addr,data);

	return 0;

}

int ql_i2c_write(long addr, long val, int data_size)
{
	  int ret = -1;
	  int write_size;
	  char buf[] = DA_QL_WRITE;

 		QL_DBG("a w0x%lx=0x%lx\n",addr,val);

	buf[1] = (uint8_t)(addr >> 8);  /* Address MS */
	buf[2] = (uint8_t)addr;  /* Address LS */

	buf[3] = val & 0xff;
	buf[4] = (data_size > 1) ? ((val >> 8) & 0xff) : 0;
	buf[5] = (data_size > 2) ? ((val >> 16) & 0xff) : 0;
	buf[6] = (data_size > 2) ? ((val >> 24) & 0xff) : 0;

	write_size = data_size + 3;
    if ((ret = i2c_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}
	  return 0;

}


#else


#define GEN_QL_CSR_OFFSET_LENGTH  {\
		CONTROL_BYTE_GEN, \
        0x29,  /* Data ID */\
        0x05,  /* Vendor Id 1 */\
        0x01,  /* Vendor Id 2 */\
        0x41,  /* Vendor Unique Command */\
        0x00,  /* Address LS */\
        0x00,  /* Address MS */\
        0x00,  /* Length LS */\
        0x00,  /* Length MS */\
    }

#define GEN_QL_CSR_WRITE  {\
		CONTROL_BYTE_GEN, \
        0x29,  /* Data ID */\
        0x05,  /* Vendor Id 1 */\
        0x01,  /* Vendor Id 2 */\
        0x40,  /* Vendor Unique Command */\
        0x00,  /* Address LS */\
        0x00,  /* Address MS */\
        0x00,  /* data LS */\
		0x00, \
	    0x00, \
        0x00,  /* data MS */\
    }



int ql_i2c_read(u32 addr, u32 *val, u32 data_size) 
{
	u32 data;
    char buf[] = GEN_QL_CSR_OFFSET_LENGTH;
	char rx[10];
	int ret = -1;
	int write_size;

  	  buf[5] = addr & 0xff;
  	  buf[6] = (addr >> 8) & 0xff;
  	  buf[7] = data_size & 0xff;
  	  buf[8] = (data_size >> 8) & 0xff;

	  write_size = 9;

    if ((ret = i2c_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}

	// generic read request 0x24 to send generic read command 
	write_size = 4;

    buf[0] = CONTROL_BYTE_GEN;
    buf[1] =    0x24;  /* Data ID */
    buf[2] =    0x05;  /* Vendor Id 1 */
    buf[3] =    0x01;  /* Vendor Id 2 */

    if ((ret = i2c_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}

	//return number of bytes or error
    if ((ret = i2c_master_recv( i2c_quick_client,
                     (char*)(&rx[0]),
                     data_size )) != data_size) {
		printk(KERN_ERR
		  "%s: i2c_master_recv failed (%d)!\n", __func__, ret);
		return -1;
	}


		data = rx[0];
		if (data_size > 1) 
			data |= (rx[1] << 8);
		if (data_size > 2)
			data |= (rx[2] << 16) | (rx[3] << 24);

		*val = data;

//    	QL_DBG("b r0x%x=0x%x\n",addr,data);

	return 0;

}

int ql_i2c_write(long addr, long val, int data_size)
{
	  int write_size;
	  int ret = -1;
	  char buf[] = GEN_QL_CSR_WRITE;

// 		QL_DBG("b w0x%lx=0x%lx\n",addr,val);

	buf[5] = (uint8_t)addr;  /* Address LS */
	buf[6] = (uint8_t)(addr >> 8);  /* Address MS */

	buf[7] = val & 0xff;
	buf[8] = (data_size > 1) ? ((val >> 8) & 0xff) : 0;
	buf[9] = (data_size > 2) ? ((val >> 16) & 0xff) : 0;
	buf[10] = (data_size > 2) ? ((val >> 24) & 0xff) : 0;

	write_size = data_size + 7;
    if ((ret = i2c_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}
	  return 0;

}

#endif

#ifdef QL_VX_LCD_I2C_for_MIPI_ACCESS
// Use I2C to send mipi command to LCD. 

//Generic Long Write
int ql_i2c_for_mipi(int dtype, int vc, int data_size,  char *ptr)
{
   char buf[256];
   int write_size;
   int ret = -1;
   int i;

   switch (dtype) {
   case DTYPE_GEN_LWRITE:
	buf[0] = CONTROL_BYTE_GEN;
	break;
   case DTYPE_DCS_WRITE:
   case DTYPE_DCS_WRITE1:
   case DTYPE_DCS_LWRITE:
	 buf[0] =  CONTROL_BYTE_DCS;
	 break;
   default:
 	QL_ERR("Error unknown data type (0x%x)\n",dtype);
	return -1;
	break;
   }
	buf[1] = ((vc & 0x3) << 6) | dtype ;  /*vc,  Data ID */

    /* Copy data */
    for(i = 0; i < data_size; i++)
    {
        buf[2+i] = *ptr++;
    }

 	write_size = data_size + 2;

    if ((ret = i2c_master_send( i2c_quick_client,
                     (char*)(&buf[0]),
                     write_size )) != write_size) {
		printk(KERN_ERR
		  "%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}
	return 0;

}

#endif

int vx5b3a_gpio_init(int en)
{
	int ret=0;

	ret = gpio_direction_output(964, 1);// eztest for power on kill
//	ret = gpio_direction_output(EN_V10_HUD, en);
//	printk("eztest vx5b3a----------->V10:%d set:%d\n",EN_V10_HUD,gpio_get_value(EN_V10_HUD));
	ret = gpio_direction_output(SYS_RST_N, 0);
	printk("eztest vx5b3a----------->RST:%d set:%d\n",SYS_RST_N,gpio_get_value(SYS_RST_N));
//	ret = gpio_direction_output(EN_V1P8_HUD, en) + (ret << 1);
//	printk("eztest vx5b3a----------->V1P8:%d set:%d\n",EN_V1P8_HUD,gpio_get_value(EN_V1P8_HUD));
//	ret = gpio_direction_output(EN_SYS_CLK, en) + (ret << 1); 
//	printk("eztest vx5b3a----------->CLK:%d set:%d\n",EN_SYS_CLK,gpio_get_value(EN_SYS_CLK));
	ret = gpio_direction_output(EN_DSI_CV, en) + (ret << 1);
//	printk("eztest vx5b3a----------->CV:%d set:%d\n",EN_DSI_CV,gpio_get_value(EN_DSI_CV));
	udelay(100);
	ret = gpio_direction_output(SYS_RST_N, 1);
	printk("eztest vx5b3a----------->RST:%d set:%d\n",SYS_RST_N,gpio_get_value(SYS_RST_N));
	return ret;
}


static int vx5b3a_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
//	struct property *prop;
	int err;
//	int ret;
//	int i,size;
//	struct device_node *np = client->dev.of_node;
//	int addresses[100];
//	int values[100];
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
#ifndef QL_CHIP_INIT_EXTERNAL
	u32 val = 0x8899;
#endif

	vx5b3a_gpio_init(1);
#if 0
	prop = of_find_property(np, "vx5b3a,addresses", NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	size = prop->length / sizeof(int);

	err = of_property_read_u32_array(np, "vx5b3a,addresses", addresses, size);
	if (err && (err != -EINVAL)) {
		dev_err(&client->dev, "Unable to read 'vx5b3a,addresses'\n");
		return err;
	}
	
	prop = of_find_property(np, "vx5b3a,values", NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	i = prop->length / sizeof(u32);
	if (i != size) {
		dev_err(&client->dev, "invalid 'vx5b3a,values' length should be same as addresses\n");
		return -EINVAL;
	}

	err = of_property_read_u32_array(np, "vx5b3a,values", values, i);
	if (err && (err != -EINVAL)) {
		dev_err(&client->dev, "Unable to read 'vx5b3a,values'\n");
		return err;
	}
#endif	

  i2c_quick_client = client;
  QL_DBG("eztest probing adapter: %s, address: 0x%x\n", adapter->name, client->addr);
  
#ifndef QL_CHIP_INIT_EXTERNAL
  err = ql_i2c_read(0x4fe, &val, 2); 
  //ql_i2c_read(0x0, &val, 2); 
  QL_DBG("eztest VEE ID 0x%x\n", val);
  ql_i2c_write(0x448, 0x12345678, 4); 
  ql_i2c_read(0x448, &val, 4);
  QL_DBG("eztest r/w test 0x%x\n", val);
	if(val != 0x12345678){
		ql_i2c_release();
		return -1;
	}

//  ql_i2c_read(0x108, &val, 4);
//printk("eztest vx5b3a probe--------->108 read:0x%x err:%d\n", val, err);
//  ql_i2c_read(0x114, &val, 4);
//printk("eztest vx5b3a probe--------->114 read:0x%x err:%d\n", val, err);
//  err = ql_i2c_read(0x402, &val, 2);
//printk("eztest vx5b3a probe--------->bef 402 read:0x%x err:%d\n", val, err);

  //init client.
  ql_i2c_read(0x218, &val, 4); 
  QL_DBG("Turn around timeout counter register 0x%x\n", val);
  ql_i2c_read(0x204, &val, 4); 
  QL_DBG("Interrupt status register 0x%x\n", val);
  ql_i2c_write(0x204, 0xFFFFFFFF, 4); 
  ql_i2c_write(0x218, 0x17, 4); 
  ql_i2c_write(0x238, 0x0060, 4); 
  ql_i2c_write(0x234, 0xCA033A10 , 4); 
  ql_i2c_write(0x240, 0x28614088, 4); 
  ql_chip_init(NULL); // first init don't have struct msm_fb_data_type *mfd yet
//  ql_i2c_write(0x402, 0xF800, 2); 
//  ql_i2c_write(0x412, 0xE6, 2); 
  ql_i2c_release();
#endif
	mdelay(10);
  err = ql_i2c_read(0x204, &val, 4);
printk("eztest vx5b3a probe################VX 204 read:0x%x err:%d\n", val, err);
#if 1//vysor test
	gpio_set_value(EN_DSI_CV, 0);
	printk("eztest vx5b3a----------->EN_DSI_CV:%d set:%d\n",EN_DSI_CV,gpio_get_value(EN_DSI_CV));
	gpio_set_value(SYS_RST_N, 0);
	printk("eztest vx5b3a----------->RST:%d set:%d\n",SYS_RST_N,gpio_get_value(SYS_RST_N));
#endif
//  err = ql_i2c_read(0x402, &val, 2);
//printk("eztest vx5b3a probe--------->aft 402 read:0x%x err:%d\n", val, err);
//  err = ql_i2c_read(0x412, &val, 1);
//printk("eztest vx5b3a probe--------->aft 412 read:0x%x err:%d\n", val, err);
//  err = ql_i2c_read(0x148, &val, 2);
//printk("eztest vx5b3a probe--------->aft 148 read:0x%x err:%d\n", val, err);
//	mdelay(100);
//	panel_hud_disp_on();
  return 0;
}

static int vx5b3a_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id vx5b3a_id[] = {
	{"vx5b3a", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, vx5b3a_id);

#ifdef CONFIG_OF
static struct of_device_id vx5b3a_match_table[] = {
	{ .compatible = "quicklogic,vx5b3a",},
	{ },
};
#else
#define vx5b3a_match_table NULL
#endif

static struct i2c_driver vx5b3a_driver = {
	.probe = vx5b3a_probe,
	.remove = vx5b3a_remove,
	.driver = {
		.name = "vx5b3a",
		.owner = THIS_MODULE,
		.of_match_table = vx5b3a_match_table,
	},
	.id_table = vx5b3a_id,
};

static int __init vx5b3a_init(void)
{
	return i2c_add_driver(&vx5b3a_driver);
}
module_init(vx5b3a_init);

static void __exit vx5b3a_exit(void)
{
	i2c_del_driver(&vx5b3a_driver);
}
module_exit(vx5b3a_exit);

MODULE_DESCRIPTION("QuickLogic vx5b3a TouchScreen driver");
MODULE_LICENSE("GPL v2");
