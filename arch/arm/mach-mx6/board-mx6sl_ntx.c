/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/input/zforce_ts.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max17135.h>
#include <sound/wm8962.h>
#include <sound/pcm.h>
#include <linux/power/sabresd_battery.h>
#include <../drivers/misc/ntx-misc.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6sl.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <linux/usbplugevent.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6sl_common.h"
#include "board-mx6sl_ntx.h"

#include "ntx_hwconfig.h"


#define GDEBUG 0
#include <linux/gallen_dbg.h>

volatile unsigned gMX6SL_NTX_ACIN_PG = IMX_GPIO_NR(4, 20);	/* FEC_MDIO */
volatile unsigned gMX6SL_NTX_CHG = IMX_GPIO_NR(4, 21);	/* FEC_TX_CLK */
volatile unsigned gMX6SL_MSP_INT = IMX_GPIO_NR(4, 19);	/* FEC_RX_ER */
volatile unsigned gMX6SL_PWR_SW = IMX_GPIO_NR(4, 25);	/* FEC_CRS_DV */
volatile unsigned gMX6SL_IR_TOUCH_INT = IMX_GPIO_NR(4, 24);	/* FEC_TXD0 */
volatile unsigned gMX6SL_IR_TOUCH_RST = IMX_GPIO_NR(4, 17);	/* FEC_RXD0 */
volatile unsigned gMX6SL_HALL_EN = IMX_GPIO_NR(4, 23);	/* FEC_MDC */
volatile unsigned gMX6SL_ON_LED = IMX_GPIO_NR(4, 22);	/* FEC_TX_EN */
volatile unsigned gMX6SL_CHG_LED = IMX_GPIO_NR(4, 16);	/* FEC_TXD1 */
volatile unsigned gMX6SL_ACT_LED = IMX_GPIO_NR(4, 26);	/* FEC_REF_CLK */
volatile unsigned gMX6SL_WIFI_3V3 = IMX_GPIO_NR(5, 0);	/* SD2_DAT7 */
volatile unsigned gMX6SL_WIFI_RST = IMX_GPIO_NR(4, 27);	/* SD2_RST */
volatile unsigned gMX6SL_WIFI_INT = IMX_GPIO_NR(4, 29);	/* SD2_DAT6 */

static int spdc_sel;
static int max17135_regulator_init(struct max17135 *max17135);
//struct clk *extern_audio_root;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int __init mx6sl_ntx_init_pfuze100(u32 int_gpio);
extern void tle4913_init(void);

#define _MYINIT_DATA	
#define _MYINIT_TEXT	
volatile static unsigned char _MYINIT_DATA *gpbHWCFG_paddr;
//volatile unsigned char *gpbHWCFG_vaddr;
volatile unsigned long _MYINIT_DATA gdwHWCFG_size;
volatile int _MYINIT_DATA giBootPort;

volatile NTX_HWCONFIG *gptHWCFG;

static void * _MemoryRequest(void *addr, u32 len, const char * name)
{
    void * mem = NULL;
    do {
        printk(KERN_DEBUG "***%s:%d: request memory region! addr=%p, len=%hd***\n",
                __FUNCTION__, __LINE__, addr, len);
        if (!request_mem_region((u32)addr, len, name)) {
            printk(KERN_CRIT "%s():  request memory region failed! addr=%p, len %hd\n",__FUNCTION__, addr, len);
            break;
        }
        mem = (void *) ioremap_nocache((u32)addr, len);
        if (!mem) {
            printk(KERN_CRIT "***%s:%d: could not ioremap %s***\n", __FUNCTION__, __LINE__, name);
            release_mem_region((u32)addr, len);
            break;
        }
    } while (0);
    return mem;
}

int gIsCustomerUi;
static int _MYINIT_TEXT hwcfg_p_setup(char *str)
{
	gpbHWCFG_paddr = (unsigned char *)simple_strtoul(str,NULL,0);
	if(NULL==gptHWCFG) {
		gptHWCFG = (NTX_HWCONFIG *)_MemoryRequest((void *)gpbHWCFG_paddr, gdwHWCFG_size, "hwcfg_p");
		if(!gptHWCFG) {
			return 0;
		}
	}
	printk("%s() hwcfg_p=%p,vaddr=%p,size=%d\n",__FUNCTION__,
		gpbHWCFG_paddr,gptHWCFG,(int)gdwHWCFG_size);
		
	if(NtxHwCfg_ChkCfgHeaderEx((NTX_HWCONFIG *)gptHWCFG,1)>=0) {
		//printk("%s() ntx_hwconfig load success !!\n",__FUNCTION__);
		printk("%s() pcba=\"%s\" !!\n",__FUNCTION__,NtxHwCfg_GetCfgFldStrVal((NTX_HWCONFIG *)gptHWCFG,HWCFG_FLDIDX_PCB));
		gIsCustomerUi = NtxHwCfg_GetCfgFldVal(gptHWCFG,HWCFG_FLDIDX_UISTYLE);
	}
	else {
		printk("%s() ntx_hwconfig check fail !!\n",__FUNCTION__);
	}
	
	return 1;
}

static int _MYINIT_TEXT hwcfg_size_setup(char *str)
{
	gdwHWCFG_size = (unsigned long)simple_strtoul(str,NULL,0);
	printk("%s() hwcfg_szie=%d\n",__FUNCTION__,(int)gdwHWCFG_size);
	return 1;
}

static int _MYINIT_TEXT boot_port_setup(char *str)
{
	giBootPort = (int)simple_strtoul(str,NULL,0);
	printk("%s() boot_port=%d\n",__FUNCTION__,giBootPort);
	return 1;
}

static void _parse_cmdline(void)
{
	static int iParseCnt = 0;
	char *pcPatternStart,*pcPatternVal,*pcPatternValEnd,cTempStore;
	unsigned long ulPatternLen;

	char *szParsePatternA[]={"hwcfg_sz=","hwcfg_p=","boot_port="};
	int ((*pfnDispatchA[])(char *str))={hwcfg_size_setup,hwcfg_p_setup,boot_port_setup };
		
	int i;
	char *pszCmdLineBuf;
	
	
	if(iParseCnt++>0) {
		printk("%s : cmdline parse already done .\n",__FUNCTION__);
		return ;
	}
	//printk("%s():cmdline(%d)=%s\n",__FUNCTION__,strlen(saved_command_line),saved_command_line);
		
	pszCmdLineBuf = kmalloc(strlen(saved_command_line)+1,GFP_KERNEL);
	//ASSERT(pszCmdLineBuf);
	strcpy(pszCmdLineBuf,saved_command_line);
	printk("%s():cp cmdline=%s\n",__FUNCTION__,pszCmdLineBuf);
	
	for(i=0;i<sizeof(szParsePatternA)/sizeof(szParsePatternA[0]);i++) {
		ulPatternLen = strlen(szParsePatternA[i]);
		pcPatternStart = strstr(pszCmdLineBuf,szParsePatternA[i]);
		if(pcPatternStart) {
			pcPatternVal=pcPatternStart + ulPatternLen ;
			pcPatternValEnd = strchr(pcPatternVal,' ');
			cTempStore='\0';
			if(pcPatternValEnd) {
				cTempStore = *pcPatternValEnd;
				*pcPatternValEnd = '\0';
			}
			//printk("%s():pattern \"%s\" ,val=\"%s\"\n",__FUNCTION__,szParsePatternA[i],pcPatternVal);
			pfnDispatchA[i](pcPatternVal);
			if(pcPatternValEnd) {
				*pcPatternValEnd = cTempStore;
			}
		}
		else 
			printk ("[%s-%d] %s not found !!!\n",__func__,__LINE__,szParsePatternA[i]);
	}
	if (NULL == gptHWCFG) {
		printk ("[%s-%d] personal initial hw config.\n",__func__,__LINE__);
		gptHWCFG = (NTX_HWCONFIG *)kmalloc(sizeof(NTX_HWCONFIG), GFP_KERNEL);
		gptHWCFG->m_val.bTouchCtrl = 8;
	}
}



/* uart2 pins */
#if 0 // gallen disabled .
static iomux_v3_cfg_t mx6sl_uart2_pads[] = {
	MX6SL_PAD_SD2_DAT5__UART2_TXD,
	MX6SL_PAD_SD2_DAT4__UART2_RXD,
	MX6SL_PAD_SD2_DAT6__UART2_RTS,
	MX6SL_PAD_SD2_DAT7__UART2_CTS,
};
#endif

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

static int plt_sd_pad_change(unsigned int index, int clock)
{
	/* LOW speed is the default state of SD pads */
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd_pads_50mhz = NULL;

	u32 sd_pads_200mhz_cnt;
	u32 sd_pads_100mhz_cnt;
	u32 sd_pads_50mhz_cnt;

	switch (index) {
	case 0:
		sd_pads_200mhz = mx6sl_sd1_200mhz;
		sd_pads_100mhz = mx6sl_sd1_100mhz;
		sd_pads_50mhz = mx6sl_sd1_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6sl_sd1_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6sl_sd1_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6sl_sd1_50mhz);
		break;
	case 1:
		sd_pads_200mhz = mx6sl_sd2_200mhz;
		sd_pads_100mhz = mx6sl_sd2_100mhz;
		sd_pads_50mhz = mx6sl_sd2_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6sl_sd2_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6sl_sd2_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6sl_sd2_50mhz);
		break;
	case 2:
		sd_pads_200mhz = mx6sl_sd3_200mhz;
		sd_pads_100mhz = mx6sl_sd3_100mhz;
		sd_pads_50mhz = mx6sl_sd3_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6sl_sd3_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6sl_sd3_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6sl_sd3_50mhz);
		break;
	case 3:
		sd_pads_200mhz = mx6sl_brd_ntx_sd4_pads;
		sd_pads_100mhz = mx6sl_brd_ntx_sd4_pads;
		sd_pads_50mhz = mx6sl_brd_ntx_sd4_pads;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6sl_brd_ntx_sd4_pads);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6sl_brd_ntx_sd4_pads);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6sl_brd_ntx_sd4_pads);
		break;
		
	default:
		printk(KERN_ERR "no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_200mhz,
							sd_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_100mhz,
							sd_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_50mhz,
							sd_pads_50mhz_cnt);
	}
}


static const struct esdhc_platform_data mx6_ntx_isd_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.delay_line		= 0,
	.platform_pad_change = plt_sd_pad_change,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct esdhc_platform_data mx6_ntx_sd_wifi_data __initconst = {
	.cd_gpio		= MX6SL_WIFI_3V3,
	.wp_gpio 		= -1,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
//	.platform_pad_change = plt_sd_pad_change,
	.cd_type = ESDHC_CD_WIFI_PWR,
};

static const struct esdhc_platform_data mx6_ntx_q22_sd_wifi_data __initconst = {
	.cd_gpio		= IMX_GPIO_NR(4, 29),
	.wp_gpio 		= -1,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
//	.platform_pad_change = plt_sd_pad_change,
	.cd_type = ESDHC_CD_WIFI_PWR,
};

static const struct esdhc_platform_data mx6_ntx_esd_data __initconst = {
	.cd_gpio		= MX6SL_EXT_SD_CD,
	.wp_gpio		= -1,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.platform_pad_change = plt_sd_pad_change,
	.cd_type = ESDHC_CD_GPIO,
};

#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

#if 1
static struct regulator_consumer_supply ntx_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data ntx_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(ntx_vmmc_consumers),
	.consumer_supplies = ntx_vmmc_consumers,
};

static struct fixed_voltage_config ntx_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &ntx_vmmc_init,
};

static struct platform_device ntx_vmmc_reg_devices = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data = &ntx_vmmc_reg_config,
	},
};
#endif

static struct regulator_consumer_supply display_consumers[] = {
	{
		/* MAX17135 */
		.supply = "DISPLAY",
	},
};

static struct regulator_consumer_supply vcom_consumers[] = {
	{
		/* MAX17135 */
		.supply = "VCOM",
	},
};

static struct regulator_consumer_supply v3p3_consumers[] = {
	{
		/* MAX17135 */
		.supply = "V3P3",
	},
};

static struct regulator_init_data max17135_init_data[] = {
	{
		.constraints = {
			.name = "DISPLAY",
			.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(display_consumers),
		.consumer_supplies = display_consumers,
	}, {
		.constraints = {
			.name = "GVDD",
			.min_uV = V_to_uV(20),
			.max_uV = V_to_uV(20),
		},
	}, {
		.constraints = {
			.name = "GVEE",
			.min_uV = V_to_uV(-22),
			.max_uV = V_to_uV(-22),
		},
	}, {
		.constraints = {
			.name = "HVINN",
			.min_uV = V_to_uV(-22),
			.max_uV = V_to_uV(-22),
		},
	}, {
		.constraints = {
			.name = "HVINP",
			.min_uV = V_to_uV(20),
			.max_uV = V_to_uV(20),
		},
	}, {
		.constraints = {
			.name = "VCOM",
			.min_uV = mV_to_uV(-4325),
			.max_uV = mV_to_uV(-500),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(vcom_consumers),
		.consumer_supplies = vcom_consumers,
	}, {
		.constraints = {
			.name = "VNEG",
			.min_uV = V_to_uV(-15),
			.max_uV = V_to_uV(-15),
		},
	}, {
		.constraints = {
			.name = "VPOS",
			.min_uV = V_to_uV(15),
			.max_uV = V_to_uV(15),
		},
	}, {
		.constraints = {
			.name = "V3P3",
			.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(v3p3_consumers),
		.consumer_supplies = v3p3_consumers,
	},
};

static const struct anatop_thermal_platform_data
	mx6sl_anatop_thermal_data __initconst = {
			.name = "anatop_thermal",
	};

static struct platform_device max17135_sensor_device = {
	.name = "max17135_sensor",
	.id = 0,
};

static struct max17135_platform_data max17135_pdata __initdata = {
	.vneg_pwrup = 1,
	.gvee_pwrup = 1,
	.vpos_pwrup = 2,
	.gvdd_pwrup = 1,
	.gvdd_pwrdn = 1,
	.vpos_pwrdn = 2,
	.gvee_pwrdn = 1,
	.vneg_pwrdn = 1,
	.gpio_pmic_pwrgood = MX6SL_BRD_EPDC_PWRSTAT,
	.gpio_pmic_vcom_ctrl = MX6SL_BRD_EPDC_VCOM,
	.gpio_pmic_wakeup = MX6SL_BRD_EPDC_PMIC_WAKE,
	.gpio_pmic_v3p3 = MX6SL_BRD_EPDC_PWRCTRL0,
	.gpio_pmic_intr = MX6SL_BRD_EPDC_PMIC_INT,
	.regulator_init = max17135_init_data,
	.init = max17135_regulator_init,
};

static int __init max17135_regulator_init(struct max17135 *max17135)
{
#if 0
	struct max17135_platform_data *pdata = &max17135_pdata;
	int i, ret;

	max17135->gvee_pwrup = pdata->gvee_pwrup;
	max17135->vneg_pwrup = pdata->vneg_pwrup;
	max17135->vpos_pwrup = pdata->vpos_pwrup;
	max17135->gvdd_pwrup = pdata->gvdd_pwrup;
	max17135->gvdd_pwrdn = pdata->gvdd_pwrdn;
	max17135->vpos_pwrdn = pdata->vpos_pwrdn;
	max17135->vneg_pwrdn = pdata->vneg_pwrdn;
	max17135->gvee_pwrdn = pdata->gvee_pwrdn;

	max17135->max_wait = pdata->vpos_pwrup + pdata->vneg_pwrup +
		pdata->gvdd_pwrup + pdata->gvee_pwrup;

	max17135->gpio_pmic_pwrgood = pdata->gpio_pmic_pwrgood;
	max17135->gpio_pmic_vcom_ctrl = pdata->gpio_pmic_vcom_ctrl;
	max17135->gpio_pmic_wakeup = pdata->gpio_pmic_wakeup;
	max17135->gpio_pmic_v3p3 = pdata->gpio_pmic_v3p3;
	max17135->gpio_pmic_intr = pdata->gpio_pmic_intr;

	gpio_request(max17135->gpio_pmic_wakeup, "epdc-pmic-wake");
	gpio_direction_output(max17135->gpio_pmic_wakeup, 0);

	gpio_request(max17135->gpio_pmic_vcom_ctrl, "epdc-vcom");
	gpio_direction_output(max17135->gpio_pmic_vcom_ctrl, 0);

	gpio_request(max17135->gpio_pmic_v3p3, "epdc-v3p3");
	gpio_direction_output(max17135->gpio_pmic_v3p3, 0);

	gpio_request(max17135->gpio_pmic_intr, "epdc-pmic-int");
	gpio_direction_input(max17135->gpio_pmic_intr);

	gpio_request(max17135->gpio_pmic_pwrgood, "epdc-pwrstat");
	gpio_direction_input(max17135->gpio_pmic_pwrgood);

	max17135->vcom_setup = false;
	max17135->init_done = false;

	for (i = 0; i < MAX17135_NUM_REGULATORS; i++) {
		ret = max17135_register_regulator(max17135, i,
			&pdata->regulator_init[i]);
		if (ret != 0) {
			printk(KERN_ERR"max17135 regulator init failed: %d\n",
				ret);
			return ret;
		}
	}

	/*
	 * TODO: We cannot enable full constraints for now, since
	 * it results in the PFUZE regulators being disabled
	 * at the end of boot, which disables critical regulators.
	 */
	/*regulator_has_full_constraints();*/
#endif
	return 0;
}

static int mx6_ntx_spi_cs[] = {
	MX6_BRD_ECSPI1_CS0,
};

static const struct spi_imx_master mx6_ntx_spi_data __initconst = {
	.chipselect     = mx6_ntx_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6_ntx_spi_cs),
};

static void spi_device_init(void)
{
}

static struct imx_ssi_platform_data mx6_sabresd_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

#if 0
static struct mxc_audio_platform_data wm8962_data;

static struct platform_device mx6_sabresd_audio_wm8962_device = {
	.name = "imx-wm8962",
};

static struct wm8962_pdata wm8962_config_data = {

};

static int wm8962_clk_enable(int enable)
{
	if (enable)
		clk_enable(extern_audio_root);
	else
		clk_disable(extern_audio_root);

	return 0;
}

static int mxc_wm8962_init(void)
{
	struct clk *pll4;
	int rate;

	extern_audio_root = clk_get(NULL, "extern_audio_clk");
	if (IS_ERR(extern_audio_root)) {
		pr_err("can't get extern_audio_root clock.\n");
		return PTR_ERR(extern_audio_root);
	}

	pll4 = clk_get(NULL, "pll4");
	if (IS_ERR(pll4)) {
		pr_err("can't get pll4 clock.\n");
		return PTR_ERR(pll4);
	}

	clk_set_parent(extern_audio_root, pll4);

	rate = clk_round_rate(extern_audio_root, 26000000);
	clk_set_rate(extern_audio_root, rate);

	wm8962_data.sysclk = rate;
	/* set AUDMUX pads to 1.8v */
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_MCLK,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_RXD,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_TXC,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_TXD,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_TXFS,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);

	return 0;
}

static struct mxc_audio_platform_data wm8962_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = MX6_BRD_HEADPHONE_DET,
	.hp_active_low = 1,
	.mic_gpio = -1,
	.mic_active_low = 1,
	.init = mxc_wm8962_init,
	.clock_enable = wm8962_clk_enable,
};

static struct regulator_consumer_supply sabresd_vwm8962_consumers[] = {
	REGULATOR_SUPPLY("SPKVDD1", "1-001a"),
	REGULATOR_SUPPLY("SPKVDD2", "1-001a"),
};

static struct regulator_init_data sabresd_vwm8962_init = {
	.constraints = {
		.name = "SPKVDD",
		.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		.boot_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(sabresd_vwm8962_consumers),
	.consumer_supplies = sabresd_vwm8962_consumers,
};

static struct fixed_voltage_config sabresd_vwm8962_reg_config = {
	.supply_name	= "SPKVDD",
	.microvolts		= 4325000,
	.gpio			= -1,
	.enabled_at_boot = 1,
	.init_data		= &sabresd_vwm8962_init,
};

static struct platform_device sabresd_vwm8962_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id		= 4,
	.dev	= {
		.platform_data = &sabresd_vwm8962_reg_config,
	},
};

static int __init imx6q_init_audio(void)
{
	platform_device_register(&sabresd_vwm8962_reg_devices);
	mxc_register_device(&mx6_sabresd_audio_wm8962_device,
			    &wm8962_data);
	imx6q_add_imx_ssi(1, &mx6_sabresd_ssi_pdata);

	return 0;
}

static int spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;
	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);
	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,
	.spdif_rx		= 0,
	.spdif_clk_44100	= 1,
	.spdif_clk_48000	= -1,
	.spdif_div_44100	= 23,
	.spdif_clk_set_rate	= spdif_clk_set_rate,
	.spdif_clk		= NULL,
};

int hdmi_enabled;
static int __init hdmi_setup(char *__unused)
{
	hdmi_enabled = 1;
	return 1;
}
__setup("hdmi", hdmi_setup);

static iomux_v3_cfg_t mx6sl_sii902x_hdmi_pads_enabled[] = {
	MX6SL_PAD_LCD_RESET__GPIO_2_19,
	MX6SL_PAD_EPDC_PWRCTRL3__GPIO_2_10,
};

static int sii902x_get_pins(void)
{
	/* Sii902x HDMI controller */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_sii902x_hdmi_pads_enabled, \
		ARRAY_SIZE(mx6sl_sii902x_hdmi_pads_enabled));

	/* Reset Pin */
	gpio_request(MX6_BRD_LCD_RESET, "disp0-reset");
	gpio_direction_output(MX6_BRD_LCD_RESET, 1);

	/* Interrupter pin GPIO */
	gpio_request(MX6SL_BRD_EPDC_PWRCTRL3, "disp0-detect");
	gpio_direction_input(MX6SL_BRD_EPDC_PWRCTRL3);
       return 1;
}

static void sii902x_put_pins(void)
{
	gpio_free(MX6_BRD_LCD_RESET);
	gpio_free(MX6SL_BRD_EPDC_PWRCTRL3);
}

static void sii902x_hdmi_reset(void)
{
	gpio_set_value(MX6_BRD_LCD_RESET, 0);
	msleep(10);
	gpio_set_value(MX6_BRD_LCD_RESET, 1);
	msleep(10);
}

static struct fsl_mxc_lcd_platform_data sii902x_hdmi_data = {
       .ipu_id = 0,
       .disp_id = 0,
       .reset = sii902x_hdmi_reset,
       .get_pins = sii902x_get_pins,
       .put_pins = sii902x_put_pins,
};
#endif

static struct imxi2c_platform_data mx6_ntx_i2c0_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_ntx_i2c1_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_ntx_i2c2_data = {
	.bitrate = 100000,
};

static struct zforce_ts_platdata zforce_ts_data = {
	.x_max = 600,
	.y_max = 800,
//	.gpio_int = TOUCH_INT,
//	.gpio_rst = IR_TOUCH_RST,
};

static struct i2c_board_info i2c_zforce_ir_touch_binfo = {
//	.type = "zforce-ir-touch",
	.type = "zforce-ts",
	.addr = 0x50,
	.platform_data = &zforce_ts_data,
 	//.platform_data = MX6SL_IR_TOUCH_INT,
 	//.irq = gpio_to_irq(MX6SL_IR_TOUCH_INT),
};

static struct i2c_board_info i2c_sysmp_msp430_binfo = {
	.type = "msp430",
	.addr = 0x43,
	//.irq = gpio_to_irq(MX6SL_MSP_INT),
};

#if 0
static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
};
static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
	 	.type = "msp430",
	 	.addr = 0x43,
	 	.irq = gpio_to_irq(MX6SL_MSP_INT),
	},
};
#endif

static struct ntx_misc_platform_data ntx_misc_info;

static struct platform_device ntx_charger = {
	.name	= "pmic_battery",
	.id = 1,
	.dev            = {
		.platform_data = &ntx_misc_info,
	}
};

static struct platform_device ntx_light_ldm = {
	.name = "pmic_light",
	.id = 1,
};

static struct mxc_dvfs_platform_data mx6sl_ntx_dvfscore_data = {
#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	.reg_id			= "VDDCORE",
	.soc_id			= "VDDSOC",
#else
	.reg_id			= "cpu_vddgp",
	.soc_id			= "cpu_vddsoc",
	.pu_id			= "cpu_vddvpu",
#endif
	.clk1_id		= "cpu_clk",
	.clk2_id		= "gpc_dvfs_clk",
	.gpc_cntr_offset	= MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset	= MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset	= MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset	= MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask		= 0x1F800,
	.prediv_offset		= 11,
	.prediv_val		= 3,
	.div3ck_mask		= 0xE0000000,
	.div3ck_offset		= 29,
	.div3ck_val		= 2,
	.emac_val		= 0x08,
	.upthr_val		= 25,
	.dnthr_val		= 9,
	.pncthr_val		= 33,
	.upcnt_val		= 10,
	.dncnt_val		= 10,
	.delay_time		= 80,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_32M,
};

void __init early_console_setup(unsigned long base, struct clk *clk);

static const struct imxuart_platform_data mx6sl_ntx_uart1_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS | IMXUART_SDMA,
	.dma_req_rx = MX6Q_DMA_REQ_UART2_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART2_TX,
};

static inline void mx6_ntx_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL); /* DEBUG UART1 */
}

#if 0
static int mx6sl_ntx_fec_phy_init(struct phy_device *phydev)
{
	int val;

	/* power on FEC phy and reset phy */
	gpio_request(MX6_BRD_FEC_PWR_EN, "fec-pwr");
	gpio_direction_output(MX6_BRD_FEC_PWR_EN, 0);
	/* wait RC ms for hw reset */
	msleep(1);
	gpio_direction_output(MX6_BRD_FEC_PWR_EN, 1);

	/* check phy power */
	val = phy_read(phydev, 0x0);
	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6sl_ntx_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RMII,
};
#endif

static int epdc_get_pins(void)
{
	int ret = 0;

	/* Claim GPIOs for EPDC pins - used during power up/down */
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_0, "epdc_d0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_1, "epdc_d1");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_2, "epdc_d2");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_3, "epdc_d3");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_4, "epdc_d4");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_5, "epdc_d5");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_6, "epdc_d6");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_7, "epdc_d7");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_8 , "epdc_d8");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_9 , "epdc_d9");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_10, "epdc_d10");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_11, "epdc_d11");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_12, "epdc_d12");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_13, "epdc_d13");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_14, "epdc_d14");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_15, "epdc_d15");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDCLK, "epdc_gdclk");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDSP, "epdc_gdsp");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDOE, "epdc_gdoe");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDRL, "epdc_gdrl");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCLK, "epdc_sdclk");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDOE, "epdc_sdoe");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDLE, "epdc_sdle");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDSHR, "epdc_sdshr");
	ret |= gpio_request(MX6SL_BRD_EPDC_BDR0, "epdc_bdr0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE0, "epdc_sdce0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE1, "epdc_sdce1");
//	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE2, "epdc_sdce2");

	return ret;
}

static void epdc_put_pins(void)
{
	gpio_free(MX6SL_BRD_EPDC_SDDO_0);
	gpio_free(MX6SL_BRD_EPDC_SDDO_1);
	gpio_free(MX6SL_BRD_EPDC_SDDO_2);
	gpio_free(MX6SL_BRD_EPDC_SDDO_3);
	gpio_free(MX6SL_BRD_EPDC_SDDO_4);
	gpio_free(MX6SL_BRD_EPDC_SDDO_5);
	gpio_free(MX6SL_BRD_EPDC_SDDO_6);
	gpio_free(MX6SL_BRD_EPDC_SDDO_7);
	gpio_free(MX6SL_BRD_EPDC_SDDO_8); 
	gpio_free(MX6SL_BRD_EPDC_SDDO_9); 
	gpio_free(MX6SL_BRD_EPDC_SDDO_10);
	gpio_free(MX6SL_BRD_EPDC_SDDO_11);
	gpio_free(MX6SL_BRD_EPDC_SDDO_12);
	gpio_free(MX6SL_BRD_EPDC_SDDO_13);
	gpio_free(MX6SL_BRD_EPDC_SDDO_14);
	gpio_free(MX6SL_BRD_EPDC_SDDO_15);
	gpio_free(MX6SL_BRD_EPDC_GDCLK);
	gpio_free(MX6SL_BRD_EPDC_GDSP);
	gpio_free(MX6SL_BRD_EPDC_GDOE);
	gpio_free(MX6SL_BRD_EPDC_GDRL);
	gpio_free(MX6SL_BRD_EPDC_SDCLK);
	gpio_free(MX6SL_BRD_EPDC_SDOE);
	gpio_free(MX6SL_BRD_EPDC_SDLE);
	gpio_free(MX6SL_BRD_EPDC_SDSHR);
	gpio_free(MX6SL_BRD_EPDC_BDR0);
	gpio_free(MX6SL_BRD_EPDC_SDCE0);
	gpio_free(MX6SL_BRD_EPDC_SDCE1);
//	gpio_free(MX6SL_BRD_EPDC_SDCE2);
}

static void epdc_enable_pins(void)
{
	/* Configure MUX settings to enable EPDC use */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_epdc_enable_pads, \
				ARRAY_SIZE(mx6sl_brd_epdc_enable_pads));

	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_1);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_2);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_3);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_4);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_5);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_6);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_7);
	gpio_direction_input(MX6SL_BRD_EPDC_GDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_GDSP);
	gpio_direction_input(MX6SL_BRD_EPDC_GDOE);
	gpio_direction_input(MX6SL_BRD_EPDC_GDRL);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_SDOE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDLE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDSHR);
	gpio_direction_input(MX6SL_BRD_EPDC_BDR0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE1);
//	gpio_direction_input(MX6SL_BRD_EPDC_SDCE2);
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to
	 * GPIO and drive to 0. */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_epdc_disable_pads, \
				ARRAY_SIZE(mx6sl_brd_epdc_disable_pads));

	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_1, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_2, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_3, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_4, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_5, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_6, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_7, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDSP, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDOE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDRL, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDOE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDLE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDSHR, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_BDR0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE1, 0);
//	gpio_direction_output(MX6SL_BRD_EPDC_SDCE2, 0);
}
#if 1 //[

static struct fb_videomode ed060sc8_mode = {
.name = "E60SC8",
.refresh = 85,
.xres = 800,
.yres = 600,
.pixclock = 30000000,
.left_margin = 8,
.right_margin = 164,
.upper_margin = 4,
.lower_margin = 18,
.hsync_len = 4,
.vsync_len = 1,
.sync = 0,
.vmode = FB_VMODE_NONINTERLACED,
.flag = 0,
};

// for ED060XC5 release by Freescale Grace 20120726 .

static struct fb_videomode ed060xc1_mode = {
.name = "E60XC1",
.refresh = 85,
.xres = 1024,
.yres = 768,
.pixclock = 40000000,
.left_margin = 12,
.right_margin = 72,
.upper_margin = 4,
.lower_margin = 5,
.hsync_len = 8,
.vsync_len = 2,
.sync = 0,
.vmode = FB_VMODE_NONINTERLACED,
.flag = 0,
};

static struct fb_videomode ed060xc5_mode = {
.name = "E60XC5",
.refresh = 85,
.xres = 1024,
.yres = 758,
.pixclock = 40000000,
.left_margin = 12,
.right_margin = 76,
.upper_margin = 4,
.lower_margin = 5,
.hsync_len = 12,
.vsync_len = 2,
.sync = 0,
.vmode = FB_VMODE_NONINTERLACED,
.flag = 0,
};

static struct fb_videomode e60_v110_mode = {
.name = "E60_V110",
.refresh = 50,
.xres = 800,
.yres = 600,
.pixclock = 18604700,
.left_margin = 8,
.right_margin = 176,
.upper_margin = 4,
.lower_margin = 2,
.hsync_len = 4,
.vsync_len = 1,
.sync = 0,
.vmode = FB_VMODE_NONINTERLACED,
.flag = 0,
};

static struct fb_videomode ed050xxx_mode = {
	.name="ED050XXXX",
	.refresh=85,
	.xres=800,
	.yres=600,
	.pixclock=26666667,
	.left_margin=4,
	.right_margin=98,
	.upper_margin=4,
	.lower_margin=9,
	.hsync_len=8,
	.vsync_len=2,
	.sync=0,
	.vmode=FB_VMODE_NONINTERLACED,
	.flag=0,
};


static struct fb_videomode ed068og1_mode = {
.name = "E68OG1",
.refresh=85,
.xres=1440,
.yres=1080,
.pixclock=120000000,
.left_margin=32,
.right_margin=508,
.upper_margin=4,
.lower_margin=5,
.hsync_len=32,
.vsync_len=2,
.sync=0,
.vmode=FB_VMODE_NONINTERLACED,
.flag=0,
};


static struct imx_epdc_fb_mode panel_modes[] = {
////////////////////
{
& ed060sc8_mode,
4,            /* vscan_holdoff */
10,          /* sdoed_width */
20,          /* sdoed_delay */
10,          /* sdoez_width */
20,          /* sdoez_delay */
465,        /* gdclk_hp_offs */
250,        /* gdsp_offs changed delay to 8.3 uS */
0,            /* gdoe_offs */
8,            /* gdclk_offs changed delay to 4.5 SDCLK */
1,            /* num_ce */
},

////////////////////
{
& e60_v110_mode,
4,            /* vscan_holdoff */
10,          /* sdoed_width */
20,          /* sdoed_delay */
10,          /* sdoez_width */
20,          /* sdoez_delay */
465,        /* gdclk_hp_offs */
250,        /* gdsp_offs changed delay to 8.3 uS */
0,            /* gdoe_offs */
8,            /* gdclk_offs changed delay to 4.5 SDCLK */
1,            /* num_ce */
},

////////////////////
{
& ed060xc5_mode,
4,            /* vscan_holdoff */
10,          /* sdoed_width */
20,          /* sdoed_delay */
10,          /* sdoez_width */
20,          /* sdoez_delay */
524,        /* gdclk_hp_offs */
25,        /* gdsp_offs changed delay to 8.3 uS */
0,            /* gdoe_offs */
19,            /* gdclk_offs changed delay to 4.5 SDCLK */
1,            /* num_ce */
},

////////////////////
{
& ed060xc1_mode,
4,            /* vscan_holdoff */
10,          /* sdoed_width */
20,          /* sdoed_delay */
10,          /* sdoez_width */
20,          /* sdoez_delay */
492,        /* gdclk_hp_offs */
29,        /* gdsp_offs changed delay to 8.3 uS */
0,            /* gdoe_offs */
23,            /* gdclk_offs changed delay to 4.5 SDCLK */
1,            /* num_ce */
},

////////////////////
{
	&ed050xxx_mode, 	/* struct fb_videomode *mode */
		4, 	/* vscan_holdoff */
		10, 	/* sdoed_width */
		20, 	/* sdoed_delay */
		10, 	/* sdoez_width */
		20, 	/* sdoez_delay */
		420, 	/* GDCLK_HP */
		20, 	/* GDSP_OFF */
		0, 	/* GDOE_OFF */
		11, 	/* gdclk_offs */
		3, 	/* num_ce */
},	

////////////////////
{
& ed068og1_mode,
4,            /* vscan_holdoff */
10,          /* sdoed_width */
20,          /* sdoed_delay */
10,          /* sdoez_width */
20,          /* sdoez_delay */
831,        /* GDCLK_HP */
285,        /* GDSP_OFF */
0,            /* GDOE_OFF */
271,        /* gdclk_offs */
1,            /* num_ce */
},
};
 

#else //][!
static struct fb_videomode e60_v110_mode = {
	.name = "E60_V110",
	.refresh = 50,
	.xres = 800,
	.yres = 600,
	.pixclock = 18604700,
	.left_margin = 8,
	.right_margin = 178,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode e60_v220_mode = {
	.name = "E60_V220",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 30000000,
	.left_margin = 8,
	.right_margin = 164,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
	.refresh = 85,
	.xres = 800,
	.yres = 600,
};
static struct fb_videomode e060scm_mode = {
	.name = "E060SCM",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 26666667,
	.left_margin = 8,
	.right_margin = 100,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode e97_v110_mode = {
	.name = "E97_V110",
	.refresh = 50,
	.xres = 1200,
	.yres = 825,
	.pixclock = 32000000,
	.left_margin = 12,
	.right_margin = 128,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct imx_epdc_fb_mode panel_modes[] = {
	{
		&e60_v110_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		428,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e60_v220_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		465,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		9,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e060scm_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		419,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		5,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e97_v110_mode,
		8,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		632,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		3,      /* num_ce */
	}
};
#endif //]

static struct imx_epdc_fb_platform_data epdc_data = {
	.epdc_mode = panel_modes,
	.num_modes = ARRAY_SIZE(panel_modes),
	.get_pins = epdc_get_pins,
	.put_pins = epdc_put_pins,
	.enable_pins = epdc_enable_pins,
	.disable_pins = epdc_disable_pins,
};

typedef void (*usb_insert_handler) (char inserted);
usb_insert_handler mxc_misc_report_usb;

static void mxc_register_usb_plug(void* pk_cb)
{
	pmic_event_callback_t usb_plug_event;

//	usb_plug_event.param = (void *)1;
//	usb_plug_event.func = (void *)pk_cb;
//	pmic_event_subscribe(EVENT_VBUSVI, usb_plug_event);
	mxc_misc_report_usb = pk_cb;
	DBG_MSG("%s(),pk_cb=%p\n",__FUNCTION__,pk_cb);

}


extern int mxc_usb_plug_getstatus (void);


#define SW_USBPLUG	0x0C
static struct usbplug_event_platform_data usbplug_data = {
	.usbevent.type = EV_SW,
	.usbevent.code = SW_USBPLUG,
	.register_usbplugevent = mxc_register_usb_plug,
	.get_key_status = mxc_usb_plug_getstatus,
};

struct platform_device mxc_usb_plug_device = {
		.name = "usb_plug",
		.id = 0,
	};



static int spdc_get_pins(void)
{
	int ret = 0;

	/* Claim GPIOs for SPDC pins - used during power up/down */
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_0, "SPDC_D0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_1, "SPDC_D1");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_2, "SPDC_D2");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_3, "SPDC_D3");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_4, "SPDC_D4");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_5, "SPDC_D5");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_6, "SPDC_D6");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_7, "SPDC_D7");

	ret |= gpio_request(MX6SL_BRD_EPDC_GDOE, "SIPIX_YOE");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_9, "SIPIX_PWR_RDY");

	ret |= gpio_request(MX6SL_BRD_EPDC_GDSP, "SIPIX_YDIO");

	ret |= gpio_request(MX6SL_BRD_EPDC_GDCLK, "SIPIX_YCLK");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDSHR, "SIPIX_XDIO");

	ret |= gpio_request(MX6SL_BRD_EPDC_SDLE, "SIPIX_LD");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE1, "SIPIX_SOE");

	ret |= gpio_request(MX6SL_BRD_EPDC_SDCLK, "SIPIX_XCLK");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_10, "SIPIX_SHD_N");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE0, "SIPIX2_CE");

	return ret;
}

static void spdc_put_pins(void)
{
	gpio_free(MX6SL_BRD_EPDC_SDDO_0);
	gpio_free(MX6SL_BRD_EPDC_SDDO_1);
	gpio_free(MX6SL_BRD_EPDC_SDDO_2);
	gpio_free(MX6SL_BRD_EPDC_SDDO_3);
	gpio_free(MX6SL_BRD_EPDC_SDDO_4);
	gpio_free(MX6SL_BRD_EPDC_SDDO_5);
	gpio_free(MX6SL_BRD_EPDC_SDDO_6);
	gpio_free(MX6SL_BRD_EPDC_SDDO_7);

	gpio_free(MX6SL_BRD_EPDC_GDOE);
	gpio_free(MX6SL_BRD_EPDC_SDDO_9);
	gpio_free(MX6SL_BRD_EPDC_GDSP);
	gpio_free(MX6SL_BRD_EPDC_GDCLK);
	gpio_free(MX6SL_BRD_EPDC_SDSHR);
	gpio_free(MX6SL_BRD_EPDC_SDLE);
	gpio_free(MX6SL_BRD_EPDC_SDCE1);
	gpio_free(MX6SL_BRD_EPDC_SDCLK);
	gpio_free(MX6SL_BRD_EPDC_SDDO_10);
	gpio_free(MX6SL_BRD_EPDC_SDCE0);
}

static void spdc_enable_pins(void)
{
	/* Configure MUX settings to enable SPDC use */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_spdc_enable_pads, \
				ARRAY_SIZE(mx6sl_brd_spdc_enable_pads));

	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_1);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_2);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_3);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_4);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_5);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_6);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_7);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_8);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_9);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_10);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_11);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_12);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_13);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_14);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_15);
	gpio_direction_input(MX6SL_BRD_EPDC_GDOE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_9);
	gpio_direction_input(MX6SL_BRD_EPDC_GDSP);
	gpio_direction_input(MX6SL_BRD_EPDC_GDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_SDSHR);
	gpio_direction_input(MX6SL_BRD_EPDC_SDLE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE1);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_10);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE0);
}

static void spdc_disable_pins(void)
{
	/* Configure MUX settings for SPDC pins to
	 * GPIO and drive to 0. */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_spdc_disable_pads, \
				ARRAY_SIZE(mx6sl_brd_spdc_disable_pads));

	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_1, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_2, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_3, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_4, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_5, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_6, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_7, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_8 , 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_9 , 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_10, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_11, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_12, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_13, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_14, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_15, 0);

	gpio_direction_output(MX6SL_BRD_EPDC_GDOE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_9, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDSP, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDSHR, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDLE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE1, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_10, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE0, 0);
}

static struct imx_spdc_panel_init_set spdc_init_set = {
	.yoe_pol = false,
	.dual_gate = false,
	.resolution = 0,
	.ud = false,
	.rl = false,
	.data_filter_n = true,
	.power_ready = true,
	.rgbw_mode_enable = false,
	.hburst_len_en = true,
};

static struct fb_videomode erk_1_4_a01 = {
	.name = "ERK_1_4_A01",
	.refresh = 50,
	.xres = 800,
	.yres = 600,
	.pixclock = 40000000,
	.vmode = FB_VMODE_NONINTERLACED,
};

static struct imx_spdc_fb_mode spdc_panel_modes[] = {
	{
		&erk_1_4_a01,
		&spdc_init_set,
		.wave_timing = "pvi"
	},
};

static struct imx_spdc_fb_platform_data spdc_data = {
	.spdc_mode = spdc_panel_modes,
	.num_modes = ARRAY_SIZE(spdc_panel_modes),
	.get_pins = spdc_get_pins,
	.put_pins = spdc_put_pins,
	.enable_pins = spdc_enable_pins,
	.disable_pins = spdc_disable_pins,
};

static int __init early_use_spdc_sel(char *p)
{
	spdc_sel = 1;
	return 0;
}
early_param("spdc", early_use_spdc_sel);

static void setup_spdc(void)
{
	/* GPR0[8]: 0:EPDC, 1:SPDC */
	if (spdc_sel)
		mxc_iomux_set_gpr_register(0, 8, 1, 1);
}

static void imx6_ntx_usbotg_vbus(bool on)
{
#if 0
	if (on)
		gpio_set_value(MX6_BRD_USBOTG1_PWR, 1);
	else
		gpio_set_value(MX6_BRD_USBOTG1_PWR, 0);
#endif
}

static void __init mx6_ntx_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */
#if 0
	ret = gpio_request(MX6_BRD_USBOTG1_PWR, "usbotg-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_BRD_USBOTG1_PWR:%d\n", ret);
		return;
	}
	gpio_direction_output(MX6_BRD_USBOTG1_PWR, 0);

	ret = gpio_request(MX6_BRD_USBOTG2_PWR, "usbh1-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_BRD_USBOTG2_PWR:%d\n", ret);
		return;
	}
	gpio_direction_output(MX6_BRD_USBOTG2_PWR, 1);
#endif

	mx6_set_otghost_vbus_func(imx6_ntx_usbotg_vbus);
	mx6_usb_dr_init();
#ifdef CONFIG_USB_EHCI_ARC_HSIC
	mx6_usb_h2_init();
#endif
}

static struct platform_pwm_backlight_data mx6_ntx_pwm_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 128,
	.pwm_period_ns	= 50000,
};
static struct fb_videomode wvga_video_modes[] = {
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 32MHz */
	 "SEIKO-WVGA", 60, 800, 480, 29850, 99, 164, 33, 10, 10, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static struct mxc_fb_platform_data wvga_fb_data[] = {
	{
	 .interface_pix_fmt = V4L2_PIX_FMT_RGB24,
	 .mode_str = "SEIKO-WVGA",
	 .mode = wvga_video_modes,
	 .num_modes = ARRAY_SIZE(wvga_video_modes),
	 },
};

static struct platform_device lcd_wvga_device = {
	.name = "lcd_seiko",
};


#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
}

static struct gpio_keys_button gpio_key_matrix_FL[] = {
//	GPIO_BUTTON(GPIO_KB_ROW0, 90, 1, "front_light", 1),			// Front light
};

static struct gpio_keys_button gpio_key_HOME_FL[] = {
//	GPIO_BUTTON(GPIO_KB_COL1, 90, 1, "front_light", 1),			// Front light
	GPIO_BUTTON(GPIO_KB_COL0, 61, 1, "home", 1),			// home
};

static struct gpio_keys_platform_data ntx_gpio_key_data = {
	.buttons	= gpio_key_matrix_FL,
	.nbuttons	= ARRAY_SIZE(gpio_key_matrix_FL),
};

static struct platform_device ntx_gpio_key_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &ntx_gpio_key_data,
	}
};
#endif




static int mx6sl_ntx_keymap[] = {
	KEY(0, 0, 90),
	KEY(0, 1, KEY_POWER),
	KEY(0, 2, KEY_H),
	KEY(0, 3, KEY_F1),
};

static const struct matrix_keymap_data mx6sl_ntx_map_data __initconst = {
	.keymap		= mx6sl_ntx_keymap,
	.keymap_size	= ARRAY_SIZE(mx6sl_ntx_keymap),
};

static struct platform_device ntx_device_rtc = {
	.name           = "ntx_misc_rtc",
	.id				= 0,
	.dev            = {
		.platform_data = -1,
	}
};

void *g_wifi_sd_host;
irq_handler_t g_cd_irq;

void ntx_register_wifi_cd (irq_handler_t handler, void *data)
{
	printk ("[%s-%d] register g_cd_irq \n",__func__,__LINE__);
	g_wifi_sd_host = data;
	g_cd_irq = handler;
}

void ntx_wifi_power_ctrl (int isWifiEnable)
{
	int iHWID;

	printk("Wifi / BT power control %d\n", isWifiEnable);
	if(isWifiEnable == 0){
		gpio_direction_output (gMX6SL_WIFI_RST, 0);
		gpio_direction_input (gMX6SL_WIFI_3V3);	// turn off Wifi_3V3_on

		// sdio port disable ...
		if(33==gptHWCFG->m_val.bPCB) {
			//E60Q2X .
			mxc_iomux_v3_setup_multiple_pads(mx6sl_ntx_sd3_gpio_pads, ARRAY_SIZE(mx6sl_ntx_sd3_gpio_pads));
			gpio_request (MX6SL_SD3_CLK	, "MX6SL_SD3_CLK" );
			gpio_request (MX6SL_SD3_CMD	, "MX6SL_SD3_CMD" );
			gpio_request (MX6SL_SD3_DAT0, "MX6SL_SD3_DAT0");
			gpio_request (MX6SL_SD3_DAT1, "MX6SL_SD3_DAT1");
			gpio_request (MX6SL_SD3_DAT2, "MX6SL_SD3_DAT2");
			gpio_request (MX6SL_SD3_DAT3, "MX6SL_SD3_DAT3");
			gpio_direction_output (MX6SL_SD3_CLK , 0);
			gpio_direction_output (MX6SL_SD3_CMD , 0);
			gpio_direction_output (MX6SL_SD3_DAT0, 0);
			gpio_direction_output (MX6SL_SD3_DAT1, 0);
			gpio_direction_output (MX6SL_SD3_DAT2, 0);
			gpio_direction_output (MX6SL_SD3_DAT3, 0);
		}
		else {
			mxc_iomux_v3_setup_multiple_pads(mx6sl_ntx_sd2_gpio_pads, ARRAY_SIZE(mx6sl_ntx_sd2_gpio_pads));
			gpio_request (MX6SL_SD2_CLK	, "MX6SL_SD2_CLK" );
			gpio_request (MX6SL_SD2_CMD	, "MX6SL_SD2_CMD" );
			gpio_request (MX6SL_SD2_DAT0, "MX6SL_SD2_DAT0");
			gpio_request (MX6SL_SD2_DAT1, "MX6SL_SD2_DAT1");
			gpio_request (MX6SL_SD2_DAT2, "MX6SL_SD2_DAT2");
			gpio_request (MX6SL_SD2_DAT3, "MX6SL_SD2_DAT3");
			gpio_direction_input (MX6SL_SD2_CLK );
			gpio_direction_input (MX6SL_SD2_CMD );
			gpio_direction_input (MX6SL_SD2_DAT0);
			gpio_direction_input (MX6SL_SD2_DAT1);
			gpio_direction_input (MX6SL_SD2_DAT2);
			gpio_direction_input (MX6SL_SD2_DAT3);
		}

#ifdef _WIFI_ALWAYS_ON_
		disable_irq_wake(gpio_to_irq(gMX6SL_WIFI_INT));
#endif
	}
	else {
		gpio_direction_output (gMX6SL_WIFI_3V3, 0);	// turn on Wifi_3V3_on
		schedule_timeout(HZ/50);


		// sdio port process ...
		if(33==gptHWCFG->m_val.bPCB) {
			// E60Q2X .
			gpio_free (MX6SL_SD3_CLK );
			gpio_free (MX6SL_SD3_CMD );
			gpio_free (MX6SL_SD3_DAT0);
			gpio_free (MX6SL_SD3_DAT1);
			gpio_free (MX6SL_SD3_DAT2);
			gpio_free (MX6SL_SD3_DAT3);
			mxc_iomux_v3_setup_multiple_pads(mx6sl_ntx_sd3_wifi_pads, ARRAY_SIZE(mx6sl_ntx_sd3_wifi_pads));			
		}
		else {
			gpio_free (MX6SL_SD2_CLK );
			gpio_free (MX6SL_SD2_CMD );
			gpio_free (MX6SL_SD2_DAT0);
			gpio_free (MX6SL_SD2_DAT1);
			gpio_free (MX6SL_SD2_DAT2);
			gpio_free (MX6SL_SD2_DAT3);
			mxc_iomux_v3_setup_multiple_pads(mx6sl_ntx_sd2_wifi_pads, ARRAY_SIZE(mx6sl_ntx_sd2_wifi_pads));
		}

		gpio_direction_input (gMX6SL_WIFI_INT);
		gpio_direction_output (gMX6SL_WIFI_RST, 1);	// turn on wifi_RST
#ifdef _WIFI_ALWAYS_ON_
		enable_irq_wake(gpio_to_irq(gMX6SL_WIFI_INT));
#endif
		schedule_timeout(HZ/10);
	}

	if (g_cd_irq) {
		g_cd_irq (0, g_wifi_sd_host);
		schedule_timeout (100);
	}
	else {
		printk ("[%s-%d] not registered.\n",__func__,__LINE__);
	}
	printk("%s() end.\n",__FUNCTION__);
}
EXPORT_SYMBOL(ntx_wifi_power_ctrl);

static iomux_v3_cfg_t mx6sl_ntx_suspend_pads[] = {
	MX6SL_PAD_I2C2_SCL__GPIO_3_14,
	MX6SL_PAD_I2C2_SDA__GPIO_3_15,
};

static iomux_v3_cfg_t mx6sl_ntx_resume_pads[] = {
	MX6SL_PAD_I2C2_SCL__I2C2_SCL,
	MX6SL_PAD_I2C2_SDA__I2C2_SDA,
};

extern void ntx_gpio_suspend (void);
extern void ntx_gpio_resume (void);

static void ntx_suspend_enter(void)
{
//	mxc_iomux_v3_setup_multiple_pads(mx6sl_ntx_suspend_pads, ARRAY_SIZE(mx6sl_ntx_suspend_pads));
//	gpio_request(MX6SL_I2C2_SCL, "MX6SL_I2C2_SCL");
//	gpio_request(MX6SL_I2C2_SDA, "MX6SL_I2C2_SDA");
//	gpio_direction_output (MX6SL_I2C2_SCL, 0);
//	gpio_direction_output (MX6SL_I2C2_SDA, 0);

#if 0
	gpio_direction_output (MX6SL_EP_PWRALL, 1);
	gpio_direction_output (MX6SL_EP_WAKEUP, 0);
	gpio_direction_output (MX6SL_EP_PWRUP, 0);
	gpio_direction_output (MX6SL_EP_VCOM, 0);
	gpio_direction_input (MX6SL_EP_INT);
	gpio_direction_input (MX6SL_EP_PWRSTAT);
#endif
	ntx_gpio_suspend ();
	
#if 0
	{
		unsigned int *pIomux = IO_ADDRESS(MX6Q_IOMUXC_BASE_ADDR)+0x4C;
		unsigned int value;
		
		printk ("Addr , value\n");
		while (IO_ADDRESS(MX6Q_IOMUXC_BASE_ADDR)+0x5A8 >= pIomux) {
			value = *pIomux;
			printk ("0x%08X, 0x%08X, %s%s%s\n",pIomux, value,((value&0x2000)?"PUE - ":""),\
			((value&0x2000)?((value&0xC000)?"Pull Up - ":"Pull Down - "):""),((value&0x1000)?"PKE":""));
			++pIomux;
		}
	}
#endif
}

static void ntx_suspend_exit(void)
{
#if 0
	gpio_direction_output (MX6SL_EP_WAKEUP, 1);
#endif

//	gpio_free(MX6SL_I2C2_SCL);
//	gpio_free(MX6SL_I2C2_SDA);
//	mxc_iomux_v3_setup_multiple_pads(mx6sl_ntx_resume_pads, ARRAY_SIZE(mx6sl_ntx_resume_pads));
	ntx_gpio_resume ();
}

static const struct pm_platform_data mx6sl_ntx_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = ntx_suspend_enter,
	.suspend_exit = ntx_suspend_exit,
};

static void ntx_gpio_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_ntx_pads,
					ARRAY_SIZE(mx6sl_brd_ntx_pads));

	if(33==gptHWCFG->m_val.bPCB) {
		// E60Q2X ...
		mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_ntx_sd4_pads,
					ARRAY_SIZE(mx6sl_brd_ntx_sd4_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_ntx_sd1_gpio_pads,
					ARRAY_SIZE(mx6sl_brd_ntx_sd1_gpio_pads));

#if 1
		mxc_iomux_v3_setup_multiple_pads(mx6sl_ntx_q22_wifictrl_pads,
					ARRAY_SIZE(mx6sl_ntx_q22_wifictrl_pads));
#endif

		gMX6SL_NTX_ACIN_PG = IMX_GPIO_NR(5, 14);	
		gMX6SL_NTX_CHG = IMX_GPIO_NR(5, 15);	
		gMX6SL_MSP_INT = IMX_GPIO_NR(5, 11);	
		gMX6SL_PWR_SW = IMX_GPIO_NR(5, 8);	
		gMX6SL_IR_TOUCH_INT = IMX_GPIO_NR(5, 6);	
		gMX6SL_IR_TOUCH_RST = IMX_GPIO_NR(5, 9);	
		gMX6SL_HALL_EN = IMX_GPIO_NR(5, 12);	
		gMX6SL_CHG_LED = IMX_GPIO_NR(5, 10);	
		gMX6SL_ACT_LED = IMX_GPIO_NR(5, 7);	
		gMX6SL_ON_LED = IMX_GPIO_NR(5, 7);	
 		gMX6SL_WIFI_3V3 = IMX_GPIO_NR(4, 29);
 		gMX6SL_WIFI_RST = IMX_GPIO_NR(5, 0);
 		gMX6SL_WIFI_INT = IMX_GPIO_NR(4, 31);

		gpio_request (GPIO_ESD_3V3_ON, "ESD_3V3_ON");
		gpio_direction_output (GPIO_ESD_3V3_ON, 1);
		gpio_request (GPIO_ISD_3V3_ON, "ISD_3V3_ON");
		gpio_direction_output (GPIO_ISD_3V3_ON, 0);
		gpio_request (GPIO_IR_3V3_ON, "IR_3V3_ON");
		gpio_direction_output (GPIO_IR_3V3_ON, 1);
		gpio_request (GPIO_EP_3V3_ON, "EP_3V3_ON");
		gpio_direction_output (GPIO_EP_3V3_ON, 1);
	}
	else {
		mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_ntx_sd4_gpio_pads,
					ARRAY_SIZE(mx6sl_brd_ntx_sd4_gpio_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_ntx_sd1_pads,
					ARRAY_SIZE(mx6sl_brd_ntx_sd1_pads));
#if 1
		mxc_iomux_v3_setup_multiple_pads(mx6sl_ntx_q12_wifictrl_pads,
					ARRAY_SIZE(mx6sl_ntx_q12_wifictrl_pads));
#endif
	}


// 	i2c_zforce_ir_touch_binfo.platform_data = gMX6SL_IR_TOUCH_INT;
	i2c_zforce_ir_touch_binfo.irq = gpio_to_irq(gMX6SL_IR_TOUCH_INT);

	zforce_ts_data.gpio_int = gMX6SL_IR_TOUCH_INT;
	zforce_ts_data.gpio_rst = gMX6SL_IR_TOUCH_RST;

	i2c_sysmp_msp430_binfo.irq = gpio_to_irq(gMX6SL_MSP_INT);

	ntx_misc_info.acin_gpio     = gMX6SL_NTX_ACIN_PG;
	ntx_misc_info.chg_gpio      = gMX6SL_NTX_CHG;

	gpio_request (MX6SL_HW_ID0, "MX6SL_HW_ID0");
	gpio_request (MX6SL_HW_ID1, "MX6SL_HW_ID1");
	gpio_request (MX6SL_HW_ID2, "MX6SL_HW_ID2");
	gpio_request (MX6SL_HW_ID3, "MX6SL_HW_ID3");
	gpio_request (MX6SL_HW_ID4, "MX6SL_HW_ID4");
	gpio_direction_input (MX6SL_HW_ID0);
	gpio_direction_input (MX6SL_HW_ID1);
	gpio_direction_input (MX6SL_HW_ID2);
	gpio_direction_input (MX6SL_HW_ID3);
	gpio_direction_input (MX6SL_HW_ID4);
	
	gpio_request (gMX6SL_ON_LED, "MX6SL_ON_LED");
	gpio_request (gMX6SL_ACT_LED, "MX6SL_ACT_LED");
	gpio_request (gMX6SL_CHG_LED, "MX6SL_CHG_LED");
	gpio_direction_input (gMX6SL_ACT_LED);
	gpio_direction_output (gMX6SL_ON_LED, 0);
	gpio_direction_input (gMX6SL_CHG_LED);
	
	gpio_request (gMX6SL_NTX_ACIN_PG, "MX6SL_NTX_ACIN_PG");
	gpio_direction_input (gMX6SL_NTX_ACIN_PG);
	
	gpio_request (gMX6SL_NTX_CHG, "MX6SL_NTX_CHG");
	gpio_direction_input (gMX6SL_NTX_CHG);
	
	gpio_request (gMX6SL_MSP_INT, "MX6SL_MSP_INT");
	gpio_direction_input (gMX6SL_MSP_INT);
	
	gpio_request (gMX6SL_PWR_SW, "MX6SL_PWR_SW");
	gpio_direction_input (gMX6SL_PWR_SW);
	
//	gpio_request (gMX6SL_IR_TOUCH_INT, "MX6SL_IR_TOUCH_INT");
//	gpio_direction_input (gMX6SL_IR_TOUCH_INT);
	
//	gpio_request (gMX6SL_IR_TOUCH_RST, "MX6SL_IR_TOUCH_RST");
//	gpio_direction_input (gMX6SL_IR_TOUCH_RST);
	
	gpio_request (gMX6SL_HALL_EN, "MX6SL_HALL_EN");
	gpio_direction_input (gMX6SL_HALL_EN);
	
	gpio_request (gMX6SL_WIFI_RST, "MX6SL_WIFI_RST");
	gpio_request (gMX6SL_WIFI_3V3, "MX6SL_WIFI_3V3");
	gpio_request (gMX6SL_WIFI_INT, "MX6SL_WIFI_INT");
	gpio_direction_input (gMX6SL_WIFI_INT);
	ntx_wifi_power_ctrl (0);
	
	gpio_request (MX6SL_FL_EN, "MX6SL_FL_EN");
	gpio_direction_input (MX6SL_FL_EN);
	gpio_request (MX6SL_FL_R_EN, "MX6SL_FL_R_EN");
	gpio_direction_input (MX6SL_FL_R_EN);

#if 0	//[
	gpio_request (MX6SL_EP_PWRALL, "MX6SL_EP_PWRALL" );
	gpio_request (MX6SL_EP_WAKEUP	, "MX6SL_EP_WAKEUP" );
	gpio_request (MX6SL_EP_PWRUP	, "MX6SL_EP_PWRUP" );
	gpio_request (MX6SL_EP_INT	    , "MX6SL_EP_INT" );
	gpio_request (MX6SL_EP_PWRSTAT  , "MX6SL_EP_PWRSTAT" );
	gpio_request (MX6SL_EP_VCOM	    , "MX6SL_EP_VCOM" );
	gpio_direction_output (MX6SL_EP_PWRALL, 1);
	gpio_direction_output (MX6SL_EP_WAKEUP, 0);
	gpio_direction_output (MX6SL_EP_PWRUP, 0);
	gpio_direction_output (MX6SL_EP_VCOM, 0);
	gpio_direction_input (MX6SL_EP_INT);
	gpio_direction_input (MX6SL_EP_PWRSTAT);
#endif //]

}
/*!
 * Board specific initialization.
 */

static void __init mx6_ntx_init(void)
{
	u32 i;

	_parse_cmdline();

	printk ("[%s-%d] \n",__func__,__LINE__);
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_pads,
					ARRAY_SIZE(mx6sl_brd_pads));
					
	printk ("[%s-%d] \n",__func__,__LINE__);
	ntx_gpio_init ();

#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	gp_reg_id = mx6sl_ntx_dvfscore_data.reg_id;
	soc_reg_id = mx6sl_ntx_dvfscore_data.soc_id;
#else
	gp_reg_id = mx6sl_ntx_dvfscore_data.reg_id;
	soc_reg_id = mx6sl_ntx_dvfscore_data.soc_id;
	pu_reg_id = mx6sl_ntx_dvfscore_data.pu_id;
	mx6_cpu_regulator_init();
#endif

	if(1==gptHWCFG->m_val.bDisplayResolution) {
		// 1024x758 .
		zforce_ts_data.x_max = 758;
		zforce_ts_data.y_max = 1024;
	} else if(2==gptHWCFG->m_val.bDisplayResolution) {
		// 1024x768
		zforce_ts_data.x_max = 768;
		zforce_ts_data.y_max = 1024;
	} else if(3==gptHWCFG->m_val.bDisplayResolution) {
		// 1440x1080
		zforce_ts_data.x_max = 1080;
		zforce_ts_data.y_max = 1440;
	} else {
		// 800x600 
		zforce_ts_data.x_max = 600;
		zforce_ts_data.y_max = 800;
	}

	imx6q_add_imx_i2c(0, &mx6_ntx_i2c0_data);
	imx6q_add_imx_i2c(1, &mx6_ntx_i2c1_data);
	imx6q_add_imx_i2c(2, &mx6_ntx_i2c2_data);

	i2c_register_board_info(0,&i2c_zforce_ir_touch_binfo,1);
	i2c_register_board_info(2,&i2c_sysmp_msp430_binfo,1);

	//i2c_register_board_info(0, mxc_i2c0_board_info,
	//		ARRAY_SIZE(mxc_i2c0_board_info));
	//i2c_register_board_info(1, mxc_i2c1_board_info,
	//		ARRAY_SIZE(mxc_i2c1_board_info));
	//i2c_register_board_info(2, mxc_i2c2_board_info,
	//		ARRAY_SIZE(mxc_i2c2_board_info));

	platform_device_register(&ntx_device_rtc);
//	imx6q_add_imx_snvs_rtc();

	/* SPI */
	imx6q_add_ecspi(0, &mx6_ntx_spi_data);
	spi_device_init();

//	mx6sl_ntx_init_pfuze100(0);
	imx6q_add_anatop_thermal_imx(1, &mx6sl_anatop_thermal_data);

	imx6q_add_pm_imx(0, &mx6sl_ntx_pm_data);
	mx6_ntx_init_uart();
	/* get enet tx reference clk from FEC_REF_CLK pad.
	 * GPR1[14] = 0, GPR1[18:17] = 00
	 */
//	mxc_iomux_set_gpr_register(1, 14, 1, 0);
//	mxc_iomux_set_gpr_register(1, 17, 2, 0);

//	imx6_init_fec(fec_data);

	//platform_device_register(&ntx_vmmc_reg_devices);


	switch(gptHWCFG->m_val.bPCB) {
	case 33: //E60Q2X .
		// SD1 = GPIO
		// SD2 = ESD
		// SD3 = SDIO WIFI
		// SD4 = EMMC
		if(1==giBootPort) {

			// ESD is boot device .
			printk("add usdhc %d as mmcblk0\n",giBootPort+1);
			imx6q_add_sdhci_usdhc_imx(giBootPort, &mx6_ntx_esd_data);
			printk("add usdhc 4 as mmcblk1\n");
			imx6q_add_sdhci_usdhc_imx(3, &mx6_ntx_isd_data); // mmcblk1
			printk("add usdhc 3 as sdio for wifi\n");
			imx6q_add_sdhci_usdhc_imx(2, &mx6_ntx_q22_sd_wifi_data); 
		}
		else {
			// EMMC is boot device .
			imx6q_add_sdhci_usdhc_imx(giBootPort, &mx6_ntx_isd_data);
			imx6q_add_sdhci_usdhc_imx(1, &mx6_ntx_esd_data); // mmcblk1
			imx6q_add_sdhci_usdhc_imx(2, &mx6_ntx_q22_sd_wifi_data);
		}
		break;

	default:
		// SD1 = ISD
		// SD2 = ESD
		// SD3 = SDIO WIFI
		// SD4 = GPIO
		imx6q_add_sdhci_usdhc_imx(giBootPort, &mx6_ntx_isd_data);
		imx6q_add_sdhci_usdhc_imx(2, &mx6_ntx_esd_data); // mmcblk1
		imx6q_add_sdhci_usdhc_imx(1, &mx6_ntx_sd_wifi_data);
		break;
	}

	mx6_ntx_init_usb();
	imx6q_add_otp();
	imx6q_add_mxc_pwm(0);
//	imx6q_add_mxc_pwm_backlight(0, &mx6_ntx_pwm_backlight_data);


//		gpio_request(MX6_BRD_LCD_PWR_EN, "elcdif-power-on");
//		gpio_direction_output(MX6_BRD_LCD_PWR_EN, 1);
		//mxc_register_device(&lcd_wvga_device, NULL);

	imx6dl_add_imx_pxp();
	imx6dl_add_imx_pxp_client();
//	mxc_register_device(&max17135_sensor_device, NULL);
//	setup_spdc();
	imx6dl_add_imx_epdc(&epdc_data);
		imx6dl_add_imx_elcdif(&wvga_fb_data[0]);
	imx6q_add_dvfs_core(&mx6sl_ntx_dvfscore_data);

//	imx6q_init_audio();

	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);

	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	if(!NTXHWCFG_TST_FLAG(gptHWCFG->m_val.bPCB_Flags,0)) {   
		// key matrix : ON
		
		//mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_ntx_kb_pads,ARRAY_SIZE(mx6sl_brd_ntx_kb_pads));
		//mdelay(1);
		
		imx6sl_add_imx_keypad(&mx6sl_ntx_map_data);
	}
	else {
		// gpio keys 
		mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_ntx_kb_gpio_pads,
				ARRAY_SIZE(mx6sl_brd_ntx_kb_gpio_pads));
		udelay(1);

		switch(gptHWCFG->m_val.bPCB) {
		case 32://E60Q1X
		case 31://E60Q0X
			// use gpio instead of keymatrix ...
			gpio_request (GPIO_KB_COL0, "KB_COL0");
			gpio_direction_output (GPIO_KB_COL0, 0);
			gpio_request (GPIO_KB_COL1, "KB_COL1");
			gpio_direction_output (GPIO_KB_COL1, 0);

			ntx_gpio_key_data.buttons = gpio_key_matrix_FL;
			ntx_gpio_key_data.nbuttons = ARRAY_SIZE(gpio_key_matrix_FL);
			break;
		default:
			ntx_gpio_key_data.buttons = gpio_key_HOME_FL;
			ntx_gpio_key_data.nbuttons = ARRAY_SIZE(gpio_key_HOME_FL);
			break;
		}
		platform_device_register(&ntx_gpio_key_device);
	}
	
	imx6q_add_busfreq();
	imx6sl_add_dcp();
	imx6sl_add_rngb();
	imx6sl_add_imx_pxp_v4l2();

/*	mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
	clk_put(mxc_spdif_data.spdif_core_clk);
	imx6q_add_spdif(&mxc_spdif_data);
	imx6q_add_spdif_dai();
	imx6q_add_spdif_audio_device();
*/
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);

	/* Register charger chips */
	platform_device_register(&ntx_charger);
	platform_device_register(&ntx_light_ldm);


	mxc_register_device(&mxc_usb_plug_device, &usbplug_data);

	if(gptHWCFG) {

		if(1==gptHWCFG->m_val.bHallSensor) {
			// hall sensor enabled .
			tle4913_init();
		}


	}
	else {
		printk(KERN_ERR "missing ntx hwconfig !!\n");
	}

}

extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6sl_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init   = mx6_timer_init,
};

static void __init mx6_ntx_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, MEMBLOCK_ALLOC_ACCESSIBLE);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

MACHINE_START(MX6SL_NTX, "Freescale i.MX 6SoloLite NTX Board")
	.boot_params	= MX6SL_PHYS_OFFSET + 0x100,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= mx6_ntx_init,
	.timer		= &mxc_timer,
	.reserve	= mx6_ntx_reserve,
MACHINE_END
