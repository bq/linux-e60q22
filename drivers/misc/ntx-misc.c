/*
 * Copyright 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * Includes
 */
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include <linux/pmic_battery.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_status.h>
#include <linux/gpio.h>
#include <mach/iomux-mx6q.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/rtc.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/irq.h>

#include "../../arch/arm/mach-mx6/ntx_hwconfig.h"
#include "ntx-misc.h"

#include "../../arch/arm/mach-mx6/board-mx6sl_ntx.h"

#define GDEBUG 0
#include <linux/gallen_dbg.h>

extern volatile NTX_HWCONFIG *gptHWCFG;

struct ntx_misc_platform_data *ntx_misc;

#define NTX_IS_CHARGING		(gpio_get_value (ntx_misc->chg_gpio)?0:1)
#define NTX_ACIN_PG 		(gpio_get_value (ntx_misc->acin_gpio)?0:1)

struct i2c_client *g_up_i2c_client;
extern int g_wakeup_by_alarm;
static struct ntx_up_dev_info *gNtxUpDevInfo;

int up_read_reg(unsigned char reg)
{
	unsigned char buffer[10];
	struct i2c_msg msg[] = 
	{
		{.addr = g_up_i2c_client->addr, .flags = 0, .len = 1, .buf = &reg,}, 
		{.addr = g_up_i2c_client->addr, .flags = I2C_M_RD, .len = 2, .buf = buffer,},
	};
	if(0 > i2c_transfer(g_up_i2c_client->adapter, msg, 2))
		printk ("[%s-%d] i2c_transfer failed...\n", __func__, __LINE__);
	
	return ((buffer[0]<<8) | buffer[1]);
}

int up_write_reg(unsigned char reg, int value)
{
	unsigned char buffer[10];
	struct i2c_msg msg[] = 
	{
		{.addr = g_up_i2c_client->addr, .flags = 0, .len = 3, .buf = buffer,}, 
	};
	buffer[0] = reg;
	buffer[1] = value >> 8;
	buffer[2] = value & 0xFF;
	
	return i2c_transfer(g_up_i2c_client->adapter, msg, 1);
}


int up_get_time(struct rtc_time *tm)
{
    unsigned int tmp;
   	tmp = up_read_reg (0x20);
	tm->tm_year = ((tmp >> 8) & 0x0FF)+100;
	tm->tm_mon = (tmp & 0x0FF)-1;
    tmp = up_read_reg (0x21);
	tm->tm_mday = (tmp >> 8) & 0x0FF;
	tm->tm_hour = tmp & 0x0FF;
	tmp = up_read_reg (0x23);
	tm->tm_min = (tmp >> 8) & 0x0FF;
	tm->tm_sec = tmp & 0x0FF;
	
	return 0;
}
EXPORT_SYMBOL_GPL(up_get_time);

int up_set_time(struct rtc_time *tm)
{
	up_write_reg (0x10, ((tm->tm_year-100)<<8));
	up_write_reg (0x11, ((tm->tm_mon+1)<<8));
	up_write_reg (0x12, (tm->tm_mday<<8));
	up_write_reg (0x13, (tm->tm_hour<<8));
	up_write_reg (0x14, (tm->tm_min<<8));
	up_write_reg (0x15, (tm->tm_sec<<8));
	
	return 0;
}
EXPORT_SYMBOL_GPL(up_set_time);

static unsigned long gAlarmTime;
static unsigned long g_alarm_enabled;

int up_get_alarm(struct rtc_time *tm)
{
	rtc_time_to_tm(gAlarmTime,tm);
	return 0;
}
EXPORT_SYMBOL_GPL(up_get_alarm);

int up_set_alarm(struct rtc_time *tm)
{
	struct rtc_time now_tm;
	unsigned long now, time;

	if (tm) {
		up_get_time (&now_tm);
		rtc_tm_to_time(&now_tm, &now);
		rtc_tm_to_time(tm, &time);
		gAlarmTime=time;
	}

	if(tm && time > now) {
		int interval = time-now;
		printk ("[%s-%d] alarm %d\n",__func__,__LINE__,interval);
		up_write_reg (0x1B, (interval&0xFF00));
		up_write_reg (0x1C, ((interval<<8)& 0xFF00));
	}
	else {
		int tmp = up_read_reg (0x60);
		if (tmp & 0x8000) {
			printk ("[%s-%d] =================> Micro P MSP430 alarm triggered <===================\n", __func__, __LINE__);
			g_wakeup_by_alarm = 1;
		}
		up_write_reg (0x1B, 0);
		up_write_reg (0x1C, 0);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(up_set_alarm);

unsigned short msp430_deviceid(void)
{
	return up_read_reg(0);
}

void msp430_auto_power(int minutes)
{
}

void msp430_powerkeep(int n)
{
}

void msp430_power_off(void)
{
   	while (1) {
		printk("Kernel--- Power Down ---\n");
		up_write_reg (0x50, 0x0100);
      	msleep(1400);
	}
}

void msp430_pm_restart(void)
{
   	while (1) {
		printk("Kernel--- restart ---\n");
		up_write_reg (0x90, 0xff00);

      	msleep(1400);
	}
}

struct ntx_up_dev_info {
	struct device *dev;
	int battery_status;
	int charger_online;
	struct delayed_work work;
	struct power_supply bat;
	struct power_supply charger;
};

static enum power_supply_property ntx_up_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE, /* External power source */
};

static enum power_supply_property ntx_up_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS, /* Charger status output */
	POWER_SUPPLY_PROP_HEALTH, /* Fault or OK */
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

int gIsMSP430IntTriggered;
unsigned long gLastBatTick, gUSB_Change_Tick;
int gLastBatValue;

static int g_dc_charger_connect=0;
void set_pmic_dc_charger_state(int dccharger)
{
	g_dc_charger_connect=dccharger;
}
EXPORT_SYMBOL(set_pmic_dc_charger_state);

#define TICS_TO_CHK_ACIN_AFTER_BOOT		500
static unsigned long gdwTheTickToChkACINPlug;
//static struct timer_list acin_pg_timer;
static void acin_pg_chk(struct work_struct *work);
//static DEFINE_TIMER(acin_pg_timer,acin_pg_chk,0,0);
static int g_acin_pg_debounce;
typedef void (*usb_insert_handler) (char inserted);

static void acin_pg_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(work_acin_pg,acin_pg_work_func);

extern usb_insert_handler mxc_misc_report_usb;
extern void ntx_charger_online_event_callback(void);

extern int gIsCustomerUi;

static void ac_in_int_function(int irq)
{
	//del_timer_sync(&acin_pg_timer);
	//cancel_delayed_work(&work_acin_pg);

	if (gpio_get_value (MX6SL_NTX_ACIN_PG)) {
		irq_set_irq_type(irq, IRQF_TRIGGER_FALLING);
	}
	else {
		irq_set_irq_type(irq, IRQF_TRIGGER_RISING);
	}
	
	g_acin_pg_debounce = 0;
	if(time_after(jiffies,gdwTheTickToChkACINPlug)) {
		//mod_timer(&acin_pg_timer, jiffies + 1);
		schedule_delayed_work(&work_acin_pg,10);
	}
	else {
		//mod_timer(&acin_pg_timer, gdwTheTickToChkACINPlug);
		schedule_delayed_work(&work_acin_pg,TICS_TO_CHK_ACIN_AFTER_BOOT);
	}

	return ;
}

static void acin_pg_chk(struct work_struct *work)
{
	int i;

	//del_timer_sync(&acin_pg_timer);
	cancel_delayed_work(&work_acin_pg);

	++g_acin_pg_debounce;
	if (1 == g_acin_pg_debounce) 
	{
		if (gIsCustomerUi) {
			if(mxc_misc_report_usb) {
				mxc_misc_report_usb(gpio_get_value (MX6SL_NTX_ACIN_PG)?0:1);
				//ntx_charger_online_event_callback ();
			}
			else {
				printk(KERN_ERR"%s(%d): mxc_misc_report_usb=0,skip %s()\n",__FILE__,__LINE__,__FUNCTION__);
			}
		}
		cancel_delayed_work(&gNtxUpDevInfo->work);
		schedule_delayed_work(&gNtxUpDevInfo->work,20);
	}
	else {
		schedule_delayed_work(&work_acin_pg,10);
	}
}

static void acin_pg_work_func(struct work_struct *work)
{
	acin_pg_chk(work);
}


int msp430_battery(void)
{
	int i, battValue, temp, result;
	gIsMSP430IntTriggered = 0;
	if (gUSB_Change_Tick) {
		if (500 < (jiffies - gUSB_Change_Tick)) {
			gUSB_Change_Tick = 0;
			gLastBatValue = 0;
		}
	}
	
	if (gIsMSP430IntTriggered || !gLastBatValue || ((0 == gUSB_Change_Tick) && (200 < (jiffies - gLastBatTick)))) {
		battValue = up_read_reg (0x41);
		if (battValue) {
			gLastBatTick = jiffies;
			if (gpio_get_value (ntx_misc->acin_gpio)) {// not charging
				temp = up_read_reg (0x60);
				if (-1 != temp ) {
					if (0x8000 & temp) {
						printk ("[%s-%d] =================> Micro P MSP430 alarm triggered <===================\n", __func__, __LINE__);
						g_wakeup_by_alarm = 1;
					}
					if (0x01 & temp) {
						printk ("[%s-%d] =================> Micro P MSP430 Critical_Battery_Low <===================\n", __func__, __LINE__);
						return 0;
					}
					else if (!gLastBatValue) 
						gLastBatValue = battValue;
					else if (gLastBatValue > battValue)
						gLastBatValue = battValue;
					else
						battValue = gLastBatValue;
				}
			}
			else {
				if (gLastBatValue < battValue)
					gLastBatValue = battValue;
				else
					battValue = gLastBatValue;
			}
		}
		else {
			printk ("[%s-%d] MSP430 read failed\n", __func__, __LINE__);
			battValue = 0;
		}
	}
	else 
		battValue = gLastBatValue;
	
	return battValue;
}

static int ntx_up_battery_vol (void)
{
	int i, battValue, result;
	const unsigned short battGasgauge[] = {
	//	3.0V, 3.1V, 3.2V, 3.3V, 3.4V, 3.5V, 3.6V, 3.7V, 3.8V, 3.9V, 4.0V, 4.1V, 4.2V,
//		 743,  767,  791,  812,  835,  860,  885,  909,  935,  960,  985, 1010, 1023,
		 767,  791,  812,  833,  852,  877,  903,  928,  950,  979,  993, 1019, 1023,
	};
	
	if (NTX_ACIN_PG && !(NTX_IS_CHARGING))
		return 4200000;
	
	battValue = msp430_battery ();
	// transfer to uV to pmic interface.
	for (i=0; i< sizeof (battGasgauge); i++) {                 
		if (battValue <= battGasgauge[i]) {
			if (i && (battValue != battGasgauge[i])) {
				result = 3000000+ (i-1)*100000;
				result += ((battValue-battGasgauge[i-1]) * 100000 / (battGasgauge[i]-battGasgauge[i-1]));
			}
			else
				result = 3000000+ i*100000;
			break;
		}
	}
	return result;
}

static int ntx_up_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct ntx_up_dev_info *di =
		container_of((psy), struct ntx_up_dev_info, charger);
	int value;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = NTX_ACIN_PG;
		return 0;
	case POWER_SUPPLY_PROP_STATUS: /* Charger status output */
		if (NTX_ACIN_PG) {
			if (NTX_IS_CHARGING)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else 
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		di->battery_status = val->intval;
		return 0;
	case POWER_SUPPLY_PROP_HEALTH: /* Fault or OK */
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		value = ntx_up_battery_vol();
		if (4100000 <= value) {
//			printk("%s : full !! %d\n",__FUNCTION__,value);
			val->intval =  100;
		}
		else if (3400000 > value) {
			printk("%s : empty !! %d\n",__FUNCTION__,value);
			val->intval = 0;
		}
		else
			val->intval  = 100 - ((4100000 - value)/7000);
//		printk ("[%s-%d] battery %d( %d\% )\n",__func__,__LINE__,value,val->intval);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = ntx_up_battery_vol();
		return 0;
	default:
		break;
	}
	return -EINVAL;
}

static ssize_t chg_wa_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (NTX_IS_CHARGING)
		return sprintf(buf, "Charger LED workaround timer is on\n");
	else
		return sprintf(buf, "Charger LED workaround timer is off\n");
}

static ssize_t chg_wa_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (strstr(buf, "1") != NULL) {
		printk(KERN_INFO "Turned on the timer\n");
	} else if (strstr(buf, "0") != NULL) {
		printk(KERN_INFO "The Charger workaround timer is off\n");
	}

	return size;
}

static DEVICE_ATTR(enable, 0644, chg_wa_enable_show, chg_wa_enable_store);

static irqreturn_t ntx_misc_dcin(int irq, void *_data)
{
	struct ntx_up_dev_info *data = _data;

	DBG_MSG("%s():irq=%d\n",__FUNCTION__,irq);	

	gUSB_Change_Tick = jiffies;	// do not check battery value in 6 seconds

	ac_in_int_function(irq);
	return IRQ_HANDLED;
}

static irqreturn_t ntx_misc_chg(int irq, void *_data)
{
	return IRQ_HANDLED;
}

static int pmic_battery_remove(struct platform_device *pdev)
{
	struct ntx_up_dev_info *di = platform_get_drvdata(pdev);

	power_supply_unregister(&di->charger);

	kfree(di);

	return 0;
}

static void pmic_battery_work(struct work_struct *work)
{
	struct ntx_up_dev_info *data;
	data = container_of(work, struct ntx_up_dev_info, work.work);
	
	if (data->charger_online != NTX_ACIN_PG) {
		data->charger_online = NTX_ACIN_PG;
		power_supply_changed(&data->charger);
	}
	power_supply_changed(&data->bat);
	/* reschedule for the next time */
	schedule_delayed_work(&data->work, 5*HZ);
}

static int pmic_battery_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct ntx_up_dev_info *di;
	int irq;

	if (!pdev->dev.platform_data)
		return -EBUSY;
		
	ntx_misc = pdev->dev.platform_data;
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}
	gNtxUpDevInfo = di;
	platform_set_drvdata(pdev, di);

	INIT_DELAYED_WORK(&di->work, pmic_battery_work);

	di->charger.name = "mc13892_charger";	// "ntx_up_charger"
	di->charger.type = POWER_SUPPLY_TYPE_MAINS;
	di->charger.properties = ntx_up_charger_props;
	di->charger.num_properties = ARRAY_SIZE(ntx_up_charger_props);
	di->charger.get_property = ntx_up_charger_get_property;
	retval = power_supply_register(&pdev->dev, &di->charger);
	if (retval) {
		dev_err(di->dev, "failed to register charger\n");
		goto charger_failed;
	}
	
	di->bat.name = "mc13892_bat";	// "ntx_up_battery"
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = ntx_up_battery_props;
	di->bat.num_properties = ARRAY_SIZE(ntx_up_battery_props);
	di->bat.get_property = ntx_up_charger_get_property;
	di->bat.use_for_apm = 1;
	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		dev_err(di->dev, "failed to register charger\n");
		goto charger_failed;
	}
	
	irq=gpio_to_irq(ntx_misc->acin_gpio);
	retval = request_threaded_irq(irq,
			ntx_misc_dcin, NULL, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"NTX_MISC DC IN", di);
	if (retval) {
		dev_err(di->dev, "Cannot request irq %d for DC (%d)\n",
				irq, retval);
		goto charger_failed;
	}
	//irq_set_irq_type(irq, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING);
	enable_irq_wake(irq);


	irq=gpio_to_irq(ntx_misc->chg_gpio);
	retval = request_threaded_irq(irq,
			ntx_misc_chg, NULL, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"NTX_MISC CHG", di);
	if (retval) {
		dev_err(di->dev, "Cannot request irq %d for CHG (%d)\n",
				irq, retval);
		goto charger_failed;
	}
	enable_irq_wake(irq);


	schedule_delayed_work(&di->work, 5*HZ);

	goto success;

charger_failed:
	kfree(di);
di_alloc_failed:
success:
	dev_dbg(di->dev, "%s battery probed!\n", __func__);
	return retval;
}

static irqreturn_t msp430_int(int irq, void *dev_id)
{
	gIsMSP430IntTriggered = 1;
	printk ("[%s-%d] \n",__func__,__LINE__);
	return IRQ_HANDLED;
}

static __devinit int msp430_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int err = 0;

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
  		printk("%s, functionality check failed\n", __func__);
    		return -1;
  	}
	
	g_up_i2c_client = client;
	printk ("[%s-%d] firmware version %X\n",__func__,__LINE__,msp430_deviceid());
	
	pm_power_off = msp430_power_off;
	arm_pm_restart = msp430_pm_restart;

	err = request_irq(client->irq, msp430_int, IRQF_TRIGGER_FALLING, "msp430_int", "msp430_int");
	if (err < 0) {
		printk(KERN_ERR "%s(%s): Can't allocate irq %d\n", __FILE__, __func__, client->irq);
	}
	
	if(0x03!=gptHWCFG->m_val.bUIConfig) {
		// UIConfig not RD mode .
		enable_irq_wake(client->irq);
	}
	
	return 0;
}

static __devexit int msp430_i2c_remove(struct i2c_client *client)
{
	return 0;
}


static const struct i2c_device_id msp430_id[] = {
	{"msp430", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, msp430_id);

static struct i2c_driver up_i2c_driver = {
	.driver = {
		   .name = "msp430",
		   .owner = THIS_MODULE,
		   },
	.probe = msp430_i2c_probe,
	.remove = __devexit_p(msp430_i2c_remove),
	.id_table = msp430_id,
};

static struct platform_driver pmic_battery_driver_ldm = {
	.driver = {
		   .name = "pmic_battery",
		   .bus = &platform_bus_type,
		   .owner	= THIS_MODULE,
		   },
	.probe = pmic_battery_probe,
	.remove = pmic_battery_remove,
};

static int __init pmic_battery_init(void)
{
	pr_debug("PMIC Battery driver loading...\n");
	gdwTheTickToChkACINPlug = (unsigned long)(jiffies+TICS_TO_CHK_ACIN_AFTER_BOOT);
	i2c_add_driver(&up_i2c_driver);
	return platform_driver_register(&pmic_battery_driver_ldm);
}

static void __exit pmic_battery_exit(void)
{
	platform_driver_unregister(&pmic_battery_driver_ldm);
	i2c_del_driver(&up_i2c_driver);
	pr_debug("PMIC Battery driver successfully unloaded\n");
}

module_init(pmic_battery_init);
module_exit(pmic_battery_exit);

MODULE_DESCRIPTION("pmic_battery driver");
MODULE_AUTHOR("Netronix Inc.");
MODULE_LICENSE("GPL");
