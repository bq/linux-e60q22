

/*
 *
 * Purpose : FP9928 driver
 * Author : Gallen Lin
 * versions :
 *
 */ 

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/interrupt.h>


#include <mach/hardware.h>
#include <mach/gpio.h>

#include <mach/iomux-mx6sl.h>


#include "ntx_hwconfig.h"
#include "fake_s1d13522.h"

#define GDEBUG	1
#include <linux/gallen_dbg.h>

#include "lk_fp9928.h"


#define DRIVER_NAME "FP9928"


#define GPIO_FP9928_VIN_PADCFG		MX6SL_PAD_EPDC_PWRWAKEUP__GPIO_2_14
#define GPIO_FP9928_VIN					IMX_GPIO_NR(2,14)

#if 1
//
#define GPIO_FP9928_EN_PADCFG			MX6SL_PAD_EPDC_PWRCTRL1__GPIO_2_8	
#define GPIO_FP9928_EN					IMX_GPIO_NR(2,8)
#define GPIO_FP9928_VCOM_PADCFG			MX6SL_PAD_EPDC_VCOM0__GPIO_2_3	
#define GPIO_FP9928_VCOM				IMX_GPIO_NR(2,3)
#else
// beta version .
#define GPIO_FP9928_EN_PADCFG			MX6SL_PAD_EPDC_VCOM0__GPIO_2_3	
#define GPIO_FP9928_EN					IMX_GPIO_NR(2,3)
#define GPIO_FP9928_VCOM_PADCFG			MX6SL_PAD_EPDC_PWRCTRL1__GPIO_2_8	
#define GPIO_FP9928_VCOM				IMX_GPIO_NR(2,8)
#endif

#define VIN_ON		1
#define VIN_OFF		0
#define EN_ON			1
#define EN_OFF		0
#define VCOM_ON		1
#define VCOM_OFF	0


#define FP9928_POWEROFF_TICKS_MAX			200
#define FP9928_PWROFFDELAYWORK_TICKS	50


#if 0
#define FP9928_VCOM_MV_MAX		(-302)
#define FP9928_VCOM_MV_MIN		(-2501)
#define FP9928_VCOM_MV_STEP		(11)
#else
#define FP9928_VCOM_MV_MAX		(-302)
#define FP9928_VCOM_MV_MIN		(-6000)
#define FP9928_VCOM_MV_STEP		(22)
#endif



typedef struct tagFP9928_PWRDWN_WORK_PARAM {
	struct delayed_work pwrdwn_work;
	int iIsTurnOffChipPwr;
} FP9928_PWRDWN_WORK_PARAM;



typedef struct tagFP9228_data {
	int iCurrent_temprature;
	unsigned short wTempratureData,wReserved;
	struct i2c_adapter *ptI2C_adapter; 
	struct i2c_client *ptI2C_client;
	struct mutex tI2CLock;
	struct mutex tPwrLock;
	int iIsPoweredON;
	int iIsOutputEnabled;
	int iIsOutputPowerDownCounting;
	int iCurrent_VCOM_mV;
	unsigned long dwTickPowerOffEnd;
	FP9928_PWRDWN_WORK_PARAM tPwrdwn_work_param;
} FP9928_data;

static FP9928_data *gptFP9928_data ;

// externals ...
extern volatile NTX_HWCONFIG *gptHWCFG;
extern volatile int gSleep_Mode_Suspend;


static struct i2c_board_info gtFP9928_BI = {
 .type = "FP9928",
 .addr = 0x48,
 .platform_data = NULL,
};

static const unsigned short gwFP9928_AddrA[] = {
	0x48,
	I2C_CLIENT_END
};


static volatile unsigned char gbFP9928_REG_TMST_addr=0x00;
static volatile unsigned char gbFP9928_REG_TMST=0;

#define FP9928_REG_FUNC_ADJUST_VCOM_ADJ			0x01
#define FP9928_REG_FUNC_ADJUST_FIX_RD_PTR		0x02
static volatile unsigned char gbFP9928_REG_FUNC_ADJUST_addr=0x01;
static volatile unsigned char gbFP9928_REG_FUNC_ADJUST=0x01;

#define FP9928_REG_VCOM_SETTING_ALL					0xff
static volatile unsigned char gbFP9928_REG_VCOM_SETTING_addr=0x02;
static volatile unsigned char gbFP9928_REG_VCOM_SETTING=0x74;


static int _fp9928_set_reg(unsigned char bRegAddr,unsigned char bRegSetVal)
{
	int iRet=FP9928_RET_SUCCESS;
	int iChk;
	unsigned char bA[2] ;

	ASSERT(gptFP9928_data);

	if(!in_interrupt()) {
		mutex_lock(&gptFP9928_data->tI2CLock);
	}

	bA[0]=bRegAddr;
	bA[1]=bRegSetVal;
	iChk = i2c_master_send(gptFP9928_data->ptI2C_client, (const char *)bA, sizeof(bA));
	if (iChk < 0) {
		ERR_MSG("%s(%d):%d=%s(),regAddr=0x%x,regVal=0x%x fail !\n",__FILE__,__LINE__,\
			iChk,"i2c_master_send",bRegAddr,bRegSetVal);
		iRet=FP9928_RET_I2CTRANS_ERR;
	}
	
	if(!in_interrupt()) {
		mutex_unlock(&gptFP9928_data->tI2CLock);
	}

	return iRet;
}

static int _fp9928_get_reg(unsigned char bRegAddr,unsigned char *O_pbRegVal)
{
	int iRet=FP9928_RET_SUCCESS;
	int iChk;
	unsigned char bA[1] ;

	ASSERT(gptFP9928_data);

	if(!in_interrupt()) {
		mutex_lock(&gptFP9928_data->tI2CLock);
	}


	bA[0]=bRegAddr;
	iChk = i2c_master_send(gptFP9928_data->ptI2C_client, (const char *)bA, 1);
	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_send fail !\n",__FILE__,__LINE__,__FUNCTION__);
		iRet = FP9928_RET_I2CTRANS_ERR;
	}
	

	iChk = i2c_master_recv(gptFP9928_data->ptI2C_client, bA, 1);
	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_recv fail !\n",__FILE__,__LINE__,__FUNCTION__);
		iRet = FP9928_RET_I2CTRANS_ERR;
	}


	if(iRet>=0) {
		*O_pbRegVal = bA[0];
	}

	
	//DBG_MSG("%s(0x%x,%p)==>0x%x\n",__FUNCTION__,bRegAddr,O_pbRegVal,bA[0]);

	if(!in_interrupt()) {
		mutex_unlock(&gptFP9928_data->tI2CLock);
	}

	return iRet;
}


#define FP9928_REG_SET(_regName,_bFieldName,_bSetVal)		\
({\
	int _iRet=FP9928_RET_SUCCESS;\
	int _iChk;\
	unsigned char _bNewReg,_bFieldMask;\
	\
	_bFieldMask=(unsigned char)FP9928_REG_##_regName##_##_bFieldName;\
	if(0xff==_bFieldMask) {\
		_bNewReg = _bSetVal;\
	}\
	else {\
		_bNewReg=gbFP9928_REG_##_regName;\
		if(_bSetVal) {\
			_bNewReg |= _bFieldMask ;\
		}\
		else {\
			_bNewReg &= ~_bFieldMask;\
		}\
	}\
	\
	_iChk = _fp9928_set_reg(gbFP9928_REG_##_regName##_##addr,_bNewReg);\
	if(_iChk<0) {\
		_iRet = _iChk;\
	}\
	else {\
		DBG_MSG("%s() : FP9928 write reg%s(%02Xh) 0x%02x->0x%02x\n",__FUNCTION__,\
		#_regName,gbFP9928_REG_##_regName##_##addr,gbFP9928_REG_##_regName,_bNewReg);\
		gbFP9928_REG_##_regName = _bNewReg;\
	}\
	_iRet;\
})

#define FP9928_REG_GET(_regName)		\
({\
	int _iChk;\
	unsigned char bReadReg=0;\
	unsigned short _wRet=0;\
	\
	_iChk = _fp9928_get_reg(gbFP9928_REG_##_regName##_##addr,&bReadReg);\
	if(_iChk<0) {\
		_wRet = (unsigned short)(-1);\
	}\
	else {\
		_wRet = bReadReg;\
		gbFP9928_REG_##_regName = bReadReg;\
		DBG_MSG("%s() : FP9928 read reg%s(%02Xh)=0x%02x\n",__FUNCTION__,\
			#_regName,gbFP9928_REG_##_regName##_##addr,bReadReg);\
	}\
	_wRet;\
})

#define FP9928_REG(_regName)	gbFP9928_REG_##_regName






static int _fp9928_gpio_init(void)
{
	int iRet = FP9928_RET_SUCCESS;

	GALLEN_DBGLOCAL_BEGIN();

	mxc_iomux_v3_setup_pad(GPIO_FP9928_VIN_PADCFG);
	if(0!=gpio_request(GPIO_FP9928_VIN, "fp9928_VIN")) {
		WARNING_MSG("%s(),request gpio fp9928_VIN fail !!\n",__FUNCTION__);
		gpio_direction_input(GPIO_FP9928_VIN);
	}

	mxc_iomux_v3_setup_pad(GPIO_FP9928_EN_PADCFG);
	if(0!=gpio_request(GPIO_FP9928_EN, "fp9928_EN")) {
		WARNING_MSG("%s(),request gpio fp9928_EN fail !!\n",__FUNCTION__);
		gpio_direction_input(GPIO_FP9928_EN);
	}

	mxc_iomux_v3_setup_pad(GPIO_FP9928_VCOM_PADCFG);
	gpio_request(GPIO_FP9928_VCOM, "fp9928_VCOM");
	if(0!=gpio_request(GPIO_FP9928_VCOM, "fp9928_VCOM")) {
		WARNING_MSG("%s(),request gpio fp9928_VCOM fail !!\n",__FUNCTION__);
		gpio_direction_input(GPIO_FP9928_VCOM);
	}

	GALLEN_DBGLOCAL_END();

	return iRet;
}

static void _fp9928_gpio_release(void)
{
	GALLEN_DBGLOCAL_BEGIN();
	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return ;
	}
	gpio_free(GPIO_FP9928_VCOM);
	gpio_free(GPIO_FP9928_EN);
	gpio_free(GPIO_FP9928_VIN);

	GALLEN_DBGLOCAL_END();
}


int _fp9928_reg_init(void)
{
	//FP9928_REG_SET(FUNC_ADJUST,FIX_RD_PTR,1);
	//FP9928_REG_GET(TMST);
	//FP9928_REG_GET(TMST);
}

int _fp9928_output_en(int iIsEnable)
{
	int iRet = FP9928_RET_SUCCESS;

	DBG_MSG("%s(%d)\n",__FUNCTION__,iIsEnable);

	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		return FP9928_RET_NOTINITEDSTATE;
	}

	if(gptFP9928_data->iIsOutputEnabled == iIsEnable) {
		// nothing have to do when state not change .
	}
	else {
		unsigned long dwTickNow = jiffies,dwTicks ;
		if(iIsEnable) {
			if ( time_before(dwTickNow,gptFP9928_data->dwTickPowerOffEnd) ) {
				WARNING_MSG("[WARNING] %s(%d) : Trunning ON the EPD power maybe in powering off state !!!\n",__FILE__,__LINE__);
				dwTicks = gptFP9928_data->dwTickPowerOffEnd-dwTickNow;

				DBG_MSG("%s() going to sleep %ld ticks \n",__FUNCTION__,dwTicks);
				msleep(jiffies_to_msecs(dwTicks));
			}

			gpio_direction_output(GPIO_FP9928_EN,EN_ON);
			gptFP9928_data->iIsOutputEnabled = 1;
			msleep(23);
			gpio_direction_output(GPIO_FP9928_VCOM,VCOM_ON);
			msleep(10);
		}
		else {

			gpio_direction_output(GPIO_FP9928_EN,EN_OFF);
			gptFP9928_data->iIsOutputEnabled = 0;
			gptFP9928_data->dwTickPowerOffEnd = jiffies + FP9928_POWEROFF_TICKS_MAX;
		}
	}

	return iRet ;
}



static int _fp9928_vin_onoff(int iIsON)
{
	int iRet = FP9928_RET_SUCCESS;

	ASSERT(gptFP9928_data);

	DBG_MSG("%s(%d)\n",__FUNCTION__,iIsON);
	
	if(iIsON==gptFP9928_data->iIsPoweredON) {
	}
	else {

		if(iIsON) {
			gpio_direction_output(GPIO_FP9928_VIN,VIN_ON);
			gptFP9928_data->iIsPoweredON = 1;
		}
		else {
			_fp9928_output_en(0);

			gpio_direction_output(GPIO_FP9928_VIN,VIN_OFF);
			gptFP9928_data->iIsPoweredON = 0;
		}
	}

	return iRet;
}

static void _fp9928_pwrdwn_work_func(struct work_struct *work)
{
	GALLEN_DBGLOCAL_BEGIN();

	mutex_lock(&gptFP9928_data->tPwrLock);

	if(!gptFP9928_data->iIsOutputPowerDownCounting) {
		WARNING_MSG("[WARNING]%s(%d): race condition occured !\n",__FILE__,__LINE__);
		return ;
	}

	_fp9928_output_en(0);
	gptFP9928_data->iIsOutputPowerDownCounting = 0;

	if(gptFP9928_data->tPwrdwn_work_param.iIsTurnOffChipPwr) {
		//msleep(FP9928_POWEROFF_TICKS_MAX/100);
		_fp9928_vin_onoff(0);
	}

	mutex_unlock(&gptFP9928_data->tPwrLock);

	GALLEN_DBGLOCAL_END();
}


/**********************************************************************
 *
 * public functions .
 *
***********************************************************************/

int fp9928_power_onoff(int iIsPowerOn,int iIsOutputPwr)
{
	int iRet = FP9928_RET_SUCCESS;

	GALLEN_DBGLOCAL_BEGIN();
	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return FP9928_RET_NOTINITEDSTATE;
	}

		

	mutex_lock(&gptFP9928_data->tPwrLock);
	if (iIsPowerOn) {
		_fp9928_vin_onoff(1);

		if(iIsOutputPwr==1) {
			gptFP9928_data->iIsOutputPowerDownCounting = 0;
			cancel_delayed_work_sync(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work);
			_fp9928_output_en(1);
		}
	}
	else {
		gptFP9928_data->tPwrdwn_work_param.iIsTurnOffChipPwr = 1;
		if(!gptFP9928_data->iIsOutputPowerDownCounting) 
		{
			gptFP9928_data->iIsOutputPowerDownCounting = 1;
			cancel_delayed_work_sync(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work);
			schedule_delayed_work(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work, \
					FP9928_PWROFFDELAYWORK_TICKS);
		}
	}
	mutex_unlock(&gptFP9928_data->tPwrLock);	

	GALLEN_DBGLOCAL_BEGIN();
	return iRet;
}

int fp9928_output_power(int iIsOutputPwr,int iIsChipPowerDown)
{
	int iRet=FP9928_RET_SUCCESS;

	GALLEN_DBGLOCAL_BEGIN();

	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return FP9928_RET_NOTINITEDSTATE;
	}


	mutex_lock(&gptFP9928_data->tPwrLock);
	if(iIsOutputPwr) {
		GALLEN_DBGLOCAL_RUNLOG(0);
		gptFP9928_data->iIsOutputPowerDownCounting = 0;
		cancel_delayed_work_sync(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work);

		if(!gptFP9928_data->iIsPoweredON) {
			GALLEN_DBGLOCAL_RUNLOG(1);;
			// auto power on chip .
			_fp9928_vin_onoff(1);
		}

		iRet = _fp9928_output_en(1);
	}
	else {
		GALLEN_DBGLOCAL_RUNLOG(2);
		if(!gptFP9928_data->iIsOutputPowerDownCounting) {
			GALLEN_DBGLOCAL_RUNLOG(3);

			udelay(100);gpio_direction_output(GPIO_FP9928_VCOM,VCOM_OFF);

			gptFP9928_data->tPwrdwn_work_param.iIsTurnOffChipPwr = iIsChipPowerDown;
			gptFP9928_data->iIsOutputPowerDownCounting = 1;
			cancel_delayed_work_sync(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work);
			schedule_delayed_work(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work, \
					FP9928_PWROFFDELAYWORK_TICKS);
		}
		else {
			GALLEN_DBGLOCAL_RUNLOG(4);
			DBG_MSG("%s(%d),power down work already exist \n",__FUNCTION__,__LINE__);
		}
	}
	mutex_unlock(&gptFP9928_data->tPwrLock);

	GALLEN_DBGLOCAL_END();
	return iRet;
}

int fp9928_suspend(void)
{
	int iRet = FP9928_RET_SUCCESS;
	int iChk;

	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		return FP9928_RET_NOTINITEDSTATE;
	}

	if(gptFP9928_data->iIsOutputEnabled) {
		WARNING_MSG("%s() : skip suspend when PMIC output enabled !!\n",__FUNCTION__);
		return FP9928_RET_PWRDWNWORKPENDING;
	}

	if(gSleep_Mode_Suspend) {
		mutex_lock(&gptFP9928_data->tPwrLock);	
		_fp9928_vin_onoff(0);
		mutex_unlock(&gptFP9928_data->tPwrLock);
	}

	return iRet;
}

void fp9928_resume(void)
{
	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		return ;
	}

	if(gSleep_Mode_Suspend) {
		mutex_lock(&gptFP9928_data->tPwrLock);	
		_fp9928_vin_onoff(1);
		mutex_unlock(&gptFP9928_data->tPwrLock);
	}
}



int fp9928_get_temperature(int *O_piTemperature)
{
	unsigned short wReg;
	int iRet=FP9928_RET_SUCCESS;

	int iTemp;
	unsigned char bReg,bTemp;

	//return FP9928_RET_SUCCESS;

	if(!gptFP9928_data) {
		WARNING_MSG("%s() cannot work if driver is not initialed \n",__FUNCTION__);
		return FP9928_RET_NOTINITEDSTATE;
	}

	//fp9928_output_power(1,0);
	
	wReg = FP9928_REG_GET(TMST);

	//fp9928_output_power(0,0);

	if(((unsigned short)(-1))==wReg) {
		ERR_MSG("%s(%d):%s regTMST read fail !\n",__FILE__,__LINE__,__FUNCTION__);
		return FP9928_RET_I2CTRANS_ERR;
	}

	bReg = (unsigned char)wReg;
	gptFP9928_data->wTempratureData = wReg;
	if(bReg&0x80) {
		// negative .
		bTemp=(~bReg)+1;
		iTemp = bTemp;
		iTemp = (~iTemp)+1;
	}
	else {
		// positive .
		iTemp = (int)(bReg);
	}
	gptFP9928_data->iCurrent_temprature = iTemp;
	printk("%s temprature data = 0x%x,%d\n",DRIVER_NAME,wReg,gptFP9928_data->iCurrent_temprature);

	if(O_piTemperature) {
		*O_piTemperature = gptFP9928_data->iCurrent_temprature;
	}

	return iRet;
}





int fp9928_vcom_set(int iVCOM_mV,int iIsWriteToFlash)
{

	const int iVCOM_mV_max=FP9928_VCOM_MV_MAX,iVCOM_mV_min=FP9928_VCOM_MV_MIN;
	int iVCOM_mV_ABS ;
	int iRet=FP9928_RET_SUCCESS;

	if(!gptFP9928_data) {
		WARNING_MSG("%s() cannot work if driver is not initialed \n",__FUNCTION__);
		return FP9928_RET_NOTINITEDSTATE;
	}
	
	if(iVCOM_mV<iVCOM_mV_min) {
		ERR_MSG("%s(%d),VCOM %d cannot < %d mV\n",
				__FUNCTION__,__LINE__,iVCOM_mV,iVCOM_mV_min);
	}
	else if(iVCOM_mV>iVCOM_mV_max) {
		ERR_MSG("%s(%d),VCOM %d cannot > %d\n",
				__FUNCTION__,__LINE__,iVCOM_mV,iVCOM_mV_max);
	}
	else {
		unsigned char bReg;
		if(iVCOM_mV<0) {
			iVCOM_mV_ABS = -iVCOM_mV;
		}
		else {
			iVCOM_mV_ABS = iVCOM_mV;
		}

		bReg = (unsigned char) (iVCOM_mV_ABS/FP9928_VCOM_MV_STEP);
		iRet = FP9928_REG_SET(VCOM_SETTING,ALL,bReg);
		if(iRet>=0) {
			gptFP9928_data->iCurrent_VCOM_mV=-((int)FP9928_REG(VCOM_SETTING)*FP9928_VCOM_MV_STEP);
		}
	}

	return iRet;
}


int fp9928_vcom_get(int *O_piVCOM_mV)
{
	int iVCOM_mV;
	unsigned short wReg;

	if(!gptFP9928_data) {
		WARNING_MSG("%s() cannot work if driver is not initialed \n",__FUNCTION__);
		return FP9928_RET_NOTINITEDSTATE;
	}

	wReg = FP9928_REG_GET(VCOM_SETTING);
	iVCOM_mV = (int)(wReg);
	iVCOM_mV = -(iVCOM_mV*FP9928_VCOM_MV_STEP);
	//DBG_MSG("%s(%d):iVCOM_mV=%d,wReg=0x%x\n",__FUNCTION__,__LINE__,iVCOM_mV,wReg);

	if(iVCOM_mV!=gptFP9928_data->iCurrent_VCOM_mV) {
		WARNING_MSG("%s(%d) VCOM read from register is not equal with stored \n",__FUNCTION__,__LINE__);
	}

	if(O_piVCOM_mV) {
		*O_piVCOM_mV = iVCOM_mV;
	}


	return iVCOM_mV;
}


void fp9928_release(void)
{
	if(!gptFP9928_data) {
		WARNING_MSG("%s() cannot work if driver is not initialed \n",__FUNCTION__);
		return ;
	}


	mutex_lock(&gptFP9928_data->tPwrLock);	
	_fp9928_vin_onoff(0);
	mutex_unlock(&gptFP9928_data->tPwrLock);	

	gptFP9928_data->ptI2C_adapter = 0;
	i2c_unregister_device(gptFP9928_data->ptI2C_client);
	gptFP9928_data->ptI2C_client = 0;
	
	_fp9928_gpio_release();

	kfree(gptFP9928_data);gptFP9928_data = 0;

}

int fp9928_init(int iPort)
{

	int iRet = FP9928_RET_SUCCESS;
	int iChk;

	unsigned long dwSize=sizeof(FP9928_data);

	GALLEN_DBGLOCAL_BEGIN();

	gptFP9928_data = kmalloc(dwSize,GFP_KERNEL);
	if(!gptFP9928_data) {
		iRet = FP9928_RET_MEMNOTENOUGH;
		ERR_MSG("%s(%d) : memory not enough !!\n",__FILE__,__LINE__);
		GALLEN_DBGLOCAL_RUNLOG(0);
		goto MEM_MALLOC_FAIL;
	}

	memset(gptFP9928_data,0,sizeof(FP9928_data));
	
	iChk = _fp9928_gpio_init();
	if(iChk<0) {
		iRet = FP9928_RET_GPIOINITFAIL;
		ERR_MSG("%s(%d) : gpio init fail !!\n",__FILE__,__LINE__);
		GALLEN_DBGLOCAL_RUNLOG(1);
		goto GPIO_INIT_FAIL;
	}

	gptFP9928_data->ptI2C_adapter = i2c_get_adapter(iPort-1);//
	if( NULL == gptFP9928_data->ptI2C_adapter) {
		ERR_MSG ("[Error] %s : FP9928_RET_I2CCHN_NOTFOUND,chn=%d\n",__FUNCTION__,iPort);
		GALLEN_DBGLOCAL_RUNLOG(2);
		iRet=FP9928_RET_I2CCHN_NOTFOUND;
		goto I2CCHN_GET_FAIL;
	}

	gptFP9928_data->ptI2C_client = i2c_new_probed_device(gptFP9928_data->ptI2C_adapter, &gtFP9928_BI,gwFP9928_AddrA,0);
	if( NULL == gptFP9928_data->ptI2C_client ) {
		GALLEN_DBGLOCAL_RUNLOG(3);
		ERR_MSG("[Error] %s : FP9928 probe fail \n",__FUNCTION__);
		goto I2CPROBE_DEVICE_FAIL;
	}

	gptFP9928_data->dwTickPowerOffEnd = jiffies;
	gptFP9928_data->iCurrent_VCOM_mV=-((int)FP9928_REG(VCOM_SETTING)*FP9928_VCOM_MV_STEP);// default VCOM voltage .

	// kernel objects initialize ...
	mutex_init(&gptFP9928_data->tPwrLock);
	mutex_init(&gptFP9928_data->tI2CLock);

	INIT_DELAYED_WORK(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work,_fp9928_pwrdwn_work_func);

	_fp9928_vin_onoff(1);

	_fp9928_reg_init();

	GALLEN_DBGLOCAL_ESC();
	return FP9928_RET_SUCCESS;


	i2c_unregister_device(gptFP9928_data->ptI2C_client);
	gptFP9928_data->ptI2C_client = 0;
I2CPROBE_DEVICE_FAIL:
	gptFP9928_data->ptI2C_adapter = 0;
I2CCHN_GET_FAIL:
	_fp9928_gpio_release();
GPIO_INIT_FAIL:
	kfree(gptFP9928_data);gptFP9928_data = 0;
MEM_MALLOC_FAIL:
	
	GALLEN_DBGLOCAL_END();
	return iRet;
}




/*
 *
 * Purpose : FP9928 driver
 * Author : Gallen Lin
 * versions :
 *
 */ 

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/interrupt.h>


#include <mach/hardware.h>
#include <mach/gpio.h>

#include <mach/iomux-mx6sl.h>


#include "ntx_hwconfig.h"
#include "fake_s1d13522.h"

#define GDEBUG	1
#include <linux/gallen_dbg.h>

#include "lk_fp9928.h"


#define DRIVER_NAME "FP9928"


#define GPIO_FP9928_VIN_PADCFG		MX6SL_PAD_EPDC_PWRWAKEUP__GPIO_2_14
#define GPIO_FP9928_VIN					IMX_GPIO_NR(2,14)

#if 1
//
#define GPIO_FP9928_EN_PADCFG			MX6SL_PAD_EPDC_PWRCTRL1__GPIO_2_8	
#define GPIO_FP9928_EN					IMX_GPIO_NR(2,8)
#define GPIO_FP9928_VCOM_PADCFG			MX6SL_PAD_EPDC_VCOM0__GPIO_2_3	
#define GPIO_FP9928_VCOM				IMX_GPIO_NR(2,3)
#else
// beta version .
#define GPIO_FP9928_EN_PADCFG			MX6SL_PAD_EPDC_VCOM0__GPIO_2_3	
#define GPIO_FP9928_EN					IMX_GPIO_NR(2,3)
#define GPIO_FP9928_VCOM_PADCFG			MX6SL_PAD_EPDC_PWRCTRL1__GPIO_2_8	
#define GPIO_FP9928_VCOM				IMX_GPIO_NR(2,8)
#endif

#define VIN_ON		1
#define VIN_OFF		0
#define EN_ON			1
#define EN_OFF		0
#define VCOM_ON		1
#define VCOM_OFF	0


#define FP9928_POWEROFF_TICKS_MAX			200
#define FP9928_PWROFFDELAYWORK_TICKS	50


#if 0
#define FP9928_VCOM_MV_MAX		(-302)
#define FP9928_VCOM_MV_MIN		(-2501)
#define FP9928_VCOM_MV_STEP		(11)
#else
#define FP9928_VCOM_MV_MAX		(-302)
#define FP9928_VCOM_MV_MIN		(-6000)
#define FP9928_VCOM_MV_STEP		(22)
#endif



typedef struct tagFP9928_PWRDWN_WORK_PARAM {
	struct delayed_work pwrdwn_work;
	int iIsTurnOffChipPwr;
} FP9928_PWRDWN_WORK_PARAM;



typedef struct tagFP9228_data {
	int iCurrent_temprature;
	unsigned short wTempratureData,wReserved;
	struct i2c_adapter *ptI2C_adapter; 
	struct i2c_client *ptI2C_client;
	struct mutex tI2CLock;
	struct mutex tPwrLock;
	int iIsPoweredON;
	int iIsOutputEnabled;
	int iIsOutputPowerDownCounting;
	int iCurrent_VCOM_mV;
	unsigned long dwTickPowerOffEnd;
	FP9928_PWRDWN_WORK_PARAM tPwrdwn_work_param;
} FP9928_data;

static FP9928_data *gptFP9928_data ;

// externals ...
extern volatile NTX_HWCONFIG *gptHWCFG;
extern volatile int gSleep_Mode_Suspend;


static struct i2c_board_info gtFP9928_BI = {
 .type = "FP9928",
 .addr = 0x48,
 .platform_data = NULL,
};

static const unsigned short gwFP9928_AddrA[] = {
	0x48,
	I2C_CLIENT_END
};


static volatile unsigned char gbFP9928_REG_TMST_addr=0x00;
static volatile unsigned char gbFP9928_REG_TMST=0;

#define FP9928_REG_FUNC_ADJUST_VCOM_ADJ			0x01
#define FP9928_REG_FUNC_ADJUST_FIX_RD_PTR		0x02
static volatile unsigned char gbFP9928_REG_FUNC_ADJUST_addr=0x01;
static volatile unsigned char gbFP9928_REG_FUNC_ADJUST=0x01;

#define FP9928_REG_VCOM_SETTING_ALL					0xff
static volatile unsigned char gbFP9928_REG_VCOM_SETTING_addr=0x02;
static volatile unsigned char gbFP9928_REG_VCOM_SETTING=0x74;


static int _fp9928_set_reg(unsigned char bRegAddr,unsigned char bRegSetVal)
{
	int iRet=FP9928_RET_SUCCESS;
	int iChk;
	unsigned char bA[2] ;

	ASSERT(gptFP9928_data);

	if(!in_interrupt()) {
		mutex_lock(&gptFP9928_data->tI2CLock);
	}

	bA[0]=bRegAddr;
	bA[1]=bRegSetVal;
	iChk = i2c_master_send(gptFP9928_data->ptI2C_client, (const char *)bA, sizeof(bA));
	if (iChk < 0) {
		ERR_MSG("%s(%d):%d=%s(),regAddr=0x%x,regVal=0x%x fail !\n",__FILE__,__LINE__,\
			iChk,"i2c_master_send",bRegAddr,bRegSetVal);
		iRet=FP9928_RET_I2CTRANS_ERR;
	}
	
	if(!in_interrupt()) {
		mutex_unlock(&gptFP9928_data->tI2CLock);
	}

	return iRet;
}

static int _fp9928_get_reg(unsigned char bRegAddr,unsigned char *O_pbRegVal)
{
	int iRet=FP9928_RET_SUCCESS;
	int iChk;
	unsigned char bA[1] ;

	ASSERT(gptFP9928_data);

	if(!in_interrupt()) {
		mutex_lock(&gptFP9928_data->tI2CLock);
	}


	bA[0]=bRegAddr;
	iChk = i2c_master_send(gptFP9928_data->ptI2C_client, (const char *)bA, 1);
	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_send fail !\n",__FILE__,__LINE__,__FUNCTION__);
		iRet = FP9928_RET_I2CTRANS_ERR;
	}
	

	iChk = i2c_master_recv(gptFP9928_data->ptI2C_client, bA, 1);
	if (iChk < 0) {
		ERR_MSG("%s(%d):%s i2c_master_recv fail !\n",__FILE__,__LINE__,__FUNCTION__);
		iRet = FP9928_RET_I2CTRANS_ERR;
	}


	if(iRet>=0) {
		*O_pbRegVal = bA[0];
	}

	
	//DBG_MSG("%s(0x%x,%p)==>0x%x\n",__FUNCTION__,bRegAddr,O_pbRegVal,bA[0]);

	if(!in_interrupt()) {
		mutex_unlock(&gptFP9928_data->tI2CLock);
	}

	return iRet;
}


#define FP9928_REG_SET(_regName,_bFieldName,_bSetVal)		\
({\
	int _iRet=FP9928_RET_SUCCESS;\
	int _iChk;\
	unsigned char _bNewReg,_bFieldMask;\
	\
	_bFieldMask=(unsigned char)FP9928_REG_##_regName##_##_bFieldName;\
	if(0xff==_bFieldMask) {\
		_bNewReg = _bSetVal;\
	}\
	else {\
		_bNewReg=gbFP9928_REG_##_regName;\
		if(_bSetVal) {\
			_bNewReg |= _bFieldMask ;\
		}\
		else {\
			_bNewReg &= ~_bFieldMask;\
		}\
	}\
	\
	_iChk = _fp9928_set_reg(gbFP9928_REG_##_regName##_##addr,_bNewReg);\
	if(_iChk<0) {\
		_iRet = _iChk;\
	}\
	else {\
		DBG_MSG("%s() : FP9928 write reg%s(%02Xh) 0x%02x->0x%02x\n",__FUNCTION__,\
		#_regName,gbFP9928_REG_##_regName##_##addr,gbFP9928_REG_##_regName,_bNewReg);\
		gbFP9928_REG_##_regName = _bNewReg;\
	}\
	_iRet;\
})

#define FP9928_REG_GET(_regName)		\
({\
	int _iChk;\
	unsigned char bReadReg=0;\
	unsigned short _wRet=0;\
	\
	_iChk = _fp9928_get_reg(gbFP9928_REG_##_regName##_##addr,&bReadReg);\
	if(_iChk<0) {\
		_wRet = (unsigned short)(-1);\
	}\
	else {\
		_wRet = bReadReg;\
		gbFP9928_REG_##_regName = bReadReg;\
		DBG_MSG("%s() : FP9928 read reg%s(%02Xh)=0x%02x\n",__FUNCTION__,\
			#_regName,gbFP9928_REG_##_regName##_##addr,bReadReg);\
	}\
	_wRet;\
})

#define FP9928_REG(_regName)	gbFP9928_REG_##_regName






static int _fp9928_gpio_init(void)
{
	int iRet = FP9928_RET_SUCCESS;

	GALLEN_DBGLOCAL_BEGIN();

	mxc_iomux_v3_setup_pad(GPIO_FP9928_VIN_PADCFG);
	if(0!=gpio_request(GPIO_FP9928_VIN, "fp9928_VIN")) {
		WARNING_MSG("%s(),request gpio fp9928_VIN fail !!\n",__FUNCTION__);
		gpio_direction_input(GPIO_FP9928_VIN);
	}

	mxc_iomux_v3_setup_pad(GPIO_FP9928_EN_PADCFG);
	if(0!=gpio_request(GPIO_FP9928_EN, "fp9928_EN")) {
		WARNING_MSG("%s(),request gpio fp9928_EN fail !!\n",__FUNCTION__);
		gpio_direction_input(GPIO_FP9928_EN);
	}

	mxc_iomux_v3_setup_pad(GPIO_FP9928_VCOM_PADCFG);
	gpio_request(GPIO_FP9928_VCOM, "fp9928_VCOM");
	if(0!=gpio_request(GPIO_FP9928_VCOM, "fp9928_VCOM")) {
		WARNING_MSG("%s(),request gpio fp9928_VCOM fail !!\n",__FUNCTION__);
		gpio_direction_input(GPIO_FP9928_VCOM);
	}

	GALLEN_DBGLOCAL_END();

	return iRet;
}

static void _fp9928_gpio_release(void)
{
	GALLEN_DBGLOCAL_BEGIN();
	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return ;
	}
	gpio_free(GPIO_FP9928_VCOM);
	gpio_free(GPIO_FP9928_EN);
	gpio_free(GPIO_FP9928_VIN);

	GALLEN_DBGLOCAL_END();
}


int _fp9928_reg_init(void)
{
	//FP9928_REG_SET(FUNC_ADJUST,FIX_RD_PTR,1);
	//FP9928_REG_GET(TMST);
	//FP9928_REG_GET(TMST);
}

int _fp9928_output_en(int iIsEnable)
{
	int iRet = FP9928_RET_SUCCESS;

	DBG_MSG("%s(%d)\n",__FUNCTION__,iIsEnable);

	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		return FP9928_RET_NOTINITEDSTATE;
	}

	if(gptFP9928_data->iIsOutputEnabled == iIsEnable) {
		// nothing have to do when state not change .
	}
	else {
		unsigned long dwTickNow = jiffies,dwTicks ;
		if(iIsEnable) {
			if ( time_before(dwTickNow,gptFP9928_data->dwTickPowerOffEnd) ) {
				WARNING_MSG("[WARNING] %s(%d) : Trunning ON the EPD power maybe in powering off state !!!\n",__FILE__,__LINE__);
				dwTicks = gptFP9928_data->dwTickPowerOffEnd-dwTickNow;

				DBG_MSG("%s() going to sleep %ld ticks \n",__FUNCTION__,dwTicks);
				msleep(jiffies_to_msecs(dwTicks));
			}

			gpio_direction_output(GPIO_FP9928_EN,EN_ON);
			gptFP9928_data->iIsOutputEnabled = 1;
			msleep(23);
			gpio_direction_output(GPIO_FP9928_VCOM,VCOM_ON);
			msleep(10);
		}
		else {

			gpio_direction_output(GPIO_FP9928_EN,EN_OFF);
			gptFP9928_data->iIsOutputEnabled = 0;
			gptFP9928_data->dwTickPowerOffEnd = jiffies + FP9928_POWEROFF_TICKS_MAX;
		}
	}

	return iRet ;
}



static int _fp9928_vin_onoff(int iIsON)
{
	int iRet = FP9928_RET_SUCCESS;

	ASSERT(gptFP9928_data);

	DBG_MSG("%s(%d)\n",__FUNCTION__,iIsON);
	
	if(iIsON==gptFP9928_data->iIsPoweredON) {
	}
	else {

		if(iIsON) {
			gpio_direction_output(GPIO_FP9928_VIN,VIN_ON);
			gptFP9928_data->iIsPoweredON = 1;
		}
		else {
			_fp9928_output_en(0);

			gpio_direction_output(GPIO_FP9928_VIN,VIN_OFF);
			gptFP9928_data->iIsPoweredON = 0;
		}
	}

	return iRet;
}

static void _fp9928_pwrdwn_work_func(struct work_struct *work)
{
	GALLEN_DBGLOCAL_BEGIN();

	mutex_lock(&gptFP9928_data->tPwrLock);

	if(!gptFP9928_data->iIsOutputPowerDownCounting) {
		WARNING_MSG("[WARNING]%s(%d): race condition occured !\n",__FILE__,__LINE__);
		return ;
	}

	_fp9928_output_en(0);
	gptFP9928_data->iIsOutputPowerDownCounting = 0;

	if(gptFP9928_data->tPwrdwn_work_param.iIsTurnOffChipPwr) {
		//msleep(FP9928_POWEROFF_TICKS_MAX/100);
		_fp9928_vin_onoff(0);
	}

	mutex_unlock(&gptFP9928_data->tPwrLock);

	GALLEN_DBGLOCAL_END();
}


/**********************************************************************
 *
 * public functions .
 *
***********************************************************************/

int fp9928_power_onoff(int iIsPowerOn,int iIsOutputPwr)
{
	int iRet = FP9928_RET_SUCCESS;

	GALLEN_DBGLOCAL_BEGIN();
	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return FP9928_RET_NOTINITEDSTATE;
	}

		

	mutex_lock(&gptFP9928_data->tPwrLock);
	if (iIsPowerOn) {
		_fp9928_vin_onoff(1);

		if(iIsOutputPwr==1) {
			gptFP9928_data->iIsOutputPowerDownCounting = 0;
			cancel_delayed_work_sync(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work);
			_fp9928_output_en(1);
		}
	}
	else {
		gptFP9928_data->tPwrdwn_work_param.iIsTurnOffChipPwr = 1;
		if(!gptFP9928_data->iIsOutputPowerDownCounting) 
		{
			gptFP9928_data->iIsOutputPowerDownCounting = 1;
			cancel_delayed_work_sync(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work);
			schedule_delayed_work(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work, \
					FP9928_PWROFFDELAYWORK_TICKS);
		}
	}
	mutex_unlock(&gptFP9928_data->tPwrLock);	

	GALLEN_DBGLOCAL_BEGIN();
	return iRet;
}

int fp9928_output_power(int iIsOutputPwr,int iIsChipPowerDown)
{
	int iRet=FP9928_RET_SUCCESS;

	GALLEN_DBGLOCAL_BEGIN();

	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		GALLEN_DBGLOCAL_ESC();
		return FP9928_RET_NOTINITEDSTATE;
	}


	mutex_lock(&gptFP9928_data->tPwrLock);
	if(iIsOutputPwr) {
		GALLEN_DBGLOCAL_RUNLOG(0);
		gptFP9928_data->iIsOutputPowerDownCounting = 0;
		cancel_delayed_work_sync(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work);

		if(!gptFP9928_data->iIsPoweredON) {
			GALLEN_DBGLOCAL_RUNLOG(1);;
			// auto power on chip .
			_fp9928_vin_onoff(1);
		}

		iRet = _fp9928_output_en(1);
	}
	else {
		GALLEN_DBGLOCAL_RUNLOG(2);
		if(!gptFP9928_data->iIsOutputPowerDownCounting) {
			GALLEN_DBGLOCAL_RUNLOG(3);

			udelay(100);gpio_direction_output(GPIO_FP9928_VCOM,VCOM_OFF);

			gptFP9928_data->tPwrdwn_work_param.iIsTurnOffChipPwr = iIsChipPowerDown;
			gptFP9928_data->iIsOutputPowerDownCounting = 1;
			cancel_delayed_work_sync(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work);
			schedule_delayed_work(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work, \
					FP9928_PWROFFDELAYWORK_TICKS);
		}
		else {
			GALLEN_DBGLOCAL_RUNLOG(4);
			DBG_MSG("%s(%d),power down work already exist \n",__FUNCTION__,__LINE__);
		}
	}
	mutex_unlock(&gptFP9928_data->tPwrLock);

	GALLEN_DBGLOCAL_END();
	return iRet;
}

int fp9928_suspend(void)
{
	int iRet = FP9928_RET_SUCCESS;
	int iChk;

	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		return FP9928_RET_NOTINITEDSTATE;
	}

	if(gptFP9928_data->iIsOutputEnabled) {
		WARNING_MSG("%s() : skip suspend when PMIC output enabled !!\n",__FUNCTION__);
		return FP9928_RET_PWRDWNWORKPENDING;
	}

	if(gSleep_Mode_Suspend) {
		mutex_lock(&gptFP9928_data->tPwrLock);	
		_fp9928_vin_onoff(0);
		mutex_unlock(&gptFP9928_data->tPwrLock);
	}

	return iRet;
}

void fp9928_resume(void)
{
	if(!gptFP9928_data) {
		WARNING_MSG("%s(%d) : %s cannot work before init !\n",__FILE__,__LINE__,__FUNCTION__);
		return ;
	}

	if(gSleep_Mode_Suspend) {
		mutex_lock(&gptFP9928_data->tPwrLock);	
		_fp9928_vin_onoff(1);
		mutex_unlock(&gptFP9928_data->tPwrLock);
	}
}



int fp9928_get_temperature(int *O_piTemperature)
{
	unsigned short wReg;
	int iRet=FP9928_RET_SUCCESS;

	int iTemp;
	unsigned char bReg,bTemp;

	//return FP9928_RET_SUCCESS;

	if(!gptFP9928_data) {
		WARNING_MSG("%s() cannot work if driver is not initialed \n",__FUNCTION__);
		return FP9928_RET_NOTINITEDSTATE;
	}

	//fp9928_output_power(1,0);
	
	wReg = FP9928_REG_GET(TMST);

	//fp9928_output_power(0,0);

	if(((unsigned short)(-1))==wReg) {
		ERR_MSG("%s(%d):%s regTMST read fail !\n",__FILE__,__LINE__,__FUNCTION__);
		return FP9928_RET_I2CTRANS_ERR;
	}

	bReg = (unsigned char)wReg;
	gptFP9928_data->wTempratureData = wReg;
	if(bReg&0x80) {
		// negative .
		bTemp=(~bReg)+1;
		iTemp = bTemp;
		iTemp = (~iTemp)+1;
	}
	else {
		// positive .
		iTemp = (int)(bReg);
	}
	gptFP9928_data->iCurrent_temprature = iTemp;
	printk("%s temprature data = 0x%x,%d\n",DRIVER_NAME,wReg,gptFP9928_data->iCurrent_temprature);

	if(O_piTemperature) {
		*O_piTemperature = gptFP9928_data->iCurrent_temprature;
	}

	return iRet;
}





int fp9928_vcom_set(int iVCOM_mV,int iIsWriteToFlash)
{

	const int iVCOM_mV_max=FP9928_VCOM_MV_MAX,iVCOM_mV_min=FP9928_VCOM_MV_MIN;
	int iVCOM_mV_ABS ;
	int iRet=FP9928_RET_SUCCESS;

	if(!gptFP9928_data) {
		WARNING_MSG("%s() cannot work if driver is not initialed \n",__FUNCTION__);
		return FP9928_RET_NOTINITEDSTATE;
	}
	
	if(iVCOM_mV<iVCOM_mV_min) {
		ERR_MSG("%s(%d),VCOM %d cannot < %d mV\n",
				__FUNCTION__,__LINE__,iVCOM_mV,iVCOM_mV_min);
	}
	else if(iVCOM_mV>iVCOM_mV_max) {
		ERR_MSG("%s(%d),VCOM %d cannot > %d\n",
				__FUNCTION__,__LINE__,iVCOM_mV,iVCOM_mV_max);
	}
	else {
		unsigned char bReg;
		if(iVCOM_mV<0) {
			iVCOM_mV_ABS = -iVCOM_mV;
		}
		else {
			iVCOM_mV_ABS = iVCOM_mV;
		}

		bReg = (unsigned char) (iVCOM_mV_ABS/FP9928_VCOM_MV_STEP);
		iRet = FP9928_REG_SET(VCOM_SETTING,ALL,bReg);
		if(iRet>=0) {
			gptFP9928_data->iCurrent_VCOM_mV=-((int)FP9928_REG(VCOM_SETTING)*FP9928_VCOM_MV_STEP);
		}
	}

	return iRet;
}


int fp9928_vcom_get(int *O_piVCOM_mV)
{
	int iVCOM_mV;
	unsigned short wReg;

	if(!gptFP9928_data) {
		WARNING_MSG("%s() cannot work if driver is not initialed \n",__FUNCTION__);
		return FP9928_RET_NOTINITEDSTATE;
	}

	wReg = FP9928_REG_GET(VCOM_SETTING);
	iVCOM_mV = (int)(wReg);
	iVCOM_mV = -(iVCOM_mV*FP9928_VCOM_MV_STEP);
	//DBG_MSG("%s(%d):iVCOM_mV=%d,wReg=0x%x\n",__FUNCTION__,__LINE__,iVCOM_mV,wReg);

	if(iVCOM_mV!=gptFP9928_data->iCurrent_VCOM_mV) {
		WARNING_MSG("%s(%d) VCOM read from register is not equal with stored \n",__FUNCTION__,__LINE__);
	}

	if(O_piVCOM_mV) {
		*O_piVCOM_mV = iVCOM_mV;
	}


	return iVCOM_mV;
}


void fp9928_release(void)
{
	if(!gptFP9928_data) {
		WARNING_MSG("%s() cannot work if driver is not initialed \n",__FUNCTION__);
		return ;
	}


	mutex_lock(&gptFP9928_data->tPwrLock);	
	_fp9928_vin_onoff(0);
	mutex_unlock(&gptFP9928_data->tPwrLock);	

	gptFP9928_data->ptI2C_adapter = 0;
	i2c_unregister_device(gptFP9928_data->ptI2C_client);
	gptFP9928_data->ptI2C_client = 0;
	
	_fp9928_gpio_release();

	kfree(gptFP9928_data);gptFP9928_data = 0;

}

int fp9928_init(int iPort)
{

	int iRet = FP9928_RET_SUCCESS;
	int iChk;

	unsigned long dwSize=sizeof(FP9928_data);

	GALLEN_DBGLOCAL_BEGIN();

	gptFP9928_data = kmalloc(dwSize,GFP_KERNEL);
	if(!gptFP9928_data) {
		iRet = FP9928_RET_MEMNOTENOUGH;
		ERR_MSG("%s(%d) : memory not enough !!\n",__FILE__,__LINE__);
		GALLEN_DBGLOCAL_RUNLOG(0);
		goto MEM_MALLOC_FAIL;
	}

	memset(gptFP9928_data,0,sizeof(FP9928_data));
	
	iChk = _fp9928_gpio_init();
	if(iChk<0) {
		iRet = FP9928_RET_GPIOINITFAIL;
		ERR_MSG("%s(%d) : gpio init fail !!\n",__FILE__,__LINE__);
		GALLEN_DBGLOCAL_RUNLOG(1);
		goto GPIO_INIT_FAIL;
	}

	gptFP9928_data->ptI2C_adapter = i2c_get_adapter(iPort-1);//
	if( NULL == gptFP9928_data->ptI2C_adapter) {
		ERR_MSG ("[Error] %s : FP9928_RET_I2CCHN_NOTFOUND,chn=%d\n",__FUNCTION__,iPort);
		GALLEN_DBGLOCAL_RUNLOG(2);
		iRet=FP9928_RET_I2CCHN_NOTFOUND;
		goto I2CCHN_GET_FAIL;
	}

	gptFP9928_data->ptI2C_client = i2c_new_probed_device(gptFP9928_data->ptI2C_adapter, &gtFP9928_BI,gwFP9928_AddrA,0);
	if( NULL == gptFP9928_data->ptI2C_client ) {
		GALLEN_DBGLOCAL_RUNLOG(3);
		ERR_MSG("[Error] %s : FP9928 probe fail \n",__FUNCTION__);
		goto I2CPROBE_DEVICE_FAIL;
	}

	gptFP9928_data->dwTickPowerOffEnd = jiffies;
	gptFP9928_data->iCurrent_VCOM_mV=-((int)FP9928_REG(VCOM_SETTING)*FP9928_VCOM_MV_STEP);// default VCOM voltage .

	// kernel objects initialize ...
	mutex_init(&gptFP9928_data->tPwrLock);
	mutex_init(&gptFP9928_data->tI2CLock);

	INIT_DELAYED_WORK(&gptFP9928_data->tPwrdwn_work_param.pwrdwn_work,_fp9928_pwrdwn_work_func);

	_fp9928_vin_onoff(1);

	_fp9928_reg_init();

	GALLEN_DBGLOCAL_ESC();
	return FP9928_RET_SUCCESS;


	i2c_unregister_device(gptFP9928_data->ptI2C_client);
	gptFP9928_data->ptI2C_client = 0;
I2CPROBE_DEVICE_FAIL:
	gptFP9928_data->ptI2C_adapter = 0;
I2CCHN_GET_FAIL:
	_fp9928_gpio_release();
GPIO_INIT_FAIL:
	kfree(gptFP9928_data);gptFP9928_data = 0;
MEM_MALLOC_FAIL:
	
	GALLEN_DBGLOCAL_END();
	return iRet;
}


