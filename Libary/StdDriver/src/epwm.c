/*******************************************************************************
* Copyright (C) 2019 China Micro Semiconductor Limited Company. All Rights Reserved.
*
* This software is owned and published by:
* CMS LLC, No 2609-10, Taurus Plaza, TaoyuanRoad, NanshanDistrict, Shenzhen, China.
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with CMS
* components. This software is licensed by CMS to be adapted only
* for use in systems utilizing CMS components. CMS shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. CMS is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/

/****************************************************************************/
/** \file epwm.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "epwm.h"
/****************************************************************************/
/*	Local pre-processor symbols/macros('#define')
*****************************************************************************/

/****************************************************************************/
/*	Global variable definitions(declared in header file with 'extern')
*****************************************************************************/

/****************************************************************************/
/*	Local type definitions('typedef')
*****************************************************************************/

/****************************************************************************/
/*	Local variable  definitions('static')
*****************************************************************************/


/****************************************************************************/
/*	Local function prototypes('static')
*****************************************************************************/

/****************************************************************************/
/*	Function implementation - global ('extern') and local('static')
*****************************************************************************/

/*****************************************************************************
 ** \brief	 EPWM_ConfigRunMode
 **			 设置EPWM的运行模式
 ** \param [in] EpwmRunMode: (1)EPWM_COUNT_UP_DOWN
							   (2)EPWM_COUNT_DOWN
							   (3)EPWM_OCU_ASYMMETRIC
							   (4)EPWM_OCU_SYMMETRIC
							   (5)EPWM_WFG_INDEPENDENT
							   (6)EPWM_WFG_COMPLEMENTARY
							   (7)EPWM_WFG_SYNCHRONIZED
							   (8)EPWM_OC_MODE_GROUP
							   (9)EPWM_OC_INDEPENDENT
 ** \return  none
 ** \note    
*****************************************************************************/
void EPWM_ConfigRunMode(uint32_t EpwmRunMode)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;
	EPWM->CON &= ~(EPWM_CON_MODE_Msk | EPWM_CON_GROUNPEN_Msk | EPWM_CON_ASYMEN_Msk);
	EPWM->CON |= EpwmRunMode;
	EPWM->LOCK = 0x0;
}
/*****************************************************************************
 ** \brief	 EPWM_ConfigChannelClk	
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **				ClkDiv: (1)EPWM_CLK_DIV_1 
 **						(2)EPWM_CLK_DIV_2
 **						(3)EPWM_CLK_DIV_4
 **						(4)EPWM_CLK_DIV_8
 **						(5)EPWM_CLK_DIV_16
 **						(6)EPWM_CLK_DIV_32
 ** \return  none	
 ** \note 
 **	(1)例因为CLKPSC01共同控制PWM0、PWM1通道 ，所以此函数统一设置CLKPSC01=0x01
 **     若需要其他的分频，请酌情对寄存器赋值.
 **	(2)EPWM 分频分为两级：CLKPSC CLKDIV
 **		第一级分频(CLKPSC)：0x00 预分频停止并且计数器停止 。其他：系统时钟分频系数 CLKPSC+1
 **		第二级分频（CLKDIV): 0x00 FEPWMPSC/2; 0x01 FEPWMPSC/4；0x02:FEPWMPSC/8；0x3 FEPWMPSC/16 0x04 FEPWMPSC ; 其他 Fsys 
 ** 	分频级数：1~4096，例Fsys=8M，EPWM的频率范围：1.95k ~ 8M	   
*****************************************************************************/
void EPWM_ConfigChannelClk(uint32_t EPWMn,uint32_t ClkDiv)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->CLKPSC = (0x01<<EPWM_CLKPSC_CLKPSC45_Pos |
					0x01<<EPWM_CLKPSC_CLKPSC23_Pos |
					0x01<<EPWM_CLKPSC_CLKPSC01_Pos);
	EPWM->CLKDIV &= ~(0xfUL <<(EPWMn*4));
	EPWM->CLKDIV |= (ClkDiv <<(EPWMn*4));		
	EPWM->LOCK = 0x0;
}

/*****************************************************************************
 ** \brief	 EPWM_ConfigChannelPeriod	
 **			 设置EPWM通道的周期
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **				Period: 16位周期值
 ** \return none
 ** \note    
*****************************************************************************/
void EPWM_ConfigChannelPeriod(uint32_t EPWMn, uint32_t Period)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;		
	EPWM->PERIOD[EPWMn] = 0xffff & Period;
	EPWM->CON3 |= (0x1UL <<(EPWMn + EPWM_CON3_LOADEN0_Pos));		/*使能加载周期&&占空比*/
	EPWM->LOCK = 0x0;
}

/*****************************************************************************
 ** \brief	 EPWM_ConfigChannelSymDuty	
 **			 设置对称模式下的EPWM通道的占空比
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **			    Duty  : 16位比较值
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_ConfigChannelSymDuty(uint32_t EPWMn,uint32_t Duty)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;		
	EPWM->CMPADT[EPWMn] &= 0xffff0000;
	EPWM->CMPADT[EPWMn] |= 0xffff & Duty;	
	EPWM->CON3 |= (0x1UL <<(EPWMn +EPWM_CON3_LOADEN0_Pos));		/*使能加载周期&&占空比*/
	EPWM->LOCK = 0x0;
}

/*****************************************************************************
 ** \brief	 EPWM_ConfigChannelAsymDuty	
 **			 设置非对称模式下的 EPWM通道的占空比
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **			    UpCmp 	  : 16位向上比较值
 **			    DownCmp   : 16位向下比较值
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_ConfigChannelAsymDuty(uint32_t EPWMn,uint32_t UpCmp, uint32_t DownCmp)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;		
	EPWM->CMPADT[EPWMn] = (0xffff & UpCmp) | ((0xffff & DownCmp)<<16);	
	EPWM->CON3 |= (0x1UL <<(EPWMn +EPWM_CON3_LOADEN0_Pos));		/*使能加载周期&&占空比*/
	EPWM->LOCK = 0x0;
}

/*****************************************************************************
 ** \brief	 EPWM_EnableOneShotMode	
 **			 使能单次模式
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_EnableOneShotMode(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->CON &= ~(EPWMnMask); 
	EPWM->LOCK = 0x0;
}
/*****************************************************************************
 ** \brief	 EPWM_EnableAutoLoadMode	
 **			 使能连续加载模式
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_EnableAutoLoadMode(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->CON |= EPWMnMask; 
	EPWM->LOCK = 0x0;
}
/*****************************************************************************
 ** \brief	 EPWM_ConfigLoadAndIntMode
 **			 配置加载方式以及中断方式
 ** \param [in]  EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **				 Mode: (1)EPWM_EACH_PERIOD_ZERO
 **					   (2)EPWM_EACH_ZERO
 **					   (3)EPWM_FIRST_ZERO_NEXT_PERIOD
 **					   (4)EPWM_EVERY_TWO_ZERO
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_ConfigLoadAndIntMode(uint32_t EPWMn, uint32_t Mode)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->CON3 &= ~(0x3UL<< ((EPWMn*2) + EPWM_CON3_LOADTYP0_Pos));
	EPWM->CON3 |= Mode << ((EPWMn*2) + EPWM_CON3_LOADTYP0_Pos);	
	EPWM->LOCK = 0x0;			
}


/*****************************************************************************
 ** \brief	 EPWM_EnableReverseOutput	
 **			 开启反相输出
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_EnableReverseOutput(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->CON |= (EPWMnMask<< EPWM_CON_PINV0_Pos); 
	EPWM->LOCK = 0x0;	
}
/*****************************************************************************
 ** \brief	 EPWM_DisableReverseOutput	
 **			 关闭反相输出
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_DisableReverseOutput(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->CON &= ~(EPWMnMask<< EPWM_CON_PINV0_Pos); 
	EPWM->LOCK = 0x0;	
}

/*****************************************************************************
 ** \brief	 EPWM_EnableDeadZone	
 **			 开启死区
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 **				DeadZone : 0x00 ~ 0x3FF;
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_EnableDeadZone(uint32_t EPWMnMask, uint32_t DeadZone)
{
	
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	if(( EPWMnMask & EPWM_CH_0_MSK) || (EPWMnMask & EPWM_CH_1_MSK))
	{
		EPWM->CON |= EPWM_CON_ENDT01_Msk;
		EPWM->DTCTL &= ~(EPWM_DTCTL_DTI01_Msk);
		EPWM->DTCTL |= 0x3ff & DeadZone;
	}
	if(( EPWMnMask & EPWM_CH_2_MSK) || (EPWMnMask & EPWM_CH_3_MSK))
	{
		EPWM->CON |= EPWM_CON_ENDT23_Msk;
		EPWM->DTCTL &= ~(EPWM_DTCTL_DTI23_Msk);
		EPWM->DTCTL |=  ((0x3ff & DeadZone)<<EPWM_DTCTL_DTI23_Pos);
	}	
	if(( EPWMnMask & EPWM_CH_4_MSK) || (EPWMnMask & EPWM_CH_5_MSK))
	{
		EPWM->CON |= EPWM_CON_ENDT45_Msk;
		EPWM->DTCTL &= ~(EPWM_DTCTL_DTI45_Msk);
		EPWM->DTCTL |= (0x3ff & DeadZone) << EPWM_DTCTL_DTI45_Pos;
	}	
	EPWM->LOCK = 0x0;	
}

/*****************************************************************************
 ** \brief	 EPWM_DisableDeadZone	
 **			 关闭死区
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_DisableDeadZone(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	if(( EPWMnMask & EPWM_CH_0_MSK) || (EPWMnMask & EPWM_CH_1_MSK))
	{
		EPWM->CON &= ~(EPWM_CON_ENDT01_Msk);
	}
	if(( EPWMnMask & EPWM_CH_2_MSK) || (EPWMnMask & EPWM_CH_3_MSK))
	{
		EPWM->CON &= ~(EPWM_CON_ENDT23_Msk);
	}	
	if(( EPWMnMask & EPWM_CH_4_MSK) || (EPWMnMask & EPWM_CH_5_MSK))
	{
		EPWM->CON &= ~(EPWM_CON_ENDT45_Msk);
	}	
	EPWM->LOCK = 0x0;	
}


/*****************************************************************************
 ** \brief	 EPWM_EnableOutput
 **			 开启EPWM通道输出
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note  
*****************************************************************************/
void EPWM_EnableOutput(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->POEN |= EPWMnMask;
	EPWM->LOCK = 0x0;
}
/*****************************************************************************
 ** \brief	 EPWM_DisableOutput
 **			 关闭EPWM通道输出
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_DisableOutput(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->POEN &= ~(EPWMnMask);
	EPWM->LOCK = 0x0;	
}

/*****************************************************************************
 ** \brief	 EPWM_Start	
 **			 开启EPWM
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_Start(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->CON2 |= EPWMnMask;	
	EPWM->LOCK = 0x0;
}
/*****************************************************************************
 ** \brief	 EPWM_Stop	
 **			 关闭EPWM
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_Stop(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->CON2 &= ~(EPWMnMask);			
	EPWM->CON3 |= EPWMnMask;			/*计数清0*/
	EPWM->LOCK = 0x0;
}
/*****************************************************************************
 ** \brief	 EPWM_ForceStop	
 **			 强制关闭EPWM
 ** \param [in]  none
 ** \return none
 ** \note   使用掩码控制EPWM
*****************************************************************************/
void EPWM_ForceStop(void)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	
	EPWM->LOCK = 0x0;	
}

/*****************************************************************************
 ** \brief	 EPWM_EnableUpCmpInt
 **			 开启向上比较中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_EnableUpCmpInt(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->IMSC |= (EPWMnMask<<EPWM_IMSC_ENUIFn_Pos);
	EPWM->LOCK = 0x0;		
}
/*****************************************************************************
 ** \brief	 EPWM_DisableUpCmpInt
 **			 关闭向上比较中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_DisableUpCmpInt(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->IMSC &= ~(EPWMnMask<<EPWM_IMSC_ENUIFn_Pos);
	EPWM->LOCK = 0x0;	
}
/*****************************************************************************
 ** \brief	 EPWM_GetUpCmpIntFlag
 **			获取向上比较中断使能并产生中断标志位
 ** \param [in]EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return 1：中断使能并且产生中断  0：无中断
 ** \note  
*****************************************************************************/
uint32_t  EPWM_GetUpCmpIntFlag(uint32_t EPWMn)
{
	return ((EPWM->MIS&(0x01UL<<(EPWMn+EPWM_MIS_UIFn_Pos)))? 1:0);
}
/*****************************************************************************
 ** \brief	 EPWM_ClearUpCmpIntFlag
 **			清除向上比较中断标志位
 ** \param [in]EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_ClearUpCmpIntFlag(uint32_t EPWMn)
{
	EPWM->ICLR |= (0x1UL<<(EPWMn+EPWM_ICLR_UIFn_Pos));
}

/*****************************************************************************
 ** \brief	 EPWM_EnableDownCmpInt
 **			 开启向下比较中断
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_EnableDownCmpInt(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->IMSC |= (EPWMnMask<<EPWM_IMSC_ENDIFn_Pos);
	EPWM->LOCK = 0x0;		
}
/*****************************************************************************
 ** \brief	 EPWM_DisableDownCmpInt
 **			 关闭向下比较中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_DisableDownCmpInt(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->IMSC &= ~(EPWMnMask<<EPWM_IMSC_ENDIFn_Pos);
	EPWM->LOCK = 0x0;
}
/*****************************************************************************
 ** \brief	 EPWM_GetDownCmpIntFlag
 **			获取向下比较中断使能并产生中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return 1：中断使能并且产生中断  0：无中断
 ** \note  
*****************************************************************************/
uint32_t  EPWM_GetDownCmpIntFlag(uint32_t EPWMn)
{
	return ((EPWM->MIS&(0x01UL<<(EPWMn+EPWM_MIS_DIFn_Pos)))? 1:0);
}
/*****************************************************************************
 ** \brief	 EPWM_ClearDownCmpIntFlag
 **			清除向下比较中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_ClearDownCmpIntFlag(uint32_t EPWMn)
{
	EPWM->ICLR |= (0x1UL<<(EPWMn+EPWM_ICLR_DIFn_Pos));	
}


/*****************************************************************************
 ** \brief	 EPWM_EnablePeriodInt
 **			 开启周期中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_EnablePeriodInt(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->IMSC |= (EPWMnMask<<EPWM_IMSC_ENPIFn_Pos);
	EPWM->LOCK = 0x0;		
}
/*****************************************************************************
 ** \brief	 EPWM_DisablePeriodInt
 **			 关闭周期中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_DisablePeriodInt(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->IMSC &= ~(EPWMnMask<<EPWM_IMSC_ENPIFn_Pos);
	EPWM->LOCK = 0x0;		
}
/*****************************************************************************
 ** \brief	 EPWM_GetPeriodIntFlag
 **			获取周期中断使能并产生中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return 1：中断使能并且产生中断  0：无中断
 ** \note  
*****************************************************************************/
uint32_t  EPWM_GetPeriodIntFlag(uint32_t EPWMn)
{
	return ((EPWM->MIS&(0x01UL<<(EPWMn+EPWM_MIS_PIFn_Pos)))? 1:0);
}
/*****************************************************************************
 ** \brief	 EPWM_ClearPeriodIntFlag
 **			清除周期中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_ClearPeriodIntFlag(uint32_t EPWMn)
{
	EPWM->ICLR |= (0x1UL<<(EPWMn+EPWM_ICLR_PIFn_Pos));
}

/*****************************************************************************
 ** \brief	 EPWM_EnableZeroInt
 **			 开启零点中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_EnableZeroInt(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->IMSC |= (EPWMnMask<<EPWM_IMSC_ENZIFn_Pos);
	EPWM->LOCK = 0x0;	
}
/*****************************************************************************
 ** \brief	 EPWM_DisableZeroInt
 **			 关闭零点中断
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_DisableZeroInt(uint32_t EPWMnMask)
{
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;	
	EPWM->IMSC &= ~(EPWMnMask<<EPWM_IMSC_ENZIFn_Pos);
	EPWM->LOCK = 0x0;	
}
/*****************************************************************************
 ** \brief	 EPWM_GetZeroIntFlag
 **			获取零点中断使能并产生中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return 1：中断使能并且产生中断  0：无中断
 ** \note  
*****************************************************************************/
uint32_t  EPWM_GetZeroIntFlag(uint32_t EPWMn)
{
	return ((EPWM->MIS&(0x01UL<<(EPWMn+EPWM_MIS_ZIFn_Pos)))? 1:0);
}
/*****************************************************************************
 ** \brief	 EPWM_ClearZeroIntFlag
 **			清除零点中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_ClearZeroIntFlag(uint32_t EPWMn)
{
	EPWM->ICLR |= (0x1UL<<(EPWMn+EPWM_ICLR_ZIFn_Pos));
}
