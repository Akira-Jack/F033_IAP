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

/*****************************************************************************/
/** \file epwm.h
**
** History:
** 
*****************************************************************************/
#ifndef __EPWM_H_
#define __EPWM_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include "cms32f033x.h"
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/
/*----------------------------------------------------------------------------
 **EPWM 通道
-----------------------------------------------------------------------------*/
#define	EPWM0 			(0x00UL)
#define	EPWM1 			(0x01UL)
#define	EPWM2 			(0x02UL)
#define	EPWM3 			(0x03UL)
#define	EPWM4 			(0x04UL)
#define	EPWM5 			(0x05UL)

#define	EPWM_CH_0_MSK 		(0x01UL)
#define	EPWM_CH_1_MSK 		(0x02UL)
#define	EPWM_CH_2_MSK 		(0x04UL)
#define	EPWM_CH_3_MSK 		(0x08UL)
#define EPWM_CH_4_MSK 		(0x10UL)
#define	EPWM_CH_5_MSK		(0x20UL)

/*----------------------------------------------------------------------------
 **EPWM 重映射
-----------------------------------------------------------------------------*/
#define	IPG0			(0x00UL)
#define	IPG1			(0x01UL)
#define	IPG2			(0x02UL)
#define	IPG3			(0x03UL)
#define	IPG4			(0x04UL)
#define	IPG5			(0x05UL)
#define EPWM_REMAP_ENABLE	(0xAAUL)
/*----------------------------------------------------------------------------
 **EPWM 模式
-----------------------------------------------------------------------------*/
/*------EPWM计数模式---------------------------------------------------------*/
#define EPWM_COUNT_UP_DOWN			(0x01UL<<EPWM_CON_CNTTYPE_Pos)  	/*EPWM计数（COUNT）模式：上下计数（中心对齐）模式*/
#define EPWM_COUNT_DOWN				(0x00UL)  							/*EPWM计数（COUNT）模式：向下计数（边沿对齐）模式*/
/*------EPWM输出比较单元(OCU)模式--------------------------------------------*/
#define EPWM_OCU_ASYMMETRIC			(0x01UL<<EPWM_CON_ASYMEN_Pos)  		/*EPWM输出比较单元（OCU）模式； 非对称模式*/
#define EPWM_OCU_SYMMETRIC			(0x00UL) 					 		/*EPWM输出比较单元（OCU）模式； 对称模式*/
/*------EPWM波形发生器单元(WFG)模式------------------------------------------*/
#define EPWM_WFG_INDEPENDENT		(0x0UL)  							/*EPWM波形发生器（WFG）模式：单个（独立）模式*/
#define EPWM_WFG_COMPLEMENTARYK 	(0x1UL<<EPWM_CON_MODE_Pos) 			/*EPWM波形发生器（WFG）模式：互补模式*/
#define EPWM_WFG_SYNCHRONIZED		(0x2UL<<EPWM_CON_MODE_Pos) 			/*EPWM波形发生器（WFG）模式：同步模式*/
/*------EPWM输出控制单元(OC)模式---------------------------------------------*/
#define EPWM_OC_GROUP				(0x01UL<<EPWM_CON_GROUNPEN_Pos)  	/*EPWM输出控制（OC）模式： 组控模式*/
#define EPWM_OC_INDEPENDENT			(0x00UL) 						 	/*EPWM输出控制（OC）模式： 通道各自独立控制*/

/*----------------------------------------------------------------------------
 **EPWM 时钟
-----------------------------------------------------------------------------*/
#define 	EPWM_CLK_DIV_1				(0x5UL)		
#define 	EPWM_CLK_DIV_2				(0x4UL)		
#define	 	EPWM_CLK_DIV_4				(0x0UL)		
#define	 	EPWM_CLK_DIV_8				(0x1UL)		
#define 	EPWM_CLK_DIV_16				(0x2UL)		
#define 	EPWM_CLK_DIV_32				(0x3UL)		

/*----------------------------------------------------------------------------
 **EPWM 周期、占空比加载方式及周期中断、零点中断产生方式
-----------------------------------------------------------------------------*/
#define		EPWM_EACH_PERIOD_ZERO		(0x0UL)		/*每个零点与周期点*/
#define		EPWM_EACH_ZERO				(0x1UL)		/*每个零点*/
#define		EPWM_FIRST_ZERO_NEXT_PERIOD	(0x2UL)		/*第一个零点与下一个周期点*/
#define		EPWM_EVERY_TWO_ZERO			(0x3UL)		/*每两个零点*/

/*----------------------------------------------------------------------------
 **EPWM 写保护
-----------------------------------------------------------------------------*/
#define		EPWM_LOCK_P1A_WRITE_KEY		(0xAAUL)
#define		EPWM_LOCK_P1B_WRITE_KEY		(0x55UL)



/*****************************************************************************
 ** \brief	EPWM_GET_UPCMP_RIS_FLAG
 **			获取向上比较中断源标志
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 ** \return  1：产生中断  0:无中断
 ** \note	 
*****************************************************************************/
#define 	EPWM_GET_UPCMP_RIS_FLAG(EPWMn)		(((EPWM->RIS&(0x1<<(EPWMn+EPWM_RIS_UIFn_Pos))? 1:0)))
		
/*****************************************************************************
 ** \brief	EPWM_GET_DOWNCMP_RIS_FLAG
 **			获取向下比较中断源标志
 ** \param [in]EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 ** \return  1：产生中断  0:无中断
 ** \note	 
*****************************************************************************/
#define 	EPWM_GET_DOWNCMP_RIS_FLAG(EPWMn)		(((EPWM->RIS&(0x1<<(EPWMn+EPWM_RIS_DIFn_Pos)))? 1:0))


/*****************************************************************************
 ** \brief	EPWM_GET_PERIOD_RIS_FLAG
 **			获取周期中断源标志
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 ** \return  1：产生中断  0:无中断
 ** \note	 
*****************************************************************************/
#define 	EPWM_GET_PERIOD_RIS_FLAG(EPWMn)		(((EPWM->RIS&(0x1<<(EPWMn+EPWM_RIS_PIFn_Pos)))? 1:0))
		
/*****************************************************************************
 ** \brief	EPWM_GET_ZERO_RIS_FLAG
 **			获取零点中断源标志
 ** \param [in]EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 ** \return  1：产生中断  0:无中断
 ** \note	 
*****************************************************************************/
#define 	EPWM_GET_ZERO_RIS_FLAG(EPWMn)	(((EPWM->RIS&(0x1<<(EPWMn+EPWM_RIS_ZIFn_Pos)))? 1:0))


/*****************************************************************************
 ** \brief	EPWM_SET_COMPARE
 **			设置EPWMn比较值
 ** \param [in]EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **			   CMR : 16位比较器值	
 ** \return  
 ** \note	 
*****************************************************************************/
#define 	EPWM_SET_COMPARE(EPWMn,CMR)	 do{\
											EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;\
											EPWM->CMPADT[EPWMn] &= 0xffff0000;\
											EPWM->CMPADT[EPWMn] |= 0xffff & CMR;\
											EPWM->CON3 |= (0x1UL <<(EPWMn +EPWM_CON3_LOADEN0_Pos ));\
											EPWM->LOCK = 0x0;\
										  }while(0)
/*****************************************************************************
 ** \brief	EPWM_SET_PERIOD
 **			设置EPWMn周期值
 ** \param [in]EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **			   period : 16位周期值	
 ** \return  
 ** \note	 
*****************************************************************************/
#define 	EPWM_SET_PERIOD(EPWMn,period)	do{\
												EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;\
												EPWM->PERIOD[EPWMn] = 0xffff & Period;\
												EPWM->CON3 |= (0x1UL <<(EPWMn +EPWM_CON3_LOADEN0_Pos ));\
												EPWM->LOCK = 0x0;\
											}while(0)

/*****************************************************************************
 ** \brief	EPWM_SET_COMPARE_D
 **			设置EPWMn向下比较值
 ** \param [in]EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **			   CMRD : 16位向下比较器值	
 ** \return  
 ** \note	 
*****************************************************************************/
#define 	EPWM_SET_COMPARE_D(EPWMn,CMRD)	do{\
													EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;\
													EPWM->CMPADT[EPWMn] &= 0xffff;\
													EPWM->CMPADT[EPWMn] |= ((0xffff & CMRD)<< 16);\
													EPWM->CON3 |= (0x1UL <<(EPWMn +EPWM_CON3_LOADEN0_Pos ));\
													EPWM->LOCK = 0x0;\
											 }while(0)
	
											 
/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/


/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/


/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/

/*****************************************************************************
 ** \brief	 EPWM_ConfigRunMode
 **			 设置EPWM的运行模式
 ** \param [in] EpwmRunMode:   (1)EPWM_COUNT_UP_DOWN
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
void EPWM_ConfigRunMode(uint32_t EpwmRunMode);

/*****************************************************************************
 ** \brief	 EPWM_ConfigChannelClk	
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **				ClkDiv: (1)EPWM_CLK_DIV_1 
 **						(2)EPWM_CLK_DIV_2
 **						(2)EPWM_CLK_DIV_4
 **						(2)EPWM_CLK_DIV_8
 **						(2)EPWM_CLK_DIV_16
 **						(2)EPWM_CLK_DIV_32
 ** \return  none	
 ** \note 
 **	(1)例因为CLKPSC01共同控制PWM0、PWM1通道 ，所以此函数统一设置CLKPSC01=0x01
 **     若需要其他的分频，请酌情对寄存器赋值.
 **	(2)EPWM 分频分为两级：CLKPSC CLKDIV
 **		第一级分频(CLKPSC)：0x00 预分频停止并且计数器停止 。其他：系统时钟分频系数 CLKPSC+1
 **		第二级分频（CLKDIV): 0x00 FEPWMPSC/2; 0x01 FEPWMPSC/4；0x02:FEPWMPSC/8；0x3 FEPWMPSC/16 0x04 FEPWMPSC ; 其他 Fsys 
 ** 	分频级数：1~4096，例Fsys=8M，EPWM的频率范围：1.95k ~ 8M	   
*****************************************************************************/
void EPWM_ConfigChannelClk(uint32_t EPWMn,uint32_t ClkDiv);

/*****************************************************************************
 ** \brief	 EPWM_ConfigChannelPeriod	
 **			 设置EPWM通道的周期
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **				Period: 16位周期值
 ** \return none
 ** \note    
*****************************************************************************/
void EPWM_ConfigChannelPeriod(uint32_t EPWMn, uint32_t Period);

/*****************************************************************************
 ** \brief	 EPWM_ConfigChannelSymDuty	
 **			 设置对称模式下的EPWM通道的占空比
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **			    Duty  : 16位比较值
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_ConfigChannelSymDuty(uint32_t EPWMn,uint32_t Duty);

/*****************************************************************************
 ** \brief	 EPWM_ConfigChannelAsymDuty	
 **			 设置非对称模式下的 EPWM通道的占空比
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 
 **			    UpCmp 	  : 16位向上比较值
 **			    DownCmp   : 16位向下比较值
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_ConfigChannelAsymDuty(uint32_t EPWMn,uint32_t UpCmp, uint32_t DownCmp);

/*****************************************************************************
 ** \brief	 EPWM_EnableOneShotMode	
 **			 使能单次模式
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_EnableOneShotMode(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_EnableAutoLoadMode	
 **			 使能连续加载模式
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_EnableAutoLoadMode(uint32_t EPWMnMask);

/*****************************************************************************
 ** \brief	 EPWM_EnableReverseOutput	
 **			 开启反相输出
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_EnableReverseOutput(uint32_t EPWMnMask);

/*****************************************************************************
 ** \brief	 EPWM_DisableReverseOutput	
 **			 关闭反相输出
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_DisableReverseOutput(uint32_t EPWMnMask);

/*****************************************************************************
 ** \brief	 EPWM_EnableDeadZone	
 **			 开启死区
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 **				DeadZone : 0x00 ~ 0x3FF;
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_EnableDeadZone(uint32_t EPWMnMask, uint32_t DeadZone);

/*****************************************************************************
 ** \brief	 EPWM_DisableDeadZone	
 **			 关闭死区
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_DisableDeadZone(uint32_t EPWMnMask);

/*****************************************************************************
 ** \brief	 EPWM_Start	
 **			 开启EPWM
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_Start(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_Stop	
 **			 关闭EPWM
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_Stop(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_ForceStop	
 **			 强制关闭EPWM
 ** \param [in]  none
 ** \return none
 ** \note   使用掩码控制EPWM
*****************************************************************************/
void EPWM_ForceStop(void);

/*****************************************************************************
 ** \brief	 EPWM_EnableOutput
 **			 开启EPWM通道输出
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note  
*****************************************************************************/
void EPWM_EnableOutput(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_DisableOutput
 **			 关闭EPWM通道输出
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_DisableOutput(uint32_t EPWMnMask);

/*****************************************************************************
 ** \brief	 EPWM_ConfigLoadAndIntMode
 **			 配置加载方式以及中断方式
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 **				 Mode: (1)EPWM_EACH_PERIOD_ZERO
 **					   (2)EPWM_EACH_ZERO
 **					   (3)EPWM_FIRST_ZERO_NEXT_PERIOD
 **					   (4)EPWM_EVERY_TWO_ZERO
 ** \return none
 ** \note   
*****************************************************************************/
void EPWM_ConfigLoadAndIntMode(uint32_t EPWMnMask, uint32_t Mode);

/*****************************************************************************
 ** \brief	 EPWM_EnableUpCmpInt
 **			 开启向上比较中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_EnableUpCmpInt(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_DisableUpCmpInt
 **			 关闭向上比较中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_DisableUpCmpInt(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_GetUpCmpIntFlag
 **			获取向上比较中断使能并产生中断标志位
 ** \param [in]EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return 1：中断使能并且产生中断  0：无中断
 ** \note  
*****************************************************************************/
uint32_t  EPWM_GetUpCmpIntFlag(uint32_t EPWMn);
/*****************************************************************************
 ** \brief	 EPWM_ClearUpCmpIntFlag
 **			清除向上比较中断标志位
 ** \param [in]EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_ClearUpCmpIntFlag(uint32_t EPWMn);

/*****************************************************************************
 ** \brief	 EPWM_EnableDownCmpInt
 **			 开启向下比较中断
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_EnableDownCmpInt(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_DisableDownCmpInt
 **			 关闭向下比较中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_DisableDownCmpInt(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_GetDownCmpIntFlag
 **			获取向下比较中断使能并产生中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return 1：中断使能并且产生中断  0：无中断
 ** \note  
*****************************************************************************/
uint32_t  EPWM_GetDownCmpIntFlag(uint32_t EPWMn);
/*****************************************************************************
 ** \brief	 EPWM_ClearDownCmpIntFlag
 **			清除向下比较中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_ClearDownCmpIntFlag(uint32_t EPWMn);


/*****************************************************************************
 ** \brief	 EPWM_EnablePeriodInt
 **			 开启周期中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_EnablePeriodInt(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_DisablePeriodInt
 **			 关闭周期中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_DisablePeriodInt(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_GetPeriodIntFlag
 **			获取周期中断使能并产生中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return 1：中断使能并且产生中断  0：无中断
 ** \note  
*****************************************************************************/
uint32_t  EPWM_GetPeriodIntFlag(uint32_t EPWMn);
/*****************************************************************************
 ** \brief	 EPWM_ClearPeriodIntFlag
 **			清除周期中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_ClearPeriodIntFlag(uint32_t EPWMn);

/*****************************************************************************
 ** \brief	 EPWM_EnableZeroInt
 **			 开启零点中断
 ** \param [in]  EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_EnableZeroInt(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_DisableZeroInt
 **			 关闭零点中断
 ** \param [in] EPWMnMask : EPWM_CH_0_MSK 、EPWM_CH_1_MSK 、EPWM_CH_2_MSK 、
 **							 EPWM_CH_3_MSK 、EPWM_CH_4_MSK 、EPWM_CH_5_MSK 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_DisableZeroInt(uint32_t EPWMnMask);
/*****************************************************************************
 ** \brief	 EPWM_GetZeroIntFlag
 **			获取零点中断使能并产生中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return 1：中断使能并且产生中断  0：无中断
 ** \note  
*****************************************************************************/
uint32_t  EPWM_GetZeroIntFlag(uint32_t EPWMn);
/*****************************************************************************
 ** \brief	 EPWM_ClearZeroIntFlag
 **			清除零点中断标志位
 ** \param [in] EPWMn : EPWM0 、EPWM1 、EPWM2 、EPWM3 、EPWM4 、EPWM5 	
 ** \return none
 ** \note  
*****************************************************************************/
void  EPWM_ClearZeroIntFlag(uint32_t EPWMn);

#ifdef __cplusplus
}
#endif

#endif /*__EPWM_H_ */

