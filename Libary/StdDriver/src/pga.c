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
/** \file pga.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "pga.h"
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
 ** \brief	 PGA_ConfigPositive
 **			 设置PGA正端输入
 ** \param [in] pga :PGA0 、PGA1
 **				Positive:	(1)PGA_POSSEL_P0
 **							(2)PGA_POSSEL_P1
 **							(3)PGA_POSSEL_P2
 **							(4)PGA_POSSEL_P3
 ** \return  none	
 ** \note    默认关闭PGA模块
 *****************************************************************************/
void PGA_ConfigPositive(PGA_T *pga, uint32_t Positive)
{
	pga->CON0 &= ~(PGA_CON0_PS_Msk|PGA_CON0_EN_Msk);
	pga->CON0 |= Positive;
}
/*****************************************************************************
 ** \brief	 OPA_ConfigGain
 **			 设置PGA增益
 ** \param [in] pga :PGA0 、PGA1
 **				Gain:  (1)PGA_GAIN_4		
 **					   (2)PGA_GAIN_8 
 **					   (3)PGA_GAIN_10 
 **					   (4)PGA_GAIN_12 
 **					   (5)PGA_GAIN_14 
 **					   (6)PGA_GAIN_16 
 **					   (6)PGA_GAIN_32 
 ** \return  none
 ** \note    
 *****************************************************************************/
void PGA_ConfigGain(PGA_T *pga, uint32_t Gain)
{
	pga->CON0 &= ~(PGA_CON0_GS_Msk);
	pga->CON0 |= Gain;	
}

/*****************************************************************************
 ** \brief	 PGA_ConfigResistance
 **			 设置PGA反馈电阻接地端
 ** \param [in] pga :PGA0 、PGA1
 **				Resistance:	(1)PGA_R_GND
 **							(2)PGA_R_PIN
 ** \return  none	
 ** \note    默认关闭PGA模块
 *****************************************************************************/
void PGA_ConfigResistance(PGA_T *pga, uint32_t Resistance)
{
	pga->CON0 &= ~(PGA_CON0_RGS_Msk|PGA_CON0_EN_Msk);
	pga->CON0 |= Resistance;	
}

/*****************************************************************************
 ** \brief	 PGA_ConfigOutput
 **			 设置PGA输出端
 ** \param [in] pga :PGA0 、PGA1
 **				Output:	(1)PGA_OUTSEL_O
 ** \return  none	
 ** \note    默认关闭PGA模块
 *****************************************************************************/
void PGA_ConfigOutput(PGA_T *pga, uint32_t Output)
{
	pga->CON0 &= ~(PGA_CON0_OS_Msk|PGA_CON0_EN_Msk);
	pga->CON0 |= Output;		
}

/*****************************************************************************
 ** \brief	 PGA_Start
 **			 开启PGA
 ** \param [in] pga :PGA0 、PGA1
 ** \return  none	
 ** \note    
 *****************************************************************************/
void PGA_Start(PGA_T *pga)
{
	pga->CON0 |= PGA_CON0_EN_Msk;		
}

/*****************************************************************************
 ** \brief	 PGA_Stop
 **			 关闭PGA
 ** \param [in] pga :PGA0 、PGA1
 ** \return  none	
 ** \note    
 *****************************************************************************/
void PGA_Stop(PGA_T *pga)
{
	pga->CON0 &= ~(PGA_CON0_EN_Msk);	
}

