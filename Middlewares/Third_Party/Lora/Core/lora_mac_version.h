 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    lora_mac_version.h
  * @author  MCD Application Team
  * @brief   defines the lora mac version
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __LORA_MAC_VERSION_H__
#define __LORA_MAC_VERSION_H__

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/*the 3 MSBs define the version based on Github version         */
/*https://github.com/Lora-net/LoRaMac-node/wiki/LoRaMAC-node-Wiki*/
/* version 4.4.0 from develop branch */
#define LORA_MAC_VERSION   (uint32_t) 0x44260000
   
/* Exported types ------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

#ifdef __cplusplus
}
#endif

#endif /*__LORA_MAC_VERSION_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
