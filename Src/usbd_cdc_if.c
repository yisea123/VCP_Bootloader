/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include "stdlib.h"
#include "globals.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  1000
#define APP_TX_DATA_SIZE  1000
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

extern void *_app_start[];

const unsigned char ACK = 0x79;
const unsigned char NACK = 0x1F;
const unsigned char GetResponse[] = {0x79, 11, 0x31, 0x00, 0x01, 0x02, 0x11, 0x21, 0x31, 0x43, 0x63, 0x73, 0x82, 0x92, 0x79};
const unsigned char GVResponse[] = {0x79, 0x31, 0x00, 0x00, 0x79};
const unsigned char GIDResponse[] = {0x79, 1, 0x04, 0x10, 0x79};

unsigned short _state = 0;
unsigned int _address;
unsigned char _len;
unsigned char _buffer[257] = {0x79};
unsigned short _bufferIndex = 0;

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);

  if(*Len == 1 && Buf[0] == 0x7F && _state != 0x4300 && _state != 0x3101 && _state != 0x3102)
  {
    CDC_Transmit_FS(&ACK, 1);
    _state = 0;
  }
  else
  {
    switch(_state)
    {
      case 0: //Get Command
      {
        unsigned char BufNot0 = ~(Buf[0]);
        unsigned char Buf1 = Buf[1];
        unsigned char res = BufNot0 ^ Buf1;
        if(*Len != 2 || res)
        {
          CDC_Transmit_FS(&NACK, 1);
        }
        else
        {
          switch(Buf[0])
          {
            case 0x00: //Get
              CDC_Transmit_FS(&GetResponse, 15);
              break;
            case 0x01: //GV
              CDC_Transmit_FS(&GVResponse, 5);
              break;
            case 0x02: //GID
              CDC_Transmit_FS(&GIDResponse, 5);
              break;
            case 0x11: //RM
              CDC_Transmit_FS(&ACK, 1);
              _state = 0x1100;
              break;
            case 0x21: //Go
              CDC_Transmit_FS(&ACK, 1);
              _state = 0x2100;
              break;
            case 0x31: //WM
              CDC_Transmit_FS(&ACK, 1);
              _state = 0x3100;
              break;
            case 0x43: //ER
              CDC_Transmit_FS(&ACK, 1);
              _state = 0x4300;
              break;
            case 0x63: //WP
              CDC_Transmit_FS(&ACK, 1);
              _state = 0x6300;
              break;
            case 0x73: //WPUN
              CDC_Transmit_FS(&ACK, 1);
              _state = 0x7300;
              break;
            case 0x82: //RDP_PRM
              //protect readout
              CDC_Transmit_FS(&ACK, 1);
              _state++;
              break;
            case 0x92: //RDU_PRM
            {
              FLASH_EraseInitTypeDef EraseInitStruct;
              HAL_FLASH_Unlock();
              unsigned char success = 1;
              for(int page = ((unsigned int)_app_start - 0x08000000) / 0x400; page<128; page++)
              {
                unsigned int PageError = 0;
                EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
                EraseInitStruct.PageAddress = page * 0x400 + 0x08000000;
                EraseInitStruct.NbPages = 1;
                if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
                  success = 0;
                }
              }
              HAL_FLASH_Lock();
              if(success)
              {
                //unprotect readout
                CDC_Transmit_FS(&ACK, 1);
                _state = 0x9200;
              }
              else
                CDC_Transmit_FS(&NACK, 1);
              break;
            }
            default:
              CDC_Transmit_FS(&NACK, 1);
              break;
          }
        }
        break;
      }
      case 0x1100:
      case 0x2100:
      case 0x3100:
      {
        unsigned char res =  Buf[0] ^ Buf[1] ^ Buf[2] ^ Buf[3] ^ Buf[4];
        if(*Len != 5 || res)
        {
          CDC_Transmit_FS(&NACK, 1);
          _state = 0;
        }
        else 
        {
          _address = Buf[0] << 24 | Buf[1] << 16 | Buf[2] << 8 | Buf[3];
          _state++;
          CDC_Transmit_FS(&ACK, 1);
        }
        break;
      }
      case 0x1101:
      {
        unsigned char res =  ~Buf[0] ^ Buf[1];
        if(*Len != 2 || res)
        {
          CDC_Transmit_FS(&NACK, 1);
        }
        _len = Buf[0] + 1;
        _state = 0;

        memcpy(&_buffer[1], (void *)_address, _len);

        CDC_Transmit_FS(_buffer, _len + 1);
        break;
      }
      case 0x3101:
        _len = Buf[0] + 1;
        _bufferIndex = 0;
        _state++;
        break;
      case 0x3102:
      {
        if(_bufferIndex < _len + 1)
        {
          memcpy(&_buffer[_bufferIndex], Buf, *Len);
          _bufferIndex += *Len;
        }
        
        if(_bufferIndex >= _len + 1)
        {
          unsigned char res = _len-1;
          for(int i = 0; i<_len + 1; i++)
          {
            res ^= _buffer[i];
          }
          
          if(res || _address < (unsigned int)_app_start)
          {
            CDC_Transmit_FS(&NACK, 1);
          }
          else
          {
            HAL_FLASH_Unlock();
            int i = 0;
            while(i<_len)
            {
              if(_len-i >= 4)
              {
                HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (void *)(_address + i), *((unsigned int *)(&_buffer[i])));
                i+=4;
              }
              else
              {
                HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (void *)(_address + i), *((unsigned short *)(&_buffer[i])));
                i+=2;
              }
            }
            HAL_FLASH_Lock();

            CDC_Transmit_FS(&ACK, 1);
          }
          _state = 0;
        }
        break;
      }
      case 0x4300:
      {
        _len = Buf[0];
        unsigned char res =  Buf[0];
        unsigned char _hasPageInBootloader = 0;
        for(int i = 1; i<*Len; i++)
        {
          res ^=  Buf[i];
          if(i < *Len - 1 && Buf[i] < ((unsigned int)_app_start - 0x08000000) / 0x400)
          {
            _hasPageInBootloader = 1;
          }
        }
        if(*Len < 2 || (_len != 0xFF && (res || _hasPageInBootloader)) || (_len == 0xFF && Buf[1] != 0))
        {
          CDC_Transmit_FS(&NACK, 1);
        }
        else if(_len == 0xFF)
        {
          FLASH_EraseInitTypeDef EraseInitStruct;
          HAL_FLASH_Unlock();
          for(int page = ((unsigned int)_app_start - 0x08000000) / 0x400; page<128; page++)
          {
            unsigned int PageError = 0;
            EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
            EraseInitStruct.PageAddress = page * 0x400 + 0x08000000;
            EraseInitStruct.NbPages = 1;
            if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
              CDC_Transmit_FS(&NACK, 1);
            }
          }
          HAL_FLASH_Lock();
          CDC_Transmit_FS(&ACK, 1);
        }
        else
        {
          for(int i = 0; i<_len+1; i++)
          {
            HAL_FLASH_Unlock();
            unsigned int PageError = 0;
            FLASH_EraseInitTypeDef EraseInitStruct;
            EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
            EraseInitStruct.PageAddress = (Buf[i + 1]) * 0x400 + 0x08000000;
            EraseInitStruct.NbPages = 1;
            if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
              CDC_Transmit_FS(&NACK, 1);
            }
            HAL_FLASH_Lock();
          }
          CDC_Transmit_FS(&ACK, 1);
        }
        _state = 0;
        break;
      }
      case 0x6300:
        //write protect
        CDC_Transmit_FS(&ACK, 1);
        _state++;
        break;
    }
  }
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
