/**********************************************************************************************
 * Filename:       spwasservice.c
 *
 * Description:    This file contains the implementation of the service.
 *
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <icall.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "spwasservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// spwasservice Service UUID
CONST uint8_t spwasserviceUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SPWASSERVICE_SERV_UUID)
};

// freq UUID
CONST uint8_t spwasservice_FreqUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SPWASSERVICE_FREQ_UUID)
};
// netPwr UUID
CONST uint8_t spwasservice_NetPwrUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SPWASSERVICE_NETPWR_UUID)
};
// Vcap UUID
CONST uint8_t spwasservice_VcapUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SPWASSERVICE_VCAP_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static spwasserviceCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t spwasserviceDecl = { ATT_UUID_SIZE, spwasserviceUUID };

// Characteristic "Freq" Properties (for declaration)
static uint8_t spwasservice_FreqProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "Freq" Value variable
static uint8_t spwasservice_FreqVal[SPWASSERVICE_FREQ_LEN] = {0};

// Characteristic "Freq" CCCD
static gattCharCfg_t *spwasservice_FreqConfig;
// Characteristic "NetPwr" Properties (for declaration)
static uint8_t spwasservice_NetPwrProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "NetPwr" Value variable
static uint8_t spwasservice_NetPwrVal[SPWASSERVICE_NETPWR_LEN] = {0};

// Characteristic "NetPwr" CCCD
static gattCharCfg_t *spwasservice_NetPwrConfig;
// Characteristic "Vcap" Properties (for declaration)
static uint8_t spwasservice_VcapProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "Vcap" Value variable
static uint8_t spwasservice_VcapVal[SPWASSERVICE_VCAP_LEN] = {0};

// Characteristic "Vcap" CCCD
static gattCharCfg_t *spwasservice_VcapConfig;

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t spwasserviceAttrTbl[] =
{
  // spwasservice Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&spwasserviceDecl
  },
    // Freq Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &spwasservice_FreqProps
    },
      // Freq Characteristic Value
      {
        { ATT_UUID_SIZE, spwasservice_FreqUUID },
        GATT_PERMIT_READ,
        0,
        spwasservice_FreqVal
      },
      // Freq CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&spwasservice_FreqConfig
      },
    // NetPwr Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &spwasservice_NetPwrProps
    },
      // NetPwr Characteristic Value
      {
        { ATT_UUID_SIZE, spwasservice_NetPwrUUID },
        GATT_PERMIT_READ,
        0,
        spwasservice_NetPwrVal
      },
      // NetPwr CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&spwasservice_NetPwrConfig
      },
    // Vcap Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &spwasservice_VcapProps
    },
      // Vcap Characteristic Value
      {
        { ATT_UUID_SIZE, spwasservice_VcapUUID },
        GATT_PERMIT_READ,
        0,
        spwasservice_VcapVal
      },
      // Vcap CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&spwasservice_VcapConfig
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t spwasservice_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t spwasservice_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t spwasserviceCBs =
{
  spwasservice_ReadAttrCB,  // Read callback function pointer
  spwasservice_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * Spwasservice_AddService- Initializes the Spwasservice service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t Spwasservice_AddService( uint8_t rspTaskId )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  spwasservice_FreqConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( spwasservice_FreqConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, spwasservice_FreqConfig );
  // Allocate Client Characteristic Configuration table
  spwasservice_NetPwrConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( spwasservice_NetPwrConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, spwasservice_NetPwrConfig );
  // Allocate Client Characteristic Configuration table
  spwasservice_VcapConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( spwasservice_VcapConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, spwasservice_VcapConfig );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( spwasserviceAttrTbl,
                                        GATT_NUM_ATTRS( spwasserviceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &spwasserviceCBs );

  return ( status );
}

/*
 * Spwasservice_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t Spwasservice_RegisterAppCBs( spwasserviceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*
 * Spwasservice_SetParameter - Set a Spwasservice parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Spwasservice_SetParameter( uint8_t param, uint16_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SPWASSERVICE_FREQ_ID:
      if ( len == SPWASSERVICE_FREQ_LEN )
      {
        memcpy(spwasservice_FreqVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( spwasservice_FreqConfig, (uint8_t *)&spwasservice_FreqVal, FALSE,
                                    spwasserviceAttrTbl, GATT_NUM_ATTRS( spwasserviceAttrTbl ),
                                    INVALID_TASK_ID,  spwasservice_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SPWASSERVICE_NETPWR_ID:
      if ( len == SPWASSERVICE_NETPWR_LEN )
      {
        memcpy(spwasservice_NetPwrVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( spwasservice_NetPwrConfig, (uint8_t *)&spwasservice_NetPwrVal, FALSE,
                                    spwasserviceAttrTbl, GATT_NUM_ATTRS( spwasserviceAttrTbl ),
                                    INVALID_TASK_ID,  spwasservice_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SPWASSERVICE_VCAP_ID:
      if ( len == SPWASSERVICE_VCAP_LEN )
      {
        memcpy(spwasservice_VcapVal, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( spwasservice_VcapConfig, (uint8_t *)&spwasservice_VcapVal, FALSE,
                                    spwasserviceAttrTbl, GATT_NUM_ATTRS( spwasserviceAttrTbl ),
                                    INVALID_TASK_ID,  spwasservice_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * Spwasservice_GetParameter - Get a Spwasservice parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t Spwasservice_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          spwasservice_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t spwasservice_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the Freq Characteristic Value
if ( ! memcmp(pAttr->type.uuid, spwasservice_FreqUUID, pAttr->type.len) )
  {
    if ( offset > SPWASSERVICE_FREQ_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, SPWASSERVICE_FREQ_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the NetPwr Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, spwasservice_NetPwrUUID, pAttr->type.len) )
  {
    if ( offset > SPWASSERVICE_NETPWR_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, SPWASSERVICE_NETPWR_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the Vcap Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, spwasservice_VcapUUID, pAttr->type.len) )
  {
    if ( offset > SPWASSERVICE_VCAP_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, SPWASSERVICE_VCAP_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      spwasservice_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t spwasservice_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb(connHandle, paramID, len, pValue); // Call app function from stack task context.

  return status;
}
