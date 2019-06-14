/******************************************************************************

Self Powered Wireless Airflow Sensor Main code for CC2640R2F RGZ package
A heavily modified version of the 'Project Zero' example project provided by TI.
There are still some remanants of the original project zero code which could possibly be removed.

Made at Imperial College London for 2018-2019 academic year MEng Final Year Project entitled 'Self-powered wireless airflow sensor' by
Shaun C Price CID 01078017
email scp15@ic.ac.uk

 
 *****************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/aux_adc.h) //This API required for converting raw ADC values to voltages

#include <ti/drivers/PIN.h>
#include <ti/display/Display.h> //For UART serial comms over PUTTY, this may be removed
#include <ti/drivers/Power.h>

#include <xdc/runtime/Diags.h>
#include <uartlog/UartLog.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#include <icall.h>

#include <osal_snv.h>
#include <peripheral.h>
#include <devinfoservice.h>

#include "util.h"

#include "Board.h"
#include "project_zero.h"

/* Sensor Controller drivers */
#include "spwasservice.h" //Custom Self Powered Wireless Airflow Sensor sensor controller service

#include "scif.h" //Sensor Controller Studio application header

/*********************************************************************
 * CONSTANTS
 */

#define STORAGE_CAPACITANCE 0.01 //Energy Harvesting storage capacitance used in power calculation

// Advertising interval when device is discoverable (units of 625us, 800 = 0.5s). This can be altered to suit power constraints
#define DEFAULT_ADVERTISING_INTERVAL          800

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Default pass-code used for pairing.
#define DEFAULT_PASSCODE                      000000

// Task configuration
#define PRZ_TASK_PRIORITY                     1

#ifndef PRZ_TASK_STACK_SIZE
#define PRZ_TASK_STACK_SIZE                   800
#endif

// Internal Events for RTOS application
#define PRZ_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define PRZ_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define PRZ_STATE_CHANGE_EVT                  Event_Id_00
#define PRZ_CHAR_CHANGE_EVT                   Event_Id_01
#define PRZ_PERIODIC_EVT                      Event_Id_02
#define PRZ_APP_MSG_EVT                       Event_Id_03

#define PRZ_ALL_EVENTS                       (PRZ_ICALL_EVT        | \
                                              PRZ_QUEUE_EVT        | \
                                              PRZ_STATE_CHANGE_EVT | \
                                              PRZ_CHAR_CHANGE_EVT  | \
                                              PRZ_PERIODIC_EVT     | \
                                              PRZ_APP_MSG_EVT)

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(registerCause) (connectionEventRegisterCauseBitMap |= registerCause )

// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(registerCause) (connectionEventRegisterCauseBitMap &= (~registerCause) )

// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)

// Gets whether the registerCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(registerCause) (connectionEventRegisterCauseBitMap & registerCause )

/*********************************************************************
 * TYPEDEFS
 */
// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
typedef enum
{
  APP_MSG_SERVICE_WRITE = 0,   /* A characteristic value has been written     */
  APP_MSG_SERVICE_CFG,         /* A characteristic configuration has changed  */
  APP_MSG_UPDATE_CHARVAL,      /* Request from ourselves to update a value    */
  APP_MSG_GAP_STATE_CHANGE,    /* The GAP / connection state has changed      */
  APP_MSG_BUTTON_DEBOUNCED,    /* A button has been debounced with new value  */
  APP_MSG_SEND_PASSCODE,       /* A pass-code/PIN is requested during pairing */
  APP_MSG_PRZ_CONN_EVT,        /* Connection Event finished report            */

  APP_MSG_SC_CTRL_READY, //Handles sensor controller task alerts
  APP_MSG_SC_TASK_ALERT,
} app_msg_types_t;

// Struct for messages sent to the application task
typedef struct
{
  Queue_Elem       _elem;
  app_msg_types_t  type;
  uint8_t          pdu[];
} app_msg_t;

// Struct for messages about characteristic data
typedef struct
{
  uint16_t svcUUID; // UUID of the service
  uint16_t dataLen; //
  uint8_t  paramID; // Index of the characteristic
  uint8_t  data[];  // Flexible array member, extended to malloc - sizeof(.)
} char_data_t;

// Struct for message about sending/requesting passcode from peer.
typedef struct
{
  uint16_t connHandle;
  uint8_t  uiInputs;
  uint8_t  uiOutputs;
  uint32   numComparison;
} passcode_req_t;

// Struct for message about button state
typedef struct
{
  PIN_Id   pinId;
  uint8_t  state;
} button_state_t;

/*********************************************************************
 * LOCAL VARIABLES
 */

/* variables used in sensor controller process task and are accessed globally */
int32_t adcData[2]; //For transferring ADC buffer from SCS to MCU
float grad; //for storing gradient value as decimal number
float mwPower; //Power in milliwatts
float frequency; //electrical frequency in Hz
uint16_t time_high; //Storage for 16 MSBs
uint16_t time_low; //Storage for 16 LSBs
uint32_t time_tot; //Merge of time_high and time_low
uint32_t rtc_Hz; //set SCS task interval

//For storing ADC parameters used in the ADC API for converting raw value to voltage
int32_t ADCGain;
int32_t ADCOffset;

//storage for value used in power calc
int32_t sub;
int32_t avg;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;

// Task configuration
Task_Struct przTask;
Char przTaskStack[PRZ_TASK_STACK_SIZE];


// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // No scan response data provided.
  0x00 // Placeholder to keep the compiler happy.
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) or general
  // discoverable mode (advertises indefinitely), depending
  // on the DEFAULT_DISCOVERY_MODE define.
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // complete name
  10,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S', 'P', 'W', 'A', 'S', ' ', 'D', 'E', 'V',

};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "SPWAS FYP";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;


/*
 * Initial LED pin configuration table, THIS MAY BE REMOVABLE AS GPIO/LEDS NOT USED
 *   - LEDs Board_LED0 & Board_LED1 are off.
 */
PIN_Config ledPinTable[] = {
  Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};


// State of the buttons


// Global display handle
Display_Handle dispHandle;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void ProjectZero_init( void );
static void ProjectZero_taskFxn(UArg a0, UArg a1);

static void user_processApplicationMessage(app_msg_t *pMsg);
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);

static void ProjectZero_sendAttRsp(void);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);
static void ProjectZero_freeAttRsp(uint8_t status);

static void ProjectZero_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void ProjectZero_processConnEvt(Gap_ConnEventRpt_t *pReport);

static void user_processGapStateChangeEvt(gaprole_States_t newState);
static void user_gapStateChangeCB(gaprole_States_t newState);
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs, uint32 numComparison);
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status);

// Sensor Controller functions
static void scCtrlReadyCallback(void);
static void scTaskAlertCallback(void);
static void processTaskAlert(void);

// Utility functions
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData, uint16_t len );

#if defined(UARTLOG_ENABLE)
static char *Util_getLocalNameStr(const uint8_t *data);
#endif
/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t user_gapRoleCBs =
{
  user_gapStateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t user_bondMgrCBs =
{
  user_gapBondMgr_passcodeCB, // Passcode callback
  user_gapBondMgr_pairStateCB // Pairing / Bonding state Callback
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */

/*********************************************************************
 * The following typedef and global handle the registration to connection event
 */
typedef enum
{
   NONE_REGISTERED    = 0,
   FOR_ATT_RSP        = 1,
} connectionEventRegisterCause_u;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t connectionEventRegisterCauseBitMap = NONE_REGISTERED; // See connectionEventRegisterCause_u


/*
 * @brief  Register to receive connection event reports for all the connections
 *
 * @param  connectionEventRegisterCause  Represents the reason for registration
 *
 * @return @ref SUCCESS
 */
bStatus_t ProjectZero_RegistertToAllConnectionEvent(connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  // In case  there is no registration for the connection event, register for report
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = GAP_RegisterConnEventCb(ProjectZero_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
  }
  
  if(status == SUCCESS)
  {
    // Add the reason bit to the bitamap.
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
}

/*
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t ProjectZero_UnRegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
  
  // In case there are no more subscribers for the connection event then unregister for report
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    GAP_RegisterConnEventCb(ProjectZero_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
  }

  return(status);
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * @brief   Task creation function for the user task.
 *
 * @param   None.
 *
 * @return  None.
 */
void ProjectZero_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = przTaskStack;
  taskParams.stackSize = PRZ_TASK_STACK_SIZE;
  taskParams.priority = PRZ_TASK_PRIORITY;

  Task_construct(&przTask, ProjectZero_taskFxn, &taskParams, NULL);
}

/*
 * @brief   Called before the task loop and contains application-specific
 *          initialization of the BLE stack, hardware setup, power-state
 *          notification if used, and BLE profile/service initialization.
 *
 * @param   None.
 *
 * @return  None.
 */
static void ProjectZero_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages via ICall to Stack.
  ICall_registerApp(&selfEntity, &syncEvent);

  Log_info0("Initializing the user task, hardware, BLE stack and services.");

  // Open display. By default this is disabled via the predefined symbol Display_DISABLE_ALL.
  dispHandle = Display_open(Display_Type_UART, NULL);

  // Initialize queue for application messages.
  // Note: Used to transfer control to application thread from e.g. interrupts.
  Queue_construct(&applicationMsgQ, NULL);
  hApplicationMsgQ = Queue_handle(&applicationMsgQ);


  // ******************************************************************
  // Hardware initialization
  // ******************************************************************
     //Get ADC parameters
     ADCGain = AUXADCGetAdjustmentGain (AUXADC_REF_FIXED);
     ADCOffset = AUXADCGetAdjustmentOffset   (AUXADC_REF_FIXED);

     //Invoke low power policy
     Power_init();
     Power_enablePolicy();
     //Initialise incoming ADC data array
     adcData[0] = 0;
     adcData[1] = 0;

  // ******************************************************************
  // BLE Stack initialization
  // ******************************************************************

  // Setup the GAP Peripheral Role Profile
  uint8_t initialAdvertEnable = TRUE;  // Advertise on power-up

  // By setting this to zero, the device will go into the waiting state after
  // being discoverable. Otherwise wait this long [ms] before advertising again.
  uint16_t advertOffTime = 0; // miliseconds

  // Set advertisement enabled.
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                       &initialAdvertEnable);

  // Configure the wait-time before restarting advertisement automatically
  GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                       &advertOffTime);

  // Initialize Scan Response data
  GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);

  // Initialize Advertisement data
  GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

  //********* CHANGING CONNECTION INTERVAL *********
  // Whether to enable automatic parameter update request when a connection is
  // formed (APP INVENTOR IGNORES THESE PARAMETERS....)
  uint8_t enableUpdateRequest = GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS;

  uint16_t desiredMinInterval = 400;
  uint16_t desiredMaxInterval = 600;
  uint16_t desiredSlaveLatency = 0;
  uint16_t desiredConnTimeout = 1000;

  GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                       &enableUpdateRequest);

  GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                       &desiredMinInterval);
  GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                       &desiredMaxInterval);
  GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                       &desiredSlaveLatency);
  GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                       &desiredConnTimeout);

// ******************************************

  Log_info1("Name in advertData array: \x1b[33m%s\x1b[0m",
            (IArg)Util_getLocalNameStr(advertData));

  // Set advertising interval
  uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

  // Set duration of advertisement before stopping in Limited adv mode.
  GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 30); // Seconds

  // ******************************************************************
  // BLE Bond Manager initialization
  // ******************************************************************
  uint32_t passkey = 0; // passkey "000000"
  uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
  uint8_t mitm = TRUE;
  uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
  uint8_t bonding = TRUE;

  GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                          &passkey);
  GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
  GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
  GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
  GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

  // ******************************************************************
  // BLE Service initialization
  // ******************************************************************

  // Add services to GATT server
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

  // Set the device name characteristic in the GAP Profile
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Add services to GATT server and give ID of this task for Indication acks.
   Spwasservice_AddService( selfEntity );


  // Initalization of characteristics in spwasService that are readable.
  uint8_t spwasservice_freq_initVal[SPWASSERVICE_FREQ_LEN] = {0};
  Spwasservice_SetParameter(SPWASSERVICE_FREQ_ID, SPWASSERVICE_FREQ_LEN, spwasservice_freq_initVal);

  uint8_t spwasservice_netPwr_initVal[SPWASSERVICE_NETPWR_LEN] = {0};
  Spwasservice_SetParameter(SPWASSERVICE_NETPWR_ID, SPWASSERVICE_NETPWR_LEN, spwasservice_netPwr_initVal);

  uint8_t spwasservice_Vcap_initVal[SPWASSERVICE_VCAP_LEN] = {0};
  Spwasservice_SetParameter(SPWASSERVICE_VCAP_ID, SPWASSERVICE_VCAP_LEN, spwasservice_Vcap_initVal);


  // Start the stack in Peripheral mode.
  VOID GAPRole_StartDevice(&user_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&user_bondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
}


/*
 * @brief   Application task entry point.
 *
 *          Invoked by TI-RTOS when BIOS_start is called. Calls an init function
 *          and enters an infinite loop waiting for messages.
 *
 *          Messages can be either directly from the BLE stack or from user code
 *          like Hardware Interrupt (Hwi) or a callback function.
 *
 *          The reason for sending messages to this task from e.g. Hwi's is that
 *          some RTOS and Stack APIs are not available in callbacks and so the
 *          actions that may need to be taken is dispatched to this Task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void ProjectZero_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  ProjectZero_init();

  // Initialize the Sensor Controller
     scifOsalInit();
     scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
     scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
     scifInit(&scifDriverSetup);

     // Set the Sensor Controller task tick interval to 1 second
        rtc_Hz = 1;  // 1Hz RTC
        scifStartRtcTicksNow(0x00010000 / rtc_Hz);

        // Start Sensor Controller task
           scifStartTasksNbl(BV(SCIF_COMP_HANDLE_TASK_ID));


  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, PRZ_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Check if we got a signal because of a stack message
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = ProjectZero_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // Process messages sent from another task or another context.
      while (!Queue_empty(hApplicationMsgQ))
      {
        app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);

        // Process application-layer message probably sent from ourselves.
        user_processApplicationMessage(pMsg);

        // Free the received message.
        ICall_free(pMsg);
      }
    }
  }
}


/*
 * @brief   Handle application messages
 *
 *          These are messages not from the BLE stack, but from the
 *          application itself.
 *
 *          For example, in a Software Interrupt (Swi) it is not possible to
 *          call any BLE APIs, so instead the Swi function must send a message
 *          to the application Task for processing in Task context.
 *
 * @param   pMsg  Pointer to the message of type app_msg_t.
 *
 * @return  None.
 */
static void user_processApplicationMessage(app_msg_t *pMsg)
{
  char_data_t *pCharData = (char_data_t *)pMsg->pdu;

  switch (pMsg->type)
  {
    case APP_MSG_SERVICE_WRITE: /* Message about received value write */
        //Do nothing
        break;
    case APP_MSG_SC_TASK_ALERT:
                      processTaskAlert();
          break;



    case APP_MSG_SERVICE_CFG: /* Message about received CCCD write */
      /* Call different handler per service */
      switch(pCharData->svcUUID) {
      //Do Nothing
      }
      break;

   case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
     AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
     break;

    case APP_MSG_UPDATE_CHARVAL: /* Message from ourselves to send  */
      //user_updateCharVal(pCharData);
      break;

    case APP_MSG_GAP_STATE_CHANGE: /* Message that GAP state changed  */
      user_processGapStateChangeEvt( *(gaprole_States_t *)pMsg->pdu );
      break;

    case APP_MSG_SEND_PASSCODE: /* Message about pairing PIN request */
      {
        passcode_req_t *pReq = (passcode_req_t *)pMsg->pdu;
        Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
                  (IArg)(pReq->uiInputs?"Sending":"Displaying"),
                  DEFAULT_PASSCODE);
        // Send passcode response.
        GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, DEFAULT_PASSCODE);
      }
      break;

    case APP_MSG_BUTTON_DEBOUNCED: /* Message from swi about pin change */
      {
      }
      break;

    case APP_MSG_PRZ_CONN_EVT:
    {
        ProjectZero_processConnEvt((Gap_ConnEventRpt_t *)pMsg->pdu);
        break;
    }
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of system/application events deferred to the user Task context.
 *  Invoked from the application Task function above.
 *
 *  Further down you can find the callback handler section containing the
 *  functions that defer their actions via messages to the application task.
 *
 ****************************************************************************
 *****************************************************************************/


/*
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void user_processGapStateChangeEvt(gaprole_States_t newState)
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        char *cstr_ownAddress = Util_convertBdAddr2Str(ownAddress);
        Log_info1("GAP is started. Our address: \x1b[32m%s\x1b[0m", (IArg)cstr_ownAddress);
      }
      break;

    case GAPROLE_ADVERTISING:
      Log_info0("Advertising");
      break;

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        char *cstr_peerAddress = Util_convertBdAddr2Str(peerAddress);
        Log_info1("Connected. Peer address: \x1b[32m%s\x1b[0m", (IArg)cstr_peerAddress);

        GAPRole_SendUpdateParam(499, 501, 0, 1000, GAPROLE_NO_ACTION); //Force connection interval to change
       }
      break;

    case GAPROLE_CONNECTED_ADV:
      Log_info0("Connected and advertising");
      break;

    case GAPROLE_WAITING:
      Log_info0("Disconnected / Idle");
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      Log_info0("Connection timed out");
      break;

    case GAPROLE_ERROR:
      Log_info0("Error");
      break;

    default:
      break;
  }
}




/*
 * @brief   Process an incoming BLE stack message.
 *
 *          This could be a GATT message from a peer device like acknowledgement
 *          of an Indication we sent, or it could be a response from the stack
 *          to an HCI message that the user application sent.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = ProjectZero_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            Log_info0("HCI Command Complete Event received");
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}


/*
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    Log_warning1("Outgoing RF FIFO full. Re-schedule transmission of msg with opcode 0x%02x",
      pMsg->method);

    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if(ProjectZero_RegistertToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS)
    {
      // First free any pending response
      ProjectZero_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Log the opcode of the message that caused the violation.
    Log_error1("Flow control violated. Opcode of offending ATT msg: 0x%02x",
      pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Log_info1("MTU Size change: %d bytes", pMsg->msg.mtuEvt.MTU);
  }
  else
  {
    // Got an expected GATT message from a peer.
    Log_info1("Recevied GATT Message. Opcode: 0x%02x", pMsg->method);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      ProjectZero_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void ProjectZero_processConnEvt(Gap_ConnEventRpt_t *pReport)
{

  if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
  {
    // The GATT server might have returned a blePending as it was trying
    // to process an ATT Response. Now that we finished with this
    // connection event, let's try sending any remaining ATT Responses
    // on the next connection event.
    // Try to retransmit pending ATT Response (if any)

    ProjectZero_sendAttRsp();
  }

}

/*
 *  Application error handling functions
 *****************************************************************************/

/*
 * @brief   Send a pending ATT response message.
 *
 *          The message is one that the stack was trying to send based on a
 *          peer request, but the response couldn't be sent because the
 *          user application had filled the TX queue with other data.
 *
 * @param   none
 *
 * @return  none
 */
static void ProjectZero_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      ProjectZero_UnRegistertToAllConnectionEvent (FOR_ATT_RSP);

      // We're done with the response message
      ProjectZero_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Log_warning2("Retrying message with opcode 0x%02x. Attempt %d",
        pAttRsp->method, rspTxRetry);
    }
  }
}

/*
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void ProjectZero_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Log_info2("Sent message with opcode 0x%02x. Attempt %d",
        pAttRsp->method, rspTxRetry);
    }
    else
    {
      Log_error2("Gave up message with opcode 0x%02x. Status: %d",
        pAttRsp->method, status);

      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of direct system callbacks.
 *
 *  Typically enqueue the information or request as a message for the
 *  application Task for handling.
 *
 ****************************************************************************
 *****************************************************************************/

/*********************************************************************
 * @fn      ProjectZero_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void ProjectZero_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  user_enqueueRawAppMsg(APP_MSG_PRZ_CONN_EVT, (uint8_t *)pReport, sizeof(pReport));
  ICall_free(pReport);
}

/*
 *  Callbacks from the Stack Task context (GAP or Service changes)
 *****************************************************************************/

/**
 * Callback from GAP Role indicating a role state change.
 */
static void user_gapStateChangeCB(gaprole_States_t newState)
{
  Log_info1("(CB) GAP State change: %d, Sending msg to app.", (IArg)newState);
  user_enqueueRawAppMsg( APP_MSG_GAP_STATE_CHANGE, (uint8_t *)&newState, sizeof(newState) );
}

/*
 * @brief   Passcode callback.
 *
 * @param   connHandle - connection handle
 * @param   uiInputs   - input passcode?
 * @param   uiOutputs  - display passcode?
 * @param   numComparison - numeric comparison value
 *
 * @return  none
 */
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs, uint32 numComparison)
{
  passcode_req_t req =
  {
    .connHandle = connHandle,
    .uiInputs = uiInputs,
    .uiOutputs = uiOutputs,
    .numComparison = numComparison
  };

  // Defer handling of the passcode request to the application, in case
  // user input is required, and because a BLE API must be used from Task.
  user_enqueueRawAppMsg(APP_MSG_SEND_PASSCODE, (uint8_t *)&req, sizeof(req));
}

/*
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 * @param   state      - pairing state
 * @param   status     - pairing status
 *
 * @return  none
 */
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Log_info0("Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Log_info0("Pairing completed successfully.");
    }
    else
    {
      Log_error1("Pairing failed. Error: %02x", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
     Log_info0("Re-established pairing from stored bond info.");
    }
  }
}



/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

/*
 * @brief  Generic message constructor for application messages.
 *
 *         Sends a message to the application for handling in Task context.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData,
                                  uint16_t len)
{
  // Allocate memory for the message.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + len );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    // Copy data into message
    memcpy(pMsg->pdu, pData, len);

    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
//    // Let application know there's a message.
    Event_post(syncEvent, PRZ_APP_MSG_EVT);
  }
}




/*
 * @brief   Convert {0x01, 0x02} to "01:02"
 *
 * @param   src - source byte-array
 * @param   src_len - length of array
 * @param   dst - destination string-array
 * @param   dst_len - length of array
 *
 * @return  array as string
 */


#if defined(UARTLOG_ENABLE)
/*
 * @brief   Extract the LOCALNAME from Scan/AdvData
 *
 * @param   data - Pointer to the advertisement or scan response data
 *
 * @return  Pointer to null-terminated string with the adv local name.
 */
static char *Util_getLocalNameStr(const uint8_t *data) {
  uint8_t nuggetLen = 0;
  uint8_t nuggetType = 0;
  uint8_t advIdx = 0;

  static char localNameStr[32] = { 0 };
  memset(localNameStr, 0, sizeof(localNameStr));

  for (advIdx = 0; advIdx < 32;) {
    nuggetLen = data[advIdx++];
    nuggetType = data[advIdx];
    if ( (nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
          nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) && nuggetLen < 31) {
      memcpy(localNameStr, &data[advIdx + 1], nuggetLen - 1);
      break;
    } else {
      advIdx += nuggetLen;
    }
  }

  return localNameStr;
}
#endif

/*********************************************************************
*********************************************************************/

static void scCtrlReadyCallback(void)
{
  // Notify application `Control READY` is active
  user_enqueueRawAppMsg(APP_MSG_SC_CTRL_READY, NULL, 0);
} // scCtrlReadyCallback

static void scTaskAlertCallback(void)
{
  // Notify application `Task ALERT` is active
  user_enqueueRawAppMsg(APP_MSG_SC_TASK_ALERT, NULL, 0);
} // scTaskAlertCallback

/* The following is the process task which is executed when a task alert is recieved from the Sensor controller */
void processTaskAlert(void){

       scifClearAlertIntSource(); //Clear the sensor controller task alert interrupt

       adcData[1] = adcData[0]; //Shift previous value
       adcData[0] = scifTaskData.compHandle.output.ADCout; //Insert new value

       //Adjust for gain and offset errors and convert to uV
       adcData[0] = AUXADCAdjustValueForGainAndOffset((int32_t) adcData[0], ADCGain, ADCOffset);
       adcData[0] = AUXADCValueToMicrovolts(AUXADC_FIXED_REF_VOLTAGE_NORMAL, (int32_t) adcData[0]);

       sub = adcData[0] - adcData[1]; // signed difference in uV
       avg = adcData[0] + adcData[1]; // signed sum in uV
       grad = sub * 0.000001 * rtc_Hz; //grad = difference * rtc_Hz = difference/rtc_period = gradient in V/s

       float vstorvolt = adcData[0] * 0.000001; //obtain storage cap voltage to send over ble

       mwPower = grad * STORAGE_CAPACITANCE * (avg * 0.000001) / 2 * 1000; //avg power in mW (gradient of voltage slope * capacitance = average current. avg current*avg voltage = avg power in W *1000 = avg power in mw.

       Spwasservice_SetParameter(SPWASSERVICE_NETPWR_ID, SPWASSERVICE_NETPWR_LEN, &mwPower); //send power
       Spwasservice_SetParameter(SPWASSERVICE_VCAP_ID, SPWASSERVICE_VCAP_LEN, &vstorvolt); //send voltage

       //Do processing
       time_high = scifTaskData.compHandle.output.TimeOutHigh;
       time_low = scifTaskData.compHandle.output.TimeOutLow;

       time_tot = (time_high << 16) + time_low; //Merge high and low words

    //Convert to frequency
    frequency = time_tot * 2; //Time_tot is time for half a cycle, see sensor controller code
    frequency = 48000000 / frequency; //48MHz is the specified time/digital converter counter frequency
    if(frequency < 35){
        frequency = 0;
        Spwasservice_SetParameter(SPWASSERVICE_FREQ_ID, SPWASSERVICE_FREQ_LEN, &frequency); //send frequency
    } else if(frequency < 1200){
        Spwasservice_SetParameter(SPWASSERVICE_FREQ_ID, SPWASSERVICE_FREQ_LEN, &frequency); //send frequency
    }


    scifAckAlertEvents();
}
