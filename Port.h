 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Omar Mohamed
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

#include "Common_Macros.h"
#include "Port.Cfg.h"

/* Id for the company in the AUTOSAR
 * for example Mohamed Tarek's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* Port Module Id */
#define PORT_MODULE_ID    (130U)

/* Port Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and GPIO Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "Port.Cfg.h"
/* Software Version checking between Dio_Cfg.h and Dio.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != DIO_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Dio_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/* service id for port_init */
#define Port_Init_SID                    (uint8)0X00

/* service id for Port_SetPinDirection */
#define Port_SetPinDirection_SID         (uint8)0x01

/* service id Port_RefreshPortDirection */
#define Port_RefreshPortDirection_SID    (uint8)0X02

 /* service id for Port_GetVersionInfo */
#define Port_GetVersionInfo_SID         (uint8)0x03

/* service id for Port_SetPinMode */
#define Port_SetPinMode_SID              (uint8)0x04



/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/

/* Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN                  (uint8)0x0A

/* Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE             (uint8)0x0B

/* Port_Init service called with wrong parameter  */
#define PORT_E_PARAM_CONFIG               (uint8)0x0C

/* incorrect Port Pin mode passed invalide*/
#define PORT_E_PARAM_INVALID_MODE         (uint8)0x0D

/*port_SetPinMode service called when the mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE          (uint8)0x0E

/* port is called without initialization*/
#define PORT_E_UNINIT                     (uint8)0x0F

/* api called with a null pointer parameter*/
#define PORT_E_PARAM_POINTER              (uint8)0x10


/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Data type for the symbolic name of a port pin */
typedef uint8 Port_PinType;

/*Data type for Different port pin modes  */
typedef uint8 Port_PinModeType;

/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Description: Enum to hold the state of the mode if it's changeable or not */
typedef enum{
    MODE_UNCHANGEABLE,MODE_CHANGEABLE
}Port_modechangeable;

/* Description: Enum to hold the state of the direction if it's changeable or not */
typedef enum{
   DIRECTION_UNCHANGEABLE,DIRECTION_CHANGEABLE
}Port_directionchangeable;


/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *  3. the direction of pin --> INPUT or OUTPUT
 *  4. the internal resistor --> Disable, Pull up or Pull down
 *  5. the mode of the pin
 *  6. check if the port mode is changeable during runtime
 *  7. check if the port direction is changeable during runtime
 */
typedef struct 
{
    uint8 port_num; 
    Port_PinType pin_num;
    Port_PinDirectionType direction;
    Port_InternalResistor resistor;
    uint8 initial_value;
    Port_PinModeType mode;
    Port_modechangeable mode_changeability;
    Port_directionchangeable direction_changeability;
}Port_ConfigParameter;

/* Data Structure required for initializing the Port Driver */
typedef struct{

    Port_ConfigParameter Pins[PORT_PINS_NUMBER];

}Port_ConfigType;


/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: non reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Setup the internal resistor for i/p pin
*              - Setup the mode of the pins
************************************************************************************/
void Port_Init(
        const Port_ConfigType * ConfigPtr
        );


/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Port Pin ID number && Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
*
************************************************************************************/
#if (PORT_SETPIN_DIRECTION_API==STD_ON)
void Port_SetPinDirection(
 Port_PinType Pin,
 Port_PinDirectionType Direction
) ;
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Sync/Async: Synchronous
* Reentrancy: non reentrant
* Service ID[hex]: 0x02
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction
*
************************************************************************************/
void Port_RefreshPortDirection(
 void
);



/************************************************************************************
* Service Name: Port_GetVersionInfo
* Sync/Async: Synchronous
* Reentrancy: non reentrant
* Service ID[hex]: 0x03
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Returns the version information of this module
*
************************************************************************************/

#if(PORT_VERSION_INFO_API==STD_ON)
void Port_GetVersionInfo(
        Std_VersionInfoType* versioninfo
        );

#endif


/************************************************************************************
* Service Name: Port_SetPinMode
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Service ID[hex]: 0x04
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode
*
************************************************************************************/
void Port_SetPinMode(
 Port_PinType Pin,
 Port_PinModeType Mode
);

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/
extern const Port_ConfigType Port_Configuration;


#endif /* PORT_H */
