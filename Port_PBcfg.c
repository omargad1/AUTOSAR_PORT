 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_PBcfg.h
 *
 * Description: source file for TM4C123GH6PM Microcontroller - post build configuration for port driver
 *
 * Author: Omar Mohamed
 ******************************************************************************/

#include "Port.h"

/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between Dio_PBcfg.c and Dio.h files */
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PBcfg.c does not match the expected version"
#endif

/* Software Version checking between Dio_PBcfg.c and Dio.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PBcfg.c does not match the expected version"
#endif


/* PB structure used with Port_Init API */
const Port_ConfigType Port_Configuration={

                     /**************** PORTA PINS **************/
     PORTA_ID,PIN0_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 0 */
     PORTA_ID,PIN1_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 1 */
     PORTA_ID,PIN2_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 2 */
     PORTA_ID,PIN3_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 3 */
     PORTA_ID,PIN4_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 4 */
     PORTA_ID,PIN5_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 5 */
     PORTA_ID,PIN6_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 6 */
     PORTA_ID,PIN7_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 7 */

                     /**************** PORTB PINS **************/
     PORTB_ID,PIN0_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 0 */
     PORTB_ID,PIN1_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 1 */
     PORTB_ID,PIN2_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 2 */
     PORTB_ID,PIN3_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 3 */
     PORTB_ID,PIN4_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 4 */
     PORTB_ID,PIN5_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 5 */
     PORTB_ID,PIN6_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 6 */
     PORTB_ID,PIN7_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 7 */

                     /**************** PORTC PINS **************/
     PORTC_ID,PIN0_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 0 */
     PORTC_ID,PIN1_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 1 */
     PORTC_ID,PIN2_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 2 */
     PORTC_ID,PIN3_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 3 */
     PORTC_ID,PIN4_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 4 */                                             PORTB_ID,PIN5_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 5 */
     PORTC_ID,PIN6_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 6 */
     PORTC_ID,PIN7_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 7 */

                     /**************** PORTD PINS **************/
     PORTD_ID,PIN0_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 0 */
     PORTD_ID,PIN1_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 1 */
     PORTD_ID,PIN2_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 2 */
     PORTD_ID,PIN3_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 3 */
     PORTD_ID,PIN4_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 4 */
     PORTD_ID,PIN5_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 5 */
     PORTD_ID,PIN6_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 6 */
     PORTD_ID,PIN7_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 7 */

                     /**************** PORTE PINS **************/
     PORTE_ID,PIN0_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 0 */
     PORTE_ID,PIN1_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 1 */
     PORTE_ID,PIN2_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 2 */
     PORTE_ID,PIN3_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 3 */
     PORTE_ID,PIN4_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 4 */
     PORTE_ID,PIN5_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 5 */

                     /**************** PORTF PINS **************/
     PORTF_ID,PIN0_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 0 */
     PORTF_ID,PIN1_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 1 */   /* led */
     PORTF_ID,PIN2_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 2 */
     PORTF_ID,PIN3_ID,PORT_PIN_IN,OFF,STD_LOW,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 3 */
     PORTF_ID,PIN4_ID,PORT_PIN_IN,PULL_UP,STD_HIGH,PORT_PIN_MODE_DIO,MODE_UNCHANGEABLE,DIRECTION_UNCHANGEABLE,   /* PIN 4 */   /* switch */



};



