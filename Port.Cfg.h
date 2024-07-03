/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Omar Mohamed
 ******************************************************************************/

#ifndef PORT_CFG_H_
#define PORT_CFG_H_


/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION      (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION      (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION      (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_ON)

/*Pre-compile option for Port_SetPinDirection  API */
#define  PORT_SETPIN_DIRECTION_API             (STD_ON)
/* number of port pins to be initialized */
#define PORT_PINS_NUMBER                           (43U)

/* PORTS ID */
#define PORTA_ID     (0U)
#define PORTB_ID     (1U)
#define PORTC_ID     (2U)
#define PORTD_ID     (3U)
#define PORTE_ID     (4U)
#define PORTF_ID     (5U)

/* PINS ID */
#define PIN0_ID      (0U)
#define PIN1_ID      (1U)
#define PIN2_ID      (2U)
#define PIN3_ID      (3U)
#define PIN4_ID      (4U)
#define PIN5_ID      (5U)
#define PIN6_ID      (6U)
#define PIN7_ID      (7U)



/* port modes id */
#define PORT_PIN_MODE_DIO              (0U)     /* DIO MODE ID */
#define PORT_PIN_MODE_UART             (1U)     /* UART MODE ID */
#define PORT_PIN_MODE_SSI              (2U)     /* Synchronous serial interface mode id */
#define PORT_PIN_MODE_I2C              (3U)     /* I2C mode id */
#define PORT_PIN_MODE_M0PWM            (4U)     /* M0PWM mode id */
#define PORT_PIN_MODE_M0FAULT          (4U)     /* M0fault mode id */
#define PORT_PIN_MODE_M1PWM            (5U)     /* M1PWM mode id */
#define PORT_PIN_MODE_IDX              (6U)     /* QEI mode id */
#define PORT_PIN_MODE_PHASE            (6U)     /* QEI mode id */
#define PORT_PIN_MODE_TIMER            (7U)     /* timer mode id */
#define PORT_PIN_MODE_CAN              (8U)     /* CAN mode id */
#define PORT_PIN_MODE_USB              (8U)     /* USB mode id */
#define PORT_PIN_MODE_NMI              (8U)     /* NMI mode id */
#define PORT_PIN_MODE_COMPARATOR       (9U)     /* analog comparator output mode id */
#define PORT_PIN_MODE_ADC              (10U)     /* ADC mode id */







#endif /* PORT_CFG_H_ */
