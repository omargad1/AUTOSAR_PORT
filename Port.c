 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Omar Mohamed
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)
#include "Det.h"
#if(    (DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
     || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
     || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION) )
#error "The AR version of Det.h does not match the expected version"
#endif

#endif

/* pointer to the array of struct holding configuration */
static const Port_ConfigParameter* port_init_ptr=NULL_PTR;
static uint8 port_status=PORT_NOT_INITIALIZED;

/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy:non reentrant
* Service ID[hex]: 0x00
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
*              - Setup the mode of the pins
************************************************************************************/
void Port_Init(const Port_ConfigType * ConfigPtr )
{
#if( PORT_DEV_ERROR_DETECT==STD_ON)

    if(ConfigPtr==NULL_PTR){
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_Init_SID, PORT_E_PARAM_CONFIG);
    }
    else
#endif
 {

    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    uint8 i;                                   /* iterator for the for loop */

    port_init_ptr=ConfigPtr->Pins;

 for(i=0;i<PORT_PINS_NUMBER;i++)               /* for loop to initialize all the pins of the microcontroller */
 {

    switch(port_init_ptr[i].port_num)
    {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		         break;
     	case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;/* PORTB Base Address */
		         break;
	    case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		         break;
	    case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		         break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		         break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		         break;
    }
    
    if( ((port_init_ptr[i].pin_num == 3) && (port_init_ptr[i].pin_num == 7)) || ((port_init_ptr[i].pin_num == 5) && (port_init_ptr[i].pin_num == 0)) ) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , port_init_ptr[i].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( (port_init_ptr[i].pin_num == 2) && (port_init_ptr[i].pin_num <= 3) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
    
    if(port_init_ptr[i].direction == PORT_PIN_OUT)
    {
	    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , port_init_ptr[i].pin_num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        
        if(port_init_ptr[i].initial_value == STD_HIGH)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , port_init_ptr[i].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) ,port_init_ptr[i].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
    }
    else if(port_init_ptr[i].direction==PORT_PIN_IN)
    {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , port_init_ptr[i].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        
        if(port_init_ptr[i].resistor == PULL_UP)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , port_init_ptr[i].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        }
        else if(port_init_ptr[i].resistor == PULL_DOWN)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , port_init_ptr[i].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , port_init_ptr[i].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , port_init_ptr[i].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        }
    }
    else
    {
        /* Do Nothing */
    }

    /* Setup the pin mode as GPIO */
    if(port_init_ptr[i].mode==PORT_PIN_MODE_ADC)
    {
        SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ALT_FUNC_REG_OFFSET),port_init_ptr[i].pin_num);              /* set the alternate register to the corresponding bit */
        CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIGITAL_ENABLE_REG_OFFSET),port_init_ptr[i].pin_num);        /* Clear the digital enable bit for this pin */
        SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ANALOG_MODE_SEL_REG_OFFSET ),port_init_ptr[i].pin_num);              /* set the anloge enable register to the corresponding bit */

    }

    else{

    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) ,port_init_ptr[i].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , port_init_ptr[i].pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (port_init_ptr[i].pin_num * 4));     /* Clear the PMCx bits for this pin */

       if(port_init_ptr[i].mode==PORT_PIN_MODE_DIO){
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , port_init_ptr[i].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)port_init_ptr[i].mode << (port_init_ptr[i].pin_num * 4));     /* set the PMCx bits for this pin */
       }

       else{

           SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ALT_FUNC_REG_OFFSET),port_init_ptr[i].pin_num);                                         /* set the alternate register to the corresponding bit */
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)port_init_ptr[i].mode << (port_init_ptr[i].pin_num * 4));      /* set the PMCx bits for this pin */
       }


    } /* end of the else statement */



 } /* end of the for loop */

 port_status=PORT_INITIALIZED;

}

} /*end of the Port_Init function */




/************************************************************************************
* Service Name: Port_SetPinDirection
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Service ID[hex]: 0x01
* Parameters (in): Port Pin ID number && Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
*
************************************************************************************/
#if (PORT_SETPIN_DIRECTION_API==STD_ON)
void Port_SetPinDirection(  Port_PinType Pin,  Port_PinDirectionType Direction )
{
    boolean error=FALSE;

#if (PORT_DEV_ERROR_DETECT==STD_ON)

    if(Pin >=PORT_PINS_NUMBER){
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,PORT_E_PARAM_PIN );
        error=TRUE;
    }
    else{
        /* do nothing */
    }
    if(port_init_ptr[Pin].direction_changeability==DIRECTION_CHANGEABLE){
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,PORT_E_DIRECTION_UNCHANGEABLE );
                error=TRUE;
    }
    else{
            /* do nothing */
        }

    if(port_status==PORT_NOT_INITIALIZED){
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,PORT_E_UNINIT  ); /*port is not initialized */
                       error=TRUE;
    }
    else{
               /* do nothing */
           }
#endif
    if(error==FALSE){

        volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

        switch(port_init_ptr[Pin].port_num)
          {
              case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                       break;
              case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;/* PORTB Base Address */
                       break;
              case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                       break;
              case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                       break;
              case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                       break;
              case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                       break;
          }

        if(Direction == PORT_PIN_OUT)
          {
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , port_init_ptr[Pin].pin_num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

              if(port_init_ptr[Pin].initial_value == STD_HIGH)
              {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) ,port_init_ptr[Pin].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
              }
              else
              {
                  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) ,port_init_ptr[Pin].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
              }
          }
          else if(Direction==PORT_PIN_IN)
          {
              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , port_init_ptr[Pin].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

              if(port_init_ptr[Pin].resistor == PULL_UP)
              {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , port_init_ptr[Pin].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
              }
              else if(port_init_ptr[Pin].resistor == PULL_DOWN)
              {
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , port_init_ptr[Pin].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
              }
              else
              {
                  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , port_init_ptr[Pin].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , port_init_ptr[Pin].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
              }
          }
    }
    else
             {
                 /* Do Nothing */
             }
} /* END OF THE FUNCTION */
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
)
{
#if (PORT_DEV_ERROR_DETECT==STD_ON)

    if(port_status==PORT_NOT_INITIALIZED){
           Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_RefreshPortDirection_SID,PORT_E_UNINIT  );
       }
    else
#endif
    {

 volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
int i;
for(i=0;i<PORT_PINS_NUMBER;i++)
{

              switch(port_init_ptr[i].port_num)
                {
                    case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                             break;
                    case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;/* PORTB Base Address */
                             break;
                    case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                             break;
                    case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                             break;
                    case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                             break;
                    case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                             break;
                }

              if( ((port_init_ptr[i].pin_num == 3) && (port_init_ptr[i].pin_num == 7)) || ((port_init_ptr[i].pin_num == 5) && (port_init_ptr[i].pin_num == 0)) ) /* PD7 or PF0 */
                 {
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */
                     SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , port_init_ptr[i].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                 }
                 else if( (port_init_ptr[i].pin_num == 2) && (port_init_ptr[i].pin_num <= 3) ) /* PC0 to PC3 */
                 {
                     /* Do Nothing ...  this is the JTAG pins */
                 }
                 else
                 {
                     /* Do Nothing ... No need to unlock the commit register for this pin */
                 }
              if(port_init_ptr[i].direction == PORT_PIN_OUT)
                  {
                      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , port_init_ptr[i].pin_num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

                      if(port_init_ptr[i].initial_value == STD_HIGH)
                      {
                          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , port_init_ptr[i].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                      }
                      else
                      {
                          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) ,port_init_ptr[i].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                      }
                  }
              else if(port_init_ptr[i].direction==PORT_PIN_IN)
                  {
                      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , port_init_ptr[i].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */


                  }
                  else
                  {
                      /* Do Nothing */
                  }
}


    }


}/* end of function */

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

void Port_GetVersionInfo(Std_VersionInfoType* versioninfo)
{
    boolean error=FALSE;
#if (PORT_DEV_ERROR_DETECT==STD_ON)

    if(port_status==PORT_NOT_INITIALIZED){
           Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_GetVersionInfo_SID,PORT_E_UNINIT  );
           error=TRUE;
       }
    else{
      /* do nothing */
    }
    if(versioninfo ==NULL_PTR ){
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_GetVersionInfo_SID,PORT_E_PARAM_POINTER);
        error=TRUE;
    }
    else{
/* do nothing */
    }
#endif
    if(error==FALSE){
        /* Copy the vendor Id */
            versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
            /* Copy the module Id */
            versioninfo->moduleID = (uint16)PORT_MODULE_ID;
            /* Copy Software Major Version */
            versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
            /* Copy Software Minor Version */
            versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
            /* Copy Software Patch Version */
            versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
    else{
 /* do nothing */
     }
}
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
)
{
    boolean error=FALSE;
#if (PORT_DEV_ERROR_DETECT==STD_ON)

    /* API service called prior to module initialization */
     if(port_status==PORT_NOT_INITIALIZED){
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID,PORT_E_UNINIT  );
            error=TRUE;
        }
     else{
         /* do nothing */
     }
     /*Incorrect Port Pin PORT_E_PARAM_PIN ID passed */
     if(Pin >=PORT_PINS_NUMBER){
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID,PORT_E_PARAM_PIN );
            error=TRUE;
        }
        else{
            /* do nothing */
        }

     /* pins mode is unchangeable during run time */
     if(port_init_ptr[Pin].mode_changeability==MODE_UNCHANGEABLE){

         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID,PORT_E_MODE_UNCHANGEABLE);
         error=TRUE;
     }
     else{
         /* do nothing */
     }
     /* invalid mode passed */
     if(Mode > PORT_PIN_MODE_ADC){
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID,PORT_E_PARAM_INVALID_MODE);
                  error=TRUE;
     }
     else{

     }

#endif

     if(error==FALSE){
         volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

               switch(port_init_ptr[Pin].port_num)
                 {
                     case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                              break;
                     case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;/* PORTB Base Address */
                              break;
                     case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                              break;
                     case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                              break;
                     case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                              break;
                     case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                              break;
                 }
               if( ((port_init_ptr[Pin].pin_num == 3) && (port_init_ptr[Pin].pin_num == 7)) || ((port_init_ptr[Pin].pin_num == 5) && (port_init_ptr[Pin].pin_num == 0)) ) /* PD7 or PF0 */
                 {
                     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */
                     SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , port_init_ptr[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                 }
                 else if( (port_init_ptr[Pin].pin_num == 2) && (port_init_ptr[Pin].pin_num <= 3) ) /* PC0 to PC3 */
                 {
                     /* Do Nothing ...  this is the JTAG pins */
                 }
                 else
                 {
                     /* Do Nothing ... No need to unlock the commit register for this pin */
                 }


               /* Setup the pin mode as GPIO */
               if(Mode==PORT_PIN_MODE_ADC)
               {
                   SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ALT_FUNC_REG_OFFSET),port_init_ptr[Pin].pin_num);              /* set the alternate register to the corresponding bit */
                   CLEAR_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_DIGITAL_ENABLE_REG_OFFSET),port_init_ptr[Pin].pin_num);        /* Clear the digital enable bit for this pin */
                   SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ANALOG_MODE_SEL_REG_OFFSET ),port_init_ptr[Pin].pin_num);              /* set the anloge enable register to the corresponding bit */

               }

               else{

               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) ,port_init_ptr[Pin].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , port_init_ptr[Pin].pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
               *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (port_init_ptr[Pin].pin_num * 4));     /* Clear the PMCx bits for this pin */

                  if(Mode==PORT_PIN_MODE_DIO){
                      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , port_init_ptr[Pin].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)Mode<< (port_init_ptr[Pin].pin_num * 4));     /* set the PMCx bits for this pin */
                  }

                  else{
                      SET_BIT(*(volatile uint32*)((volatile uint8*)PortGpio_Ptr+PORT_ALT_FUNC_REG_OFFSET),port_init_ptr[Pin].pin_num);                                         /* set the alternate register to the corresponding bit */
                      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)Mode << (port_init_ptr[Pin].pin_num * 4));      /* set the PMCx bits for this pin */
                  }
               } /* end of the else statement */
     }/* end of if condition */

}



