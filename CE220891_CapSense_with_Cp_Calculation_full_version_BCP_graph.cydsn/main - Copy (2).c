/*****************************************************************************
* File Name: main.c
*
* Version: 1.00
*
* Update - 12-Aug-2021 (David Waskevich) --> open circuit test using Cp array in RAM
*
* Update - 17-Aug-2021 (David Waskevich) --> adding Cp calculations using scan parameters
*
* Update - 18-Aug-2021 (David Waskevich) --> added check for scan clock source which
*                                            appeard to be responsible for 2x error in Cp
*
* Update - 19-Aug-2021 (David Waskevich) --> created function to calculate Cp
*
* Update -  1-Oct-2021 (David Waskevich) --> adding scan parameter printing
*
* Update -  4-Oct-2021 (David Waskevich) --> adapting to BCP graphing for delta Cp
*
* Description: This code example demonstrates the use of a CapSense Slider and 
* Buttons using PSoC 4100S Plus Device. As soon as the user touches the button
* respective LED will glow. When user touches the slider, LEDs up to the relative
* position glows. Touching CapSense buttons and slider for about 2 sec will auto-reset
* the sensor, turning off LEDs. Apart from that there is a breathing LED which keeps 
* glowing on the User LED which has been implemented using SmartIO.
*
* Related Document: CE220891_CapSense_with_Breathing_LED.pdf
*
* Hardware Dependency: CY8CKIT-149 PSoC 4100S Plus Prototyping Kit
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*****************************************************************************/
/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "CapSense.h"
#include <project.h>
#include <stdio.h>
#include "calculateCp.h"

/*****************************************************************************
* MACRO Definitions
*****************************************************************************/     

/* Boolean constants */
#define LED_ON						(1u)
#define LED_OFF						(0u)
#define FALSE                       (0u)
#define TRUE                        (1u)

/*Set the macro value to '1' to use tuner for debugging and tuning CapSense sensors
  Set the macro value to '0' to disable the tuner*/
#define ENABLE_TUNER                (1u)

/* NOTE - Sensor_Cp test requires CapSense BIST library. Turn off SmartSense and enable BIST. */

/* define register map that will be exposed to I2C host */
struct RegisterMap {
    uint16 deltaCp[CapSense_TOTAL_CSD_SENSORS]; /* holds delta Cp values (based on finger touch) */
    uint16 SensorCp_Calculated[CapSense_TOTAL_CSD_SENSORS]; /* holds calculated sensor Cp values */
    uint16 CycleCount; /* holds I2C Master-read cycle count */
    uint16 CmodValue; /* holds measured value of Cmod capacitor on P4[1] */
    uint16 SensorCp_BIST[CapSense_TOTAL_CSD_SENSORS]; /* holds sensor Cp values calculated using BIST CapSense_GetSensorCapacitance API */
    uint8 EndOfBuffer; /* arbitrary marker */
} RegisterMap;

/* construct look-up table for iDAC gain values */
uint16 static const GainValueLUT[] = {37u, 300u, 2400, 1200};

/* Finite state machine for device operating states 
    SENSOR_SCAN - Sensors are scanned in this state
    WAIT_FOR_SCAN_COMPLETE - CPU is put to sleep in this state
    PROCESS_DATA - Sensor data is processed, LEDs are controlled,
                   and I2C buffer is updated in this state */
typedef enum
{
    SENSOR_SCAN = 0x01u,                
    WAIT_FOR_SCAN_COMPLETE = 0x02u,     
    PROCESS_DATA = 0x03u,               
} DEVICE_STATE;

int main()
{	        
    uint8 loopCount;
	CapSense_FLASH_WD_STRUCT const *ptrFlashWdgt; /* pointer to widget array in Flash */
//    CapSense_RAM_SNS_STRUCT *ptrSns; /* pointer variable to point to widget's sensor array in SRAM */
//    CapSense_RAM_WD_BUTTON_STRUCT *ptrRamWdgt; /* pointer to widget array in SRAM */
    CapSense_TST_MEASUREMENT_STATUS_ENUM measurementStatus; /* variable to pass to CapSense_GetSensorCapacitance API */
    uint32 status; /* EZI2C function call status return */
    
    char printBuf[100];
    char rxChar;
    uint8 printIndividual = FALSE;
    uint32 widgetID = 0;
    
    /* Variable to hold the current device state 
    *  State machine starts with Sensor_Scan state after power-up
    */
    DEVICE_STATE currentState = SENSOR_SCAN;  
           
     /* Enable global interrupts. */
    CyGlobalIntEnable; 

    /* Start EZI2C block */
    EZI2C_Start();
    /* Start CapSense block */
    CapSense_Start();
    
    UART_Start();
    UART_UartPutString("\r\nUART started ...\r\n");
    
    #if ENABLE_TUNER
        /* Set up I2C communication data buffer with CapSense data structure 
        to be exposed to I2C master on a primary slave address request (Tuner) */
        EZI2C_EzI2CSetBuffer1(sizeof(CapSense_dsRam), sizeof(CapSense_dsRam),(uint8 *)&CapSense_dsRam);
    #endif
    
    /* Set up communication data buffer to be exposed to I2C master at secondary slave address (BCP). */
	EZI2C_EzI2CSetBuffer2(sizeof(RegisterMap), sizeof(RegisterMap), (uint8 *) &RegisterMap);

    RegisterMap.EndOfBuffer = 0x55; /* arbitrary value to mark end of buffer */
    
    RegisterMap.CmodValue = (uint16) CapSense_GetExtCapCapacitance(CapSense_TST_CMOD_ID);
    
    sprintf(printBuf, "\r\nCmod = %hu pF\r\n", RegisterMap.CmodValue);
    UART_UartPutString(printBuf);
    
    UART_UartPutString("\r\nCY8CKIT-149 sensors (iDAC auto-cal enabled):\r\n\tSensor 0 --> P0.1 (no load R58) \
        \r\n\tSensors 1-3 --> wing board buttons (CSD, Rx-only) \
        \r\n\tSensors 4-9 --> wing board linear slider segments\r\n\tSensor 10 --> P0.0 (no connect) \
        \r\n\tSensor 11 --> P0.7 (on-board 12pf ECO load capacitor)\r\n\n");
    
    loopCount = 0;            	
    for(uint8 i = 0; i < CapSense_TOTAL_WIDGETS; i++)
	{
		/* using structure pointer to access widget array */
		ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[i]; /* use loop index "i" to point to current widget */
        for(uint8 j = 0; j < ptrFlashWdgt->totalNumSns; j++) /* process all sensors associated with current widget */
		{
			RegisterMap.SensorCp_BIST[loopCount] = (uint16) CapSense_GetSensorCapacitance(i, j, &measurementStatus); /* BIST calculated */
			loopCount++; /* increment index to next location in RegisterMap */
		}
    }    
    sprintf(printBuf, "BIST Cp values ... %hu, %hu, %hu, %hu, %hu, %hu, %hu, %hu, %hu, %hu, %hu, %hu\r\n\n",
        RegisterMap.SensorCp_BIST[0],
        RegisterMap.SensorCp_BIST[1],
        RegisterMap.SensorCp_BIST[2],
        RegisterMap.SensorCp_BIST[3],
        RegisterMap.SensorCp_BIST[4],
        RegisterMap.SensorCp_BIST[5],
        RegisterMap.SensorCp_BIST[6],
        RegisterMap.SensorCp_BIST[7],
        RegisterMap.SensorCp_BIST[8],
        RegisterMap.SensorCp_BIST[9],
        RegisterMap.SensorCp_BIST[10],
        RegisterMap.SensorCp_BIST[11]);
    UART_UartPutString(printBuf);
    
    UART_UartPutString("Cp formula = [idac_gain * mod_idac * raw] / [vref * scan_freq * MAX_resolution] +\r\n");
    UART_UartPutString("             [idac_gain * comp_idac] / [vref * scan_freq].\r\n\n");
    
    sprintf(printBuf, "MODULATOR_CLK_FREQ = %u\r\n\n", MODULATOR_CLK_FREQ);
    UART_UartPutString(printBuf);
    
    /* calculate Cp for first sensor */
    sprintf(printBuf, "Sensor  0 --> %lu\r\n\n", calculateCp(CapSense_BTN0_WDGT_ID, CapSense_BTN0_SNS0_ID));
    UART_UartPutString(printBuf);
                       
    sensor_Cp_calculated = calculateCp(CapSense_BTN0_WDGT_ID, CapSense_BTN0_SNS0_ID);

    UART_UartPutString("Scan parameters for sensor 0 ...\r\nmod, comp, raw, divider, freq, Vref, index, gain, resolution, N-1, Cp,      |---BIST----|\r\n");
    sprintf(printBuf, "Sensor 0 --> %lu, %lu, %5lu, %lu, %luMHz, %lu, %lu, %4lu, %2lu, %5lu, Cp= %5lu, BIST= %5hu\r\n\n", mod_iDAC_value,
        comp_iDAC_value, sensor_raw_count, snsClkDiv, snsClkFreq, vref,
        idacGainTableIndex, iDAC_gain, scan_resolution, scan_resolution_MAX_value, sensor_Cp_calculated,
        RegisterMap.SensorCp_BIST[0]);
    UART_UartPutString(printBuf);
    
    UART_UartPutString("\r\nPress <enter> to print all sensor Cp's.\r\n\n");
     
    for(;;)
    {
        /* Switch between SENSOR_SCAN->WAIT_FOR_SCAN_COMPLETE->PROCESS_DATA states */
        switch(currentState)
        {
            case SENSOR_SCAN:
	            /* Initiate new scan only if the CapSense block is idle */
                if(CapSense_NOT_BUSY == CapSense_IsBusy())
                {
                    #if ENABLE_TUNER
                        /* Update CapSense parameters set via CapSense tuner before the 
                           beginning of CapSense scan 
                        */
                        CapSense_RunTuner();
                    #endif
                    
                    /* Scan widget(s) */
                    if(FALSE == printIndividual)
                        CapSense_ScanAllWidgets();
                    else
                    {
                        CapSense_SetupWidget(widgetID);                        
                        /* Scan widget configured by CSDSetupWidget API */
                        CapSense_Scan();
                    }
                                        
                    /* Set next state to WAIT_FOR_SCAN_COMPLETE  */
                    currentState = WAIT_FOR_SCAN_COMPLETE;
                }
                break;

            case WAIT_FOR_SCAN_COMPLETE:

                /* Put the device to CPU Sleep until CapSense scanning is complete*/
                if(CapSense_NOT_BUSY != CapSense_IsBusy())
                {
                    CySysPmSleep();
                }
                /* If CapSense scanning is complete, process the CapSense data */
                else
                {
                    currentState = PROCESS_DATA;
                }
                break;
        
            case PROCESS_DATA:
                
                /* Process data on all the enabled widgets */
                CapSense_ProcessAllWidgets();
                
                if(CapSense_IsAnyWidgetActive())
                    LED1_Write(LED_ON);
                else LED1_Write(LED_OFF);

                if(0 != UART_SpiUartGetRxBufferSize())
                {
                    rxChar = UART_UartGetChar();
                    if('b' == rxChar)
                    {
                        UART_UartPutString("\r\nRECALCULATING BIST VALUES\r\n\n");
                        loopCount = 0;            	
                        for(uint8 i = 0; i < CapSense_TOTAL_WIDGETS; i++)
                    	{
                    		/* using structure pointer to access widget array */
                    		ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[i]; /* use loop index "i" to point to current widget */
                            for(uint8 j = 0; j < ptrFlashWdgt->totalNumSns; j++) /* process all sensors associated with current widget */
                    		{
                    			RegisterMap.SensorCp_BIST[loopCount] = (uint16) CapSense_GetSensorCapacitance(i, j, &measurementStatus); /* BIST calculated */
                                sprintf(printBuf, "Sensor %2d BIST Cp = %5hu\r\n", loopCount, RegisterMap.SensorCp_BIST[loopCount]);
                                UART_UartPutString(printBuf);
                    			loopCount++; /* increment index to next location in RegisterMap */
                    		}
                        }
                    }
                    if('x' == rxChar)
                    {
                        if(FALSE == printIndividual)
                        {
                            UART_UartPutString("Manually scanning each widget. Press <enter> for next sensor. Press 'x' to exit.\r\n\n");
                            CapSense_Stop();
                            widgetID = 0;
                            CapSense_SetupWidget(widgetID);
                            
                            /* Scan widget configured by CSDSetupWidget API */
                            CapSense_Scan();
                            
                            /* Put the device to CPU Sleep until CapSense scanning is complete*/
                            if(CapSense_NOT_BUSY != CapSense_IsBusy())
                            {
                                CySysPmSleep();
                            }
                        }
                        else
                        {
                            CapSense_Start();
                        }
                        printIndividual ^= 1u;
                    }
                    
                    if(FALSE == printIndividual)
                    {
                        UART_UartPutString("\r\n-- Using calculateCp function --\r\n\n");
                        
                        /* calculate sensor Cp's from scan parameters */
                        loopCount = 0;            	
                        for(uint8 i = 0; i < CapSense_TOTAL_WIDGETS; i++)
                    	{
                    		ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[i];
                    		for(uint8 j = 0; j < ptrFlashWdgt->totalNumSns; j++) /* process all sensors associated with current widget */
                    		{
                                sprintf(printBuf, "Sensor %2d -->  Cp = %5lu\r\n", loopCount, calculateCp(i, j));
                                UART_UartPutString(printBuf);
                    			loopCount++; /* increment index to next location in RegisterMap */
                    		}
                        }
                        
                        sprintf(printBuf, "\r\nCapSense_CSD_IDACA_REG = 0x%08lx\r\n", CapSense_CSD_IDACA_REG);
                        UART_UartPutString(printBuf);
                        
                        UART_UartPutString("\r\nPress <enter> to print new sensor Cp's. Press 'b' to recalculate BIST values. Press 'x' for individual.\r\n\n");
                    }
                    if(TRUE == printIndividual)
                    {
                        /* calculate sensor Cp's from scan parameters */
                		ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[widgetID];
                		for(uint8 j = 0; j < ptrFlashWdgt->totalNumSns; j++) /* process all sensors associated with current widget */
                		{
                            sensor_Cp_calculated = calculateCp(widgetID, j);
                            
                            sprintf(printBuf, "Widget %2lu -->  Cp = %5lu\r\n", widgetID, sensor_Cp_calculated);
                            UART_UartPutString(printBuf);
                            
                            sprintf(printBuf,"Scan parameters for sensor %2d ...\r\nmod, comp, raw, divider, freq, Vref, index, gain, resolution, N-1, Cp\r\n", j);
                            UART_UartPutString(printBuf);
                            sprintf(printBuf, "Sensor %2d --> %lu, %lu, %5lu, %lu, %luMHz, %lu, %lu, %4lu, %2lu, %5lu, Cp= %5lu\r\n\n", j, mod_iDAC_value,
                                comp_iDAC_value, sensor_raw_count, snsClkDiv, snsClkFreq, vref,
                                idacGainTableIndex, iDAC_gain, scan_resolution, scan_resolution_MAX_value, sensor_Cp_calculated);
                            UART_UartPutString(printBuf);                            
                            
                			loopCount++; /* increment index to next location in RegisterMap */
                		}
                        
                        /* print and decode iDAC information from CSD_IDACA_REG register */
                        sprintf(printBuf, "CapSense_CSD_IDACA_REG = 0x%08lx, Mod_iDAC = %lu, iDAC gain = %d\r\n\n", CapSense_CSD_IDACA_REG, (CapSense_CSD_IDACA_REG & 0x000000ff), \
                            (GainValueLUT[(CapSense_CSD_IDACA_REG & 0x00c00000) >> 22]) << ((CapSense_CSD_IDACA_REG & 0x02000000) >> 25));
                        UART_UartPutString(printBuf);
                        
                        widgetID++;
                        if(widgetID >= CapSense_TOTAL_CSD_WIDGETS)
                        {
                            widgetID = 0;
                            loopCount = 0;
                        }
                    }
                }                
                else
                {
                    /* calculate sensor Cp's from scan parameters */
                    loopCount = 0;            	
                    for(uint8 i = 0; i < CapSense_TOTAL_WIDGETS; i++)
                	{
                		ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[i];
                		for(uint8 j = 0; j < ptrFlashWdgt->totalNumSns; j++) /* process all sensors associated with current widget */
                		{
                            RegisterMap.SensorCp_Calculated[loopCount] = (uint16) calculateCp(i, j); /* calculated Cp */
                            RegisterMap.deltaCp[loopCount] = (uint16) calculateDeltaCp(i, j);
                			loopCount++; /* increment index to next location in RegisterMap */
                		}
                    }
                }
                
                /* I2C - BCP interface */
                status = EZI2C_EzI2CGetActivity(); /* Get slave status to see if a Master read transaction happened. */
                if(1 == (status & EZI2C_EZI2C_STATUS_READ2))
                {
                    RegisterMap.CycleCount += 1; /* arbitrary transaction counter */
                }
                
                /* Set the device state to SENSOR_SCAN */
                currentState = SENSOR_SCAN;  
                break;  
             
            /*******************************************************************
             * Unknown power mode state. Unexpected situation.
             ******************************************************************/    
            default:
                break;
        } 
    }
}

/* [] END OF FILE */
