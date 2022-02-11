/* ========================================
 *
 * Copyright Cypress/Infineon, 2021
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Cypress/Infineon.
 *
 * Date:        19-Aug-2021
 * Author:      David Waskevich
 * Filename:    calculateCp.h
 *
 * ========================================
*/
#include <cytypes.h>
#include <project.h>

#define MODULATOR_CLK_FREQ      (CYDEV_BCLK__HFCLK__KHZ / CapSense_CSD_SCANSPEED_DIVIDER)

/* variables to hold scan parameters retrieved from CapSense dsRAM structure and used to calculate sensor Cp */
uint32 iDAC_gain, mod_iDAC_value, comp_iDAC_value, sensor_raw_count, snsClkDiv, snsClkSource, snsClkFreq,
    vref, scan_resolution, idacGainTableIndex, scan_resolution_MAX_value, sensor_Cp_calculated;
    
/*******************************************************************************
* Function Prototype
*******************************************************************************/
uint32 calculateCp(uint32 widgetId, uint32 sensorElement);
uint32 calculateDeltaCp(uint32 widgetId, uint32 sensorElement);
    
/* [] END OF FILE */
