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
 * Filename:    calculateCp.c
 *
 * Description: Calculates parasitic capacitance (Cp) for dual-iDAC
 *              CSD sensors using scan parameters and values retrieved
 *              from the dsRAM array. This should replicate results
 *              obtained from the BIST CapSense_GetSensorCapacitance()
 *              API without having to disrupt normal scanning to run
 *              the BIST algorithm (which can take up to 1 ms per
 *              sensor).
 *
 * Cp formula = [idac_gain * mod_idac * raw] / [vref * scan_freq * MAX_resolution] +
 *              [idac_gain * comp_idac] / [vref * scan_freq]
 *
 * Returns sensor Cp in femtofarads. Returns 0x00 if requested widget or 
 *              sensor is out of range.
 *
 * ========================================
*/
#include <project.h>
#include "calculateCp.h"

CapSense_FLASH_WD_STRUCT const *ptrFlashWdgt; /* pointer to widget array in Flash */
CapSense_RAM_SNS_STRUCT *ptrSns; /* pointer variable to point to widget's sensor array in SRAM */
CapSense_RAM_WD_BUTTON_STRUCT *ptrRamWdgt; /* pointer to widget array in SRAM */

///* variables to hold scan parameters retrieved from CapSense dsRAM structure and used to calculate sensor Cp */
//uint32 iDAC_gain, mod_iDAC_value, comp_iDAC_value, sensor_raw_count, snsClkDiv, snsClkSource, snsClkFreq,
//    vref, scan_resolution, idacGainTableIndex, scan_resolution_MAX_value, sensor_Cp_calculated;

uint32 calculateCp(uint32 widgetId, uint32 sensorElement)
{
    ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[widgetId];
    if(widgetId < CapSense_TOTAL_CSD_WIDGETS && sensorElement < ptrFlashWdgt->totalNumSns)
    {
        ptrRamWdgt = ptrFlashWdgt->ptr2WdgtRam; /* set pointer to requested widget in SRAM */
        ptrSns = ptrFlashWdgt->ptr2SnsRam; /* set pointer to point to the current widget's sensor data in SRAM */
        ptrSns += sensorElement; /* adjust pointer to requested sensor element */
        mod_iDAC_value = ptrRamWdgt->idacMod[0]; /* modulator iDAC is associated with the entire widget */
        snsClkDiv = ptrRamWdgt->snsClk;
        snsClkSource = ptrRamWdgt->snsClkSource;
        if(CapSense_CLK_SOURCE_DIRECT != snsClkSource)  /* dithered clocks have 1/2 effective frequency */
            snsClkFreq = (MODULATOR_CLK_FREQ / snsClkDiv) / 2;
        else snsClkFreq = MODULATOR_CLK_FREQ / snsClkDiv;
        vref = CapSense_CSD_VREF_MV;
        idacGainTableIndex = ptrRamWdgt->idacGainIndex;
        scan_resolution = ptrRamWdgt->resolution;
        scan_resolution_MAX_value = (1u << scan_resolution) - 1;
        iDAC_gain = CapSense_idacGainTable[idacGainTableIndex].gainValue / 1000;
        comp_iDAC_value = ptrSns->idacComp[0]; /* compensation iDAC is associated with each sensor within the widget */
        sensor_raw_count = ptrSns->raw[0]; /* raw values are associated with each sensor within the widget */
        sensor_Cp_calculated = ((iDAC_gain * mod_iDAC_value * sensor_raw_count) / (( ((vref * snsClkFreq) / 100) * scan_resolution_MAX_value) / 1000) +
            (iDAC_gain * comp_iDAC_value) / (((vref * snsClkFreq) / 1000) / 100)) * 10;
        
        return sensor_Cp_calculated;
    }
    else
    {
        return 0x00;
    }
}

uint32 calculateDeltaCp(uint32 widgetId, uint32 sensorElement)
{
    ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[widgetId];
    uint32  untouchedCp, touchedCp;
    if(widgetId < CapSense_TOTAL_CSD_WIDGETS && sensorElement < ptrFlashWdgt->totalNumSns)
    {
        ptrRamWdgt = ptrFlashWdgt->ptr2WdgtRam; /* set pointer to requested widget in SRAM */
        ptrSns = ptrFlashWdgt->ptr2SnsRam; /* set pointer to point to the current widget's sensor data in SRAM */
        ptrSns += sensorElement; /* adjust pointer to requested sensor element */
        if(ptrSns->diff > 0)
        {
            mod_iDAC_value = ptrRamWdgt->idacMod[0]; /* modulator iDAC is associated with the entire widget */
            snsClkDiv = ptrRamWdgt->snsClk;
            snsClkSource = ptrRamWdgt->snsClkSource;
            if(CapSense_CLK_SOURCE_DIRECT != snsClkSource)  /* dithered clocks have 1/2 effective frequency */
                snsClkFreq = (MODULATOR_CLK_FREQ / snsClkDiv) / 2;
            else snsClkFreq = MODULATOR_CLK_FREQ / snsClkDiv;
            vref = CapSense_CSD_VREF_MV;
            idacGainTableIndex = ptrRamWdgt->idacGainIndex;
            scan_resolution = ptrRamWdgt->resolution;
            scan_resolution_MAX_value = (1u << scan_resolution) - 1;
            iDAC_gain = CapSense_idacGainTable[idacGainTableIndex].gainValue / 1000;
            comp_iDAC_value = ptrSns->idacComp[0]; /* compensation iDAC is associated with each sensor within the widget */
            sensor_raw_count = ptrSns->raw[0]; /* raw values are associated with each sensor within the widget */
            untouchedCp = ((iDAC_gain * mod_iDAC_value * sensor_raw_count) / (( ((vref * snsClkFreq) / 100) * scan_resolution_MAX_value) / 1000) +
                (iDAC_gain * comp_iDAC_value) / (((vref * snsClkFreq) / 1000) / 100)) * 10;
            sensor_raw_count = ptrSns->bsln[0];
            touchedCp = ((iDAC_gain * mod_iDAC_value * sensor_raw_count) / (( ((vref * snsClkFreq) / 100) * scan_resolution_MAX_value) / 1000) +
                (iDAC_gain * comp_iDAC_value) / (((vref * snsClkFreq) / 1000) / 100)) * 10;
            if(untouchedCp < touchedCp)
                sensor_Cp_calculated = 0;
            else sensor_Cp_calculated = untouchedCp - touchedCp;
            return sensor_Cp_calculated;
        }
        else return 0;
    }
    else
    {
        return 0x00;
    }
}

/* [] END OF FILE */
