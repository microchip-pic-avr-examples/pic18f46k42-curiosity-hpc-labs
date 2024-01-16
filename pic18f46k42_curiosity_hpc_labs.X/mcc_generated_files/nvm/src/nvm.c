/**
 *
 * @file nvm.c
 *
 * @ingroup nvm_driver
 *
 * @brief This file contains the API implementations for the Non-Volatile Memory (NVM) driver.
 *
 * @version NVM Driver Version 1.0.0
 */

/*
© [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#include "../nvm.h"

/*
 * AoU: These variables are placed in the Access RAM using the "__near" qualifier.
 *      Additionally, "Address Qualifiers" must be configured to the "Require" under
 *      "Project Properties -> XC8 Compiler -> Optimizations".
 */
__near volatile static uint8_t unlockKeyLow;
__near volatile static uint8_t unlockKeyHigh;

void NVM_Initialize(void)
{
    NVM_StatusClear();
}

bool NVM_IsBusy(void)
{
    return (NVMCON1bits.WR || NVMCON1bits.RD);
}

nvm_status_t NVM_StatusGet(void)
{
    if (NVMCON1bits.WRERR == 1)
    {
        return NVM_ERROR;
    }
    else
    {
        return NVM_OK;
    }
}

void NVM_StatusClear(void)
{
    NVMCON1bits.WRERR = 0;
}

void NVM_UnlockKeySet(uint16_t unlockKey)
{
    unlockKeyHigh = (uint8_t) (unlockKey >> 8);
    unlockKeyLow = (uint8_t) unlockKey;
}

void NVM_UnlockKeyClear(void)
{
    unlockKeyHigh = 0x00;
    unlockKeyLow = 0x00;
}

flash_data_t FLASH_Read(flash_address_t address)
{
    //Save the table pointer
    uint32_t tablePointer = ((uint32_t) TBLPTRU << 16) | ((uint32_t) TBLPTRH << 8) | ((uint32_t) TBLPTRL);
	
    //Access Program Flash Memory
    NVMCON1bits.NVMREG = 2;
    
    //Load table pointer with the target address of the byte  
    TBLPTRU = (uint8_t) (address >> 16);
    TBLPTRH = (uint8_t) (address >> 8);
    TBLPTRL = (uint8_t) address;

    asm("TBLRD");
    
    //Restore the table pointer
    TBLPTRU = (uint8_t) (tablePointer >> 16);
    TBLPTRH = (uint8_t) (tablePointer >> 8);
    TBLPTRL = (uint8_t) tablePointer;
    
    return (TABLAT);
}

nvm_status_t FLASH_RowWrite(flash_address_t address, flash_data_t *dataBuffer)
{
    uint8_t flashDataCount = PROGMEM_PAGE_SIZE;
    
    //Save the table pointer
    uint32_t tablePointer = ((uint32_t) TBLPTRU << 16) | ((uint32_t) TBLPTRH << 8) | ((uint32_t) TBLPTRL);
    
    //Save global interrupt enable bit value
    uint8_t globalInterruptBitValue = INTCON0bits.GIE;   

    //Access program Flash memory
    NVMCON1bits.NVMREG = 2;   
    
    //Enable write operation
    NVMCON1bits.WREN = 1;   
     
    //Load table pointer with the target address of the byte  
    TBLPTRU = (uint8_t) (address >> 16);
    TBLPTRH = (uint8_t) (address >> 8);
    TBLPTRL = (uint8_t) address;
    
    while (flashDataCount-- > 0U)
    {       
        TABLAT = *dataBuffer;
        
        dataBuffer++;
        
        //If last latch to be written
        if (flashDataCount == 0U)
        {
            //Write program Flash memory 
            asm("TBLWT");
        }
        else
        {
            asm("TBLWTPOSTINC");
        }
    }
        
    //Disable global interrupt
    INTCON0bits.GIE = 0; 

    //Perform the unlock sequence 
    asm("asmopt push"); //save the current selection of optimizations
    asm("asmopt off"); //turn off assembler optimizations
    asm("banksel(_NVMCON2)"); //select the bank of the NVMCON2 register
    NVMCON2 = unlockKeyLow; //assign 'unlockKeyLow' to NVMCON2.
    asm("MOVF (_unlockKeyHigh&0xFF),w,c"); //load 'unlockKeyHigh' into the W register
    asm("MOVWF (_NVMCON2&0xFF),b"); //move the W register to NVMCON2

    //Start byte write
    asm("bsf (_NVMCON1bits&0xFF)," ___mkstr(_NVMCON1_WR_POSN) ",b"); //Set WR bit   
    asm("asmopt pop"); //restore assembler optimization settings

    //Restore global interrupt enable bit value
    INTCON0bits.GIE = globalInterruptBitValue;     
    
    //Disable write operation
    NVMCON1bits.WREN = 0;      
    
    if (NVMCON1bits.WRERR == 1) 
    {
        return NVM_ERROR;
    } 
    else 
    {
        return NVM_OK;
    }
}

nvm_status_t FLASH_PageErase(flash_address_t address)
{
    //Save the table pointer
    uint32_t tablePointer = ((uint32_t) TBLPTRU << 16) | ((uint32_t) TBLPTRH << 8) | ((uint32_t) TBLPTRL);
    
    //Save global interrupt enable bit value
    uint8_t globalInterruptBitValue = INTCON0bits.GIE;   
    
    //Load table pointer with the target address of the byte  
    TBLPTRU = (uint8_t) (address >> 16);
    TBLPTRH = (uint8_t) (address >> 8);
    TBLPTRL = (uint8_t) address;

    //Access program Flash memory
    NVMCON1bits.NVMREG = 2;
    
    //Performs an erase operation with the next WR command
    NVMCON1bits.FREE = 1;   
    
    //Enable erase operation
    NVMCON1bits.WREN = 1;    

    //Disable global interrupt
    INTCON0bits.GIE = 0;
    
    //Perform the unlock sequence 
    asm("asmopt push"); //save the current selection of optimizations
    asm("asmopt off"); //turn off assembler optimizations
    asm("banksel(_NVMCON2)"); //select the bank of the NVMCON2 register
    NVMCON2 = unlockKeyLow; //assign 'unlockKeyLow' to NVMCON2.
    asm("MOVF (_unlockKeyHigh&0xFF),w,c"); //load 'unlockKeyHigh' into the W register
    asm("MOVWF (_NVMCON2&0xFF),b"); //move the W register to NVMCON2

    //Start byte write
    asm("bsf (_NVMCON1bits&0xFF)," ___mkstr(_NVMCON1_WR_POSN) ",b"); //Set WR bit   
    asm("asmopt pop"); //restore assembler optimization settings
    
    //Restore global interrupt enable bit value
    INTCON0bits.GIE = globalInterruptBitValue;	

    //Disable erase operation
    NVMCON1bits.WREN = 0;      
    
    //Restore the table pointer
    TBLPTRU = (uint8_t) (tablePointer >> 16);
    TBLPTRH = (uint8_t) (tablePointer >> 8);
    TBLPTRL = (uint8_t) tablePointer;
    
    if (NVMCON1bits.WRERR == 1) 
    {
       return NVM_ERROR;
    }
    else
    {
        return NVM_OK;
    }
}

flash_address_t FLASH_PageAddressGet(flash_address_t address)
{
    return (flash_address_t) (address & ((PROGMEM_SIZE - 1U) ^ (PROGMEM_PAGE_SIZE - 1U)));
}

uint16_t FLASH_PageOffsetGet(flash_address_t address)
{
    return (uint16_t) (address & (PROGMEM_PAGE_SIZE - 1U));
}

eeprom_data_t EEPROM_Read(eeprom_address_t address)
{
    //Load NVMADR with the EEPROM address
    NVMADRH = (uint8_t) (address >> 8);
    NVMADRL = (uint8_t) address;
    
    //Access EEPROM
    NVMCON1bits.NVMREG = 0; 
    
    //Initiate Read
    NVMCON1bits.RD = 1;

    return NVMDAT;
}

void EEPROM_Write(eeprom_address_t address, eeprom_data_t data)
{
    //Save global interrupt enable bit value
    uint8_t globalInterruptBitValue = INTCON0bits.GIE;

    //Load NVMADR with the target address of the byte
    NVMADRH = (uint8_t) (address >> 8);
    NVMADRL = (uint8_t) address;

    //Load NVMDAT with the desired value
    NVMDAT = data;
    
     //Access EEPROM
    NVMCON1bits.NVMREG = 0;   
    
    //Enable write operation
    NVMCON1bits.WREN = 1;
    
    //Disable global interrupt
    INTCON0bits.GIE = 0;

    //Perform the unlock sequence 
    asm("asmopt push"); //save the current selection of optimizations
    asm("asmopt off"); //turn off assembler optimizations
    asm("banksel(_NVMCON2)"); //select the bank of the NVMCON2 register
    NVMCON2 = unlockKeyLow; //assign 'unlockKeyLow' to NVMCON2.
    asm("MOVF (_unlockKeyHigh&0xFF),w,c"); //load 'unlockKeyHigh' into the W register
    asm("MOVWF (_NVMCON2&0xFF),b"); //move the W register to NVMCON2

    //Start byte write
    asm("bsf (_NVMCON1bits&0xFF)," ___mkstr(_NVMCON1_WR_POSN) ",b"); //Set WR bit   
    asm("asmopt pop"); //restore assembler optimization settings
    
    //Restore global interrupt enable bit value
    INTCON0bits.GIE = globalInterruptBitValue; 
   
    //Disable write operation
    NVMCON1bits.WREN = 0;
}
