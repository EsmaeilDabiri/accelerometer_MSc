/*
 * File:   newavr-main.c
 * Author: Lenovo
 *
 * Created on March 12, 2024, 9:57 AM
 */

#include <stdio.h>
#include <avr/io.h>
#define F_CPU               4000000UL
#include <util/delay.h>
#include <xc.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
//#include <avr/ioavr128db48.h>
////MEMORY REGISTERS
#define Read_Status_Register 0x05
#define write_Status_Register 0x01
#define Write_Enable 0x06
#define  Write_Disable 0x04
#define Read_Memory 0x03
#define write_Memory 0x02
////Accelerometer REGISTERS
#define DEVID_address 0x00
#define THRESH_TAP_address 0x1D
#define OFSX_address 0x1E
#define OFSY_address 0x1F
#define OFSZ _address 0x20
#define DUR_address 0x21
#define Latent_address 0x22
#define Window_address 0x23
#define THRESH_ACT_address 0x24
#define THRESH_INACT_address 0x25
#define PTIME_INACT_address 0x26
#define ACT_INACT_CTL_address 0x27
#define THRESH_FF_address 0x28
#define TIME_FF_address 0x29
#define TAP_AXES_address 0x2A
#define ACT_TAP_STATUS_address 0x2B
#define BW_RATE_address 0x2C
#define POWER_CTL_address 0x2D
#define INT_ENABLE_address 0x2E
#define INT_MAP_address 0x2F
#define INT_SOURCE_address 0x30
#define DATA_FORMAT_address 0x31
#define DATAX0_address 0x32
#define DATAX1_address 0x33
#define DATAY0_address 0x34
#define DATAY1_address 0x35
#define DATAZ0_address 0x36
#define DATAZ1_address 0x37
#define FIFO_CTL_address 0x38
#define FIFO_STATUS_address 0x39

#define ADXL345_READ        0x80
#define ADXL345_WRITE       0x00

#define x_upper_limit 50
#define x_lower_limit 10
#define y_upper_limit 50
#define y_lower_limit 10
// Function to initialize SPI
void SPI0_Init() {
    SPI0_CTRLA |=(1<<SPI_MASTER_bp)|(SPI_PRESC_DIV64_gc)|(1<<SPI_ENABLE_bp);
    SPI0_CTRLB |= (SPI_MODE_3_gc);
    SPI0_INTCTRL |=(1<<SPI_IE_bp);
    SPI0_INTFLAGS |= (1<<SPI_IF_bp);
    PORTMUX_SPIROUTEA = PORTMUX_SPI1_ALT1_gc; //USING PC4 TO PC7 AS SPI1 PINS
    }
void SPI0_Enable()
{
    SPI0.CTRLA |= SPI_ENABLE_bm;
}
void SPI0_Disable()
{
    SPI0.CTRLA &= ~SPI_ENABLE_bm;
}
uint8_t SPI0_ReadData()
{
 SPI0.DATA = 0xFF; //send dummy data
 while (!(SPI0.INTFLAGS & SPI_IF_bm)); /* Waits until data are exchanged*/

 return SPI0.DATA;
}
void SPI0_WriteData(uint8_t data)
{
 SPI0.DATA = data; //send data
 while (!(SPI0.INTFLAGS & SPI_IF_bm)); /* Waits until data are exchanged*/
}

void SPI0_WaitDataready()
{
    while (!(SPI0.INTFLAGS & SPI_RXCIF_bm));
}

void enable_measurement_mode ()
{
    PORTA_OUTCLR = PIN7_bm; //SS low
    SPI0_WriteData(POWER_CTL_address | ADXL345_WRITE);
    SPI0_WriteData(0x00); //first in standby mode
    PORTA_OUTSET = PIN7_bm; //SS high
    _delay_us(50);
    PORTA_OUTCLR = PIN7_bm; //SS low
    SPI0_WriteData(POWER_CTL_address | ADXL345_WRITE);
    SPI0_WriteData(0x08); //then in measurement mode
    PORTA_OUTSET = PIN7_bm; //SS high
    _delay_us(50);  
}
void enable_sleep_mode ()
{   
    PORTA_OUTCLR = PIN7_bm; //SS low
    SPI0_WriteData(POWER_CTL_address | ADXL345_WRITE);
    SPI0_WriteData(0x05);  // sleep mode + sampling freq of 4Hz
    PORTA_OUTSET = PIN7_bm; //SS high
    _delay_us(50);  
}

void setting_the_g_range (){
    PORTA_OUTCLR = PIN7_bm; //SS low
    SPI0_WriteData(DATA_FORMAT_address | ADXL345_WRITE);
    SPI0_WriteData(0x02);
    PORTA_OUTSET = PIN7_bm; //SS high
   _delay_us(50);
}

void main(void) {
    // Initialize SPI
    SPI0_Init();
_delay_us(50);
PORTC_DIRSET = PIN7_bm;  //SS IS OUTPUT
    PORTC_DIRSET = PIN6_bm;  //clk IS OUTPUT
    PORTC_DIRCLR = PIN5_bm; //MISO
    PORTC_DIRSET = PIN4_bm ; //MOSI
    PORTA_DIRCLR = PIN3_bm; //sleep mode trigger
    PORTC_DIRSET |= PIN2_bm | PIN3_bm;
    
    PORTA_OUTSET = PIN7_bm; //SS high
    PORTA_OUTCLR = PIN3_bm; // not sleep
    uint8_t xyz_data[6];
    uint16_t x_out, y_out, z_out;
    int x=1;
    PORTA_OUTCLR = PIN7_bm; //SS low
    
    /// clk init
    ccp_write_io((void*) &(CLKCTRL_MCLKCTRLA) ,( CLKCTRL_CLKSEL_OSCHF_gc ));
    //ccp_write_io((void*) &(CLKCTRL_MCLKCTRLB) ,( CLKCTRL_PEN_bm | CLKCTRL_PDIV_64X_gc ));
    ccp_write_io((void*) &(CLKCTRL_MCLKCTRLC) ,(CLKCTRL_CFDSRC_CLKMAIN_gc | CLKCTRL_CFDEN_bm ));
    ccp_write_io((void*) &(CLKCTRL_OSCHFCTRLA) ,(CLKCTRL_RUNSTDBY_bm | CLKCTRL_FRQSEL_1M_gc )); 
      
    //////setting the g range on +-8g
    setting_the_g_range();
      
    
    while (1) {
    switch (x){
               case 1: //measurement mode
                   enable_measurement_mode(); 
                   if(PORTA_IN & PIN3_bm)
                       x=2;
                   else
                       x=3;
                break;
                case 2: //sleep mode
                       enable_sleep_mode();
                       if(!(PORTA_IN & PIN3_bm))
                       x=1;
                   else
                       x=3;
                       
                break;
                case 3: //getting XYZ values
                        SPI0_WriteData(0x32 | ADXL345_READ); //starting address
                        for (int i=0; i<6; i++)
                        {
                         xyz_data[i]=SPI0_ReadData();  //reading 6 values
                        }
                        PORTA_OUTSET = PIN7_bm; //SS high
                        ///////
                        x_out = xyz_data[0] | (xyz_data[1] << 8) ; 
                        x_out /= 64;
                        y_out = xyz_data[2]| (xyz_data[3] <<8) ; 
                        y_out /= 64;
                        z_out = xyz_data[4]| (xyz_data[5] <<8) ; 
                        z_out /= 64; 
                        _delay_us(10);
                        x=4;
                break;
                case 4: //actions on thrusters
                    if((x_lower_limit<= x_out) && (x_out<= x_upper_limit))
                        PORTC_OUTSET = PIN2_bm;
                    else
                        PORTC_OUTCLR = PIN2_bm;
                    
                    if((y_lower_limit<=y_out) && (y_out<= y_upper_limit))
                        PORTC_OUTSET = PIN3_bm;
                    else
                        PORTC_OUTCLR = PIN3_bm;
                    
                    if(PORTA_IN & PIN2_bm)
                        x=5;
                    else if (PORTA_IN & PIN3_bm)
                       x=2;
                       else
                       x=1;
                      
                break;
                case 5: //save memory
                    //save memory code
                    if (PORTA_IN & PIN3_bm)
                       x=2;
                       else
                       x=1;
                    
                break;
                    
    }
  
   
    }
}