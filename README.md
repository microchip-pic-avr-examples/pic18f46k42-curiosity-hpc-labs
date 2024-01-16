[![MCHP](images/microchip.png)](https://www.microchip.com)

# Curiosity HPC Labs - PIC18F46K42

The goal of this example is to provide 10 labs that will demonstrate Curiosity HPC board capabilities and showcase the functionality of PIC18F46K42 device. The concept is to create a state machine where you can navigate through each lab using the S1 button. The output of labs is shown using four LEDs provided by the Curiosity development board. Some labs need a potentiometer for Analog input. This project is primarily created with the aim of helping beginners in basic programming of MCUs.

This example also make use of the latest MPLAB Code Configurator (MCC) Melody, an easy-to-use plugin tool for MPLAB X IDE that you can use to generate codes for a more efficient use of the CPU and memory resources. All labs are written in C language and are compatible with the latest XC8 compiler.

## Related Documentation

- [PIC18F-K42 Family Product Page](https://www.microchip.com/en-us/product/PIC18F46K42)
- [PIC18F46K42 Data Sheet](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/PIC18%28L%29F26-27-45-46-47-55-56-57K42-Data-Sheet-40001919G.pdf)
- [Curiosity HPC Development Board User's guide](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/UserGuides/Curiosity-High-Pin-Count-Development-Board-User-Guide-40001856C.pdf)

## Software Used
 
- [MPLAB® X IDE](http://www.microchip.com/mplab/mplab-x-ide) v6.15 or newer
- [MPLAB XC8](http://www.microchip.com/mplab/compilers) v2.45 or newer
- [PIC18F-K_DFP Series Device Pack](https://packs.download.microchip.com) v1.13.292 or newer
- [MPLAB Code Configurator](https://www.microchip.com/en-us/tools-resources/configure/mplab-code-configurator) 5.5.7 or newer
- [MPLAB Code Configurator Melody](https://www.microchip.com/en-us/tools-resources/configure/mplab-code-configurator/melody) core 2.6.2 or newer

## Hardware Used
- [Curiosity HPC Development board](https://www.microchip.com/en-us/development-tool/DM164136) 

<img src="https://i.imgur.com/e74InQZ.jpg" width = "600"><br>



## System Setup
MPLAB Code Configurator (MCC) is used to configure the system setup for the PIC18F46K42. The clock is set to 500KHz. The watchdog timer is set through SWDTEN with a period of approximately four seconds.
<br><img src= "images/clock control.png">
<br><img src= "images/config1_config2.png">
<br><img src= "images/config3_thru_config5.png">
<br><img src= "images/pin_grid_view.png">
<br><img src= "images/pins.png">
<br><img src= "images/project_and_device_resources.png">

## LABS
The labs in this project are presented in the same order as they appear on the programmed labs. You can progress through each of the labs by simply pressing the S1 button on the board.

• Lab 1: Hello World (Turn On an LED) <br />
• Lab 2: Blink <br />
• Lab 3: Rotate (Moving the Light Across LEDs) <br />
• Lab 4: Analog-to-Digital Conversion (ADC) <br />
• Lab 5: Variable Speed Rotate <br />
• Lab 6: Pulse-Width Modulation (PWM) <br />
• Lab 7: Timers <br />
• Lab 8: Interrupts <br />
• Lab 9: Wake-up from Sleep Using Watchdog Timer <br />
• Lab 10: EEPROM <br />

## INPUTS AND DISPLAY
• Push Button Switch – Only one on-board push button switch S1 is utilized. S1 is connected to the PIC MCU’s RB4 pin and is used to switch to the next lab. <br />
• Potentiometer – A 10kΩ potentiometer connected to the RA0 pin is used in labs requiring analog inputs. <br />
• LEDs - The Curiosity HPC Development Board has four LEDs (D2 through D5) that are connected to I/O ports RA4 through RA7, respectively. These LEDs are used to display the output of the different labs. <br/>
## Labs Flow Chart
<img src="https://i.imgur.com/snXbThS.png" width = "600"><br>



### <u>Lab 1: Hello World</u>
#### Introduction
This lab shows how to turn on an LED.

#### Hardware Effects
LED D2 will light up and stay lit.

#### Summary
The LEDs are connected to the input-output (I/O) pins. First, the I/O pin must be configured to be an output. In this case, when one of these pins is driven high (LED_D2 = 1), the LED will turn on. These two logic levels are derived from the power pins of the PIC MCU. Since the PIC's power pin (VDD) is connected to 5V or 3.3V and the source (VSS) to ground (0V), a logic level of ‘1’ is equivalent to 5V or 3.3V, and a logic level of ‘0’ is 0V.

### <u>Lab 2: Blink</u>
#### Introduction
This lab shows how to blink an LED.

#### Hardware Effects
LED D2 blinks at a rate of approximately 1.5 seconds.

#### Summary
One way to create a delay is to spend time decrementing a value. In assembly, the timing can be accurately programmed since the user will have direct control on how the code is executed. In 'C', the compiler takes the 'C' and compiles it into assembly before creating the file to program to the actual PIC MCU (HEX file). Because of this, it is hard to predict exactly how many instructions it takes for a line of 'C' to execute. For a more accurate timing in C, this lab uses the MCU’s TIMER1 module to produce the desired delay. TIMER1 is discussed in Lab 7: Timers.
<br><img src= "https://i.imgur.com/hK95BJI.jpg">

### <u>Lab 3: ROTATE</u>
#### Introduction
This lab is built on Lab 1 and 2, which showed how to light up a LED and then make it blink using loops. This lab incorporates four onboard LEDs (D2, D3, D4 and D5) and the program will light up each LED in turn.

#### Hardware Effects
LEDs D2, D3, D4 and D5 light up in turn every 500 milliseconds. Once D5 is lit, D2 lights up and the pattern repeats.

#### Summary
In C, we use Binary Left Shift and Right Shift Operators (<< and >>, respectively) to move bits around in the registers. The shift operations are 9-bit operations involving the 8-bit register being manipulated and the Carry bit in the STATUS register as the ninth bit. With the rotate instructions, the register contents are rotated through the Carry bit. <br />

For example, for a certain register rotateReg, we want to push a ‘1’ into the LSB of the register and have the rest of the bits shift to the left, we can use the Binary Left Shift Operator (<<). We would first have to set up the Carry bit with the value that we want to push into the register before we execute shift instruction, as seen in the figure below.

###### Left Shift Binary Operation
![Lab3-left-shift-binary-operation](images/Lab3-left-shift-binary-operation.PNG)

Similarly, if we want to push a ‘1’ into the MSB of the register and have the rest of the bits shift to the right, we can use the Binary Right Shift Operator (>>). We would first have to set up the Carry bit with the value that we want to push into the register before we execute shift instruction, as seen in  the figure below.

###### Right Shift Binary Operation
![Lab3-right-shift-binary-operation](images/Lab3-right-shift-binary-operation.PNG)
 

![](https://i.imgur.com/6TIkKQC.jpg){width=auto height=auto align=center}

### <u>Lab 4: ANALOG-TO-DIGITAL CONVERSION (ADC)</u>
#### Introduction
This lab shows how to configure the ADC, run a conversion, read the analog voltage controlled by the on-board potentiometer, print the conversion result on UART and display the high order four bits on the display.

#### Hardware Effects
The four most significant bits of the ADC result are reflected onto each of the four LEDs respectively. Rotate the potentiometer to change the display. <br />
The ADC value will be printed on UART TX pin which is connected to pin RC6 through PPS. Connect this pin to the Virtual COM port's TX pin using a jumper wire to use the onboard serial to USB feature.

#### Summary
The PIC18-K42 family of devices have an on-board Analog-to-Digital Converter with Computation (ADCC) with 12 bits of resolution. The converter can be referenced to the device’s VDD or an external voltage reference. This lab references it to VDD. The result from the ADC is represented by a ratio of the voltage to the reference.
<br><img src= "https://i.imgur.com/xOCuHXb.png">
<br><img src= "images/adcc.png">

### <u>Lab 5: VARIABLE SPEED ROTATE</u>
#### Introduction
This lab combines all of the previous lab to produce a variable speed rotating LED display that is proportional to the ADC value. The ADC value and LED rotate speed are inversely proportional to each other.
#### Hardware Effects
Rotate the on-board potentiometer to change the speed of the LEDs shift. <br />
The ADC value will be printed on UART TX pin which is connected to pin RC6 through PPS. Connect this pin to the Virtual COM port's TX pin using a jumper wire to use the onboard serial to USB feature.
#### Summary
A crucial step in this lab is to check if the ADC value is 0. If it does not perform the zero check, and the ADC result is zero, the LEDs will rotate at an incorrect speed. This is an effect of the delay value underflowing from 0 to 255.

###### Program Flow
![Lab 5 Program Flow](images/Lab5-program-flow.PNG)

<br><img src= "https://i.imgur.com/JaJYUTV.png">

### <u>Lab 6: PULSE-WIDTH MODULATION (PWM)</u>
#### Introduction
In this lab, the PIC MCU generates a PWM signal that lights an LED with the potentiometer thereby controlling the brightness.
#### Hardware Effects
Rotating potentiometer will adjust the brightness of LED D5. <br />
The ADC value will be printed on UART TX pin which is connected to pin RC6 through PPS. Connect this pin to the Virtual COM port's TX pin using a jumper wire to use the onboard serial to USB feature.
#### Summary
Pulse-Width Modulation (PWM) is a scheme that provides power to a load by switching quickly between fully ON and fully OFF states. The PWM signal resembles a square wave where the high portion of the signal is considered the ON state and the low portion of the signal is considered the OFF state. The high portion, also known as the pulse width, can vary in time and is defined in steps. A longer, high ON time will illuminate the LED brighter. The frequency or period of the PWM does not change. The PWM period is defined as the duration of one cycle or the total amount of ON and OFF time combined. Another important term to take note is the PWM duty cycle which is the ratio of the pulse width to the period and is often expressed in percentage. A lower duty cycle corresponds to less power applied and a higher duty cycle corresponds to more power applied.

<br><img src= "https://i.imgur.com/m3owUho.png">
<br><img src= "images/tmr2.png">
<br><img src= "images/pwm6.png">

### <u>Lab 7: TIMERS</u>
#### Introduction
This lab will produce the same output as Lab 3: ROTATE. The only difference is that this version uses Timer1 to provide the delay routine.
#### Hardware Effects
LEDs rotate from right to left, similar to Lab 3.
#### Summary
Timer1 is a counter module that uses two 8-bit paired registers (TMR1H:TMR1L) to implement a 16-bit timer/counter in the processor. It may be used to count instruction cycles or external events that occur at or below the instruction cycle rate. <br />
This lab configures Timer1 to count instruction cycles and to set a flag when it rolls over. This frees up the processor to do meaningful work rather than wasting instruction cycles in a timing loop. Using a counter provides a convenient method of measuring time or delay loops as it allows the processor to work on other tasks rather than counting instruction cycles.
<br><img src= "https://i.imgur.com/H3HGzxq.jpg">
<br><img src= "images/tmr1.png">

### <u>Lab 8: INTERRUPTS</u>
#### Introduction
This lab discusses all about interrupts – its purpose, capabilities and how to set them up. Most interrupts are sourced from MCU peripheral modules. Some I/O pins can also be configured to generate interrupts whenever a change in state is detected. Interrupts usually signal events that require servicing by the software’s Interrupt Service Routine (ISR). Once an interrupt occurs, the program counter immediately jumps to the ISR and once the Interrupt Flag is cleared, resumes what it was doing before. It is a rather more efficient way of watching out for events than continuously polling a bit or register.
#### Hardware Effects
LEDs D5, D4, D3 and D2 rotate from left to right at a constant rate of 499.712 ms.
#### Summary
This lab demonstrates the advantage of using interrupts over polling. An interrupt is generated whenever the Timer0 register reaches 0xFF and goes back to reset value. This indicates that 500 ms has passed and it is time to rotate the light. This interrupt is serviced by the TMR0_ISR() function. Note that this is the same for Lab 7: TIMER1 but this time, we are not continuously watching the TMR1IF flag.
<br><img src= "https://i.imgur.com/uUNaVvt.jpg">
<br><img src= "images/tmr0.png">

### <u>Lab 9: WAKE-UP FROM SLEEP USING WATCHDOG TIMER</u>
#### Introduction
This lab will introduce the Sleep mode. SLEEP() function is used to put the device into a low power standby mode.
#### Hardware Effects
Once this lab is on RUNNING state, the watchdog timer will start counting. While in Sleep mode, LEDs D2/D4 and LEDs D3/D5 are turned ON and OFF respectively. Pressing the switch won't go to the next lab since the PIC is in Sleep mode. After the watchdog timer has reached its period, which is approximately 4 seconds for this lab, the PIC exits sleep mode and the four LEDs, D2 through D5, are toggled.
#### Summary
The Power-Down mode is entered by executing the SLEEP instruction. Upon entering Sleep mode, there are different conditions that can exist such as:
• WDT will be cleared but keeps running, if enabled for operation during Sleep.
• PD bit of the STATUS register is cleared.
• TO bit of the STATUS register is set.
• CPU clock is disabled.
Different PICs have different condition once they enter Sleep mode so it is recommended that the reader refer to the datasheet to know more of these conditions. <br />
The Watchdog Timer (WDT) is a system timer that generates a Reset if the firmware does not issue a CLRWDT instruction within the time-out period. WDT is typically used to recover the system from unexpected events. When the device enters Sleep, the WDT is cleared. If the WDT is enabled during Sleep, the WDT resumes counting. When the device exits Sleep, the WDT is cleared again. When a WDT time-out occurs while the device is in Sleep, no Reset is generated.
<br><img src= "https://i.imgur.com/osl4EM9.jpg">

### <u>Lab 10: EEPROM</u>
#### Introduction
This lab provides code for writing and reading a single byte onto the on-board EEPROM. EEPROM is nonvolatile memory, meaning that it does not lose its value when power is shut off. This is unlike RAM, which will lose its value when no power is applied. The EEPROM is useful for storing variables that must still be present during no power. A good use case is to store calibration data for the user application and have it loaded on every boot-up. It is also convenient to use if the entire RAM space is used up. Writes and reads to the EEPROM are relatively quick, and are much faster than program memory operations.
#### Hardware Effects
The top 4 MSBs of the ADC is written to EEPROM. These are read afterwards and displayed on the LEDs. Rotating the potentiometer changes value of the ADC to be written to and read from EEPROM.
#### Summary
This lab has a similar appearance to Lab 4: ADC. But instead of directly moving the ADC result directly onto the LEDs, it performs a simple “write” and “read” on the EEPROM. As shown on figure below, the top 4 MSBs of the ADC result is first written to EEPROM, and retrieved later from the same address before moving onto the LEDs.

###### Program Flow
![Lab 10 Program Flow](images/Lab10-program-flow.PNG)
<br><img src= "https://i.imgur.com/CP6hlMJ.png">	
<br><img src= "images/nvm.png">

## Operation
Program Device using MPLABX project provided. Press S1 to switch through each lab. Rotate Potentiometer while in ADCC, Variable Speed Rotate, PWM and EEPROM labs.

## Summary
After programming and pressing S1 you have observed all 10 labs demonstrated by this code example.

Note: For the complete step by step guide in making this example, [click here](http://ww1.microchip.com/downloads/en/DeviceDoc/Curiosity%20HPC%20Demo%20Code.zip) and open the Read Me document inside the downloaded folder.
