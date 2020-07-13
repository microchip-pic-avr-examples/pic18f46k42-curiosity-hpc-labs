[![MCHP](https://cldup.com/U0qhLwBijF.png)](https://www.microchip.com)
# I.Introduction
The goal of this example is to provide 10 labs that will demonstrate Curiosity HPC board capabilities and showcase the functionality of PIC18F46K42 device. The concept is to create a state machine where you can navigate through each lab using the S1 button. The output of labs is shown using four LEDs provided by the Curiosity development board. Some labs need a potentiometer for Analog input. This project is primarily created with the aim of helping beginners in basic programming of MCUs.

This example also make use of the latest MPLAB Code Configurator (MCC v3.55), an easy-to-use plugin tool for MPLAB X IDE(v4.15) that you can use to generate codes for a more efficient use of the CPU and memory resources. All labs are written in C language and are compatible with the latest XC8 compiler (XC8 v1.45).

# II.Tools
## A.Curiosity HPC Development Board
![](https://i.imgur.com/e74InQZ.jpg){width=auto height=auto align=center}

## B.Software
a.MPLAB X IDE [(http://www.microchip.com/mplab/mplab-x-ide](http://www.microchip.com/mplab/mplab-x-ide)) 

b.	MPLAB X Code Configurator [(http://www.microchip.com/mplab/mplab-code-configurator](http://www.microchip.com/mplab/mplab-code-configurator)) 

c.	XC8 Compilers ([http://www.microchip.com/mplab/compilers](http://www.microchip.com/mplab/compilers)) 

# III. Flow Chart
![](https://i.imgur.com/snXbThS.png){width=auto height=auto align=center}
# IV.	System Setup
MPLAB Code Configurator (MCC) is used to configure the system setup for the PIC18F46K42. The clock is set to 500KHz. The watchdog timer is set through SWDTEN with a period of approximately four seconds.
![](https://i.imgur.com/AoDf3F1.jpg){width=auto height=auto align=center}

# V.	Lab Descriptions
## Lessons:
### A.	Hello World
This lab shows how to configure, initialize and turn on an I/O (LED_D2) pin.

### B.	Blink
This lab shows how to create a blinking LED using Timer1. LED_D2, which is OFF at its initial state, toggles every 1.5 seconds creating a blinking sequence. The Timer1 module is used for accurate timing.

![](https://i.imgur.com/hK95BJI.jpg){width=auto height=auto align=center}

### C.	Rotate
LEDs D2, D3, D4 and D5 light up in turn every 500 milliseconds software delay. Once D5 is lit, D2 lights up and the pattern repeats.  

![](https://i.imgur.com/6TIkKQC.jpg){width=auto height=auto align=center}

### D.	ADC 
This lab shows how to configure the ADC, run a conversion, read the analog voltage controlled by the on-board potentiometer, and display the high order four bits on the LEDs. 

![](https://i.imgur.com/xOCuHXb.png){width=auto height=auto align=center}

![](https://i.imgur.com/F3IOwRY.jpg){width=auto height=auto align=center}

### E.	VSR
This lesson combines all of the previous lessons to produce a variable speed rotating LED display. The LEDs rotates from right to left. The Speed of the rotation depends on the ADC value provided by the potentiometer.

![](https://i.imgur.com/JaJYUTV.png){width=auto height=auto align=center}

### F.	PWM
In this lab, rotating the potentiometer changes the light intensity of LED_D5. The value of the ADC provided by the potentiometer changes the value of the duty cycle which determines the brightness of the LED.

![](https://i.imgur.com/m3owUho.png){width=auto height=auto align=center}

![](https://i.imgur.com/go1EEJ4.jpg){width=auto height=auto align=center}

![](https://i.imgur.com/qPYyynM.jpg){width=auto height=auto align=center}


### G.	Timers
This lab produces the same output as rotate lab the only difference is that it uses Timer1 instead of software delay function.

![](https://i.imgur.com/H3HGzxq.jpg){width=auto height=auto align=center}

![](https://i.imgur.com/oNMNeCR.jpg){width=auto height=auto align=center}

### H.	Interrupt
Interrupt lab functions like the timer lab except that it rotates from left to right and uses Timer 0 interrupt instead of polling. Interrupts signal events that require servicing by the software’s Interrupt Service Routine(ISR). It is a more efficient way of watching out for events than continuously polling a register.

![](https://i.imgur.com/uUNaVvt.jpg){width=auto height=auto align=center}

![](https://i.imgur.com/vd7ILcH.jpg){width=auto height=auto align=center}

### I.	Sleep/Wakeup
This lab introduces the Sleep mode function which puts the device into a low power standby mode. LEDs D2/D4 are OFF and LEDs D3/D5 are ON at the start. For the duration of sleep period, pressing the switch won’t proceed to the next lab. After the watchdog timer reached its time out period which is approximately four seconds, LEDs D2 to D5 will toggle.

![](https://i.imgur.com/osl4EM9.jpg){width=auto height=auto align=center}


### J.	EEPROM
In this lab, the top 4 converted MSB value of the ADC from the potentiometer input is written on the EEPROM. The value written on the EEPROM is then read and displayed on the LEDs D2-D4. 
	
![](https://i.imgur.com/CP6hlMJ.png){width=auto height=auto align=center}
 
![](https://i.imgur.com/Tnez3ag.jpg){width=auto height=auto align=center}


Note: For the complete step by step guide in making this example, [click here](http://ww1.microchip.com/downloads/en/DeviceDoc/Curiosity%20HPC%20Demo%20Code.zip) and open the Read Me document inside the downloaded folder.
