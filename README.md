# STM32 Gamepad

This is a game controller made using STM32 Black Pill (STM32F401CCU6). I have used STM32CubeIde for building this project and used the auto code generator for making my work easy.

We can use different cheaper alternatives to the black pill like the standar Blue Pill board (STM32F103C8T6) or any other microcontroller which supports HID.

Before starting this project I would suggest y'all to read <a href="https://docs.kernel.org/hid/hidintro.html" target="_blank">HID Report Discriptors</a> for better understanding of the HID class.


# Prequesites

Make sure to install the <a href="https://www.st.com/en/development-tools/stm32cubeide.html" target="_blank">STM32CubeIde</a> and also the <a href="https://www.st.com/en/development-tools/stsw-link004.htmll" target="_blank">STLink Utility</a> for uploading the programs if you have the cheap STLink programmer. It might and might not work with STM32CubeIDe.

Create a new project and enter the microcontroller eg- STM32F103C8T6

# Setup

We need to set up the STM32CubeIde for our particular microcontroller so that we can use the auto code generator.

It uses the Stm32 HAL library to make programming easy just like Arduino IDE. Eg- Arduino - delay(500);     STM32 - HAL_Delay(500);    

## System Core
1. We need to configure the RCC in the System Core tab to set the internal clock of the microcontroller. We can use the internal clock (HSI), external crystal (HSE) or bypass the internal clock which means that your clock signal goes directly into the clock input it can be from an external clock generator or anything which make the OSC_OUT pin floating.
2. To congifure the debug wires , select the Serial Wire for debugging in the SYS tab.

## Analog

You can use the Timer 3 for the ADC conversion to get triggers every time the Timer resets using the interrupt generated.

If using the DMA make sure to use volatile datatype.
```C++
volatile uint16_t adcVal[4] = {0};
```
1. Select the appropriate ADC pins for your purpose. I'm using the IN0 IN1 IN2 IN3 for the ADC.
2. In the parameters settings select the appropriate resolution for your purpose. I'm using 12bit (15ADC clock cycle)
3. Make sure the scan conversion and continuous conversion mode in enabled below and the EOC flag to be set at the end of all conversions.
4. Set the number of conversions according to the number of channels used.
5. Set the rank at which the channels should be sampled and set the sampled time as the largest you can get if you are using low impedence joysticks like the common joystick module. (480 cycles). Set the channels for the appropriate pins for conversions. eg - IN0 - Channel0 , IN1 - Channel1
6. Go to the DMA setting and add the DMA.
7. Set the mode to be circular and direction should be Peripheral to Memory.
8. Set the data width appropriately according to the resolution of the ADC. eg - 8bit resolution then the data width should be Byte , 12bit then half word.
9. Go the NVIC and select the interrupt for the DMA and ADC.

