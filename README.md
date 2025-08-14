# STM32 Gamepad

This is a game controller made using STM32 Black Pill (STM32F401CCU6). I have used STM32CubeIde for building this project and used the auto code generator for making my work easy.

We can use different cheaper alternatives to the black pill like the standar Blue Pill board (STM32F103C8T6) or any other microcontroller which supports HID.

Before starting this project I would suggest y'all to read <a href="https://docs.kernel.org/hid/hidintro.html" target="_blank">HID Report Discriptors</a> for better understanding of the HID class.


# Prequesites

Make sure to install the [STM32CubeIde] (https://www.st.com/en/development-tools/stm32cubeide.html) and also the [STLink Utility] (https://www.st.com/en/development-tools/stsw-link004.html) for uploading the programs if you have the cheap STLink programmer. It might and might not work with STM32CubeIDe.

Create a new project and enter the microcontroller eg- STM32F103C8T6


```C++
# This is a Python program
def say_hello():
    print("Hello, World!")

say_hello()
