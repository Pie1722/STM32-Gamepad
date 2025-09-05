# STM32 Gamepad

This is a game controller made using STM32 Black Pill (STM32F401CCU6). I have used STM32CubeIde for building this project and used the auto code generator for making my work easy.

We can use different cheaper alternatives to the black pill like the standar Blue Pill board (STM32F103C8T6) or any other microcontroller which supports HID.

Before starting this project I would suggest y'all to read <a href="https://docs.kernel.org/hid/hidintro.html" target="_blank">HID Report Discriptors</a> for better understanding of the HID class.

## Contents
- [Setup](#Setup)
- [Code](#Codes)

# Prequesites

Make sure to install the <a href="https://www.st.com/en/development-tools/stm32cubeide.html" target="_blank">STM32CubeIde</a> and also the <a href="https://www.st.com/en/development-tools/stsw-link004.htmll" target="_blank">STLink Utility</a> for uploading the programs if you have the cheap STLink programmer. It might and might not work with STM32CubeIDe.

Create a new project and enter the microcontroller eg- STM32F103C8T6

# Setup

We need to set up the STM32CubeIde for our particular microcontroller so that we can use the auto code generator.

It uses the Stm32 HAL library to make programming easy just like Arduino IDE. Eg- Arduino - delay(500);     STM32 - HAL_Delay(500);    

## Table of Contents
- [System Core](#1-system-core)
- [Analog](#2-analog)
- [Connectivity](#3-connectivity)
- [Middlewares](#4-middlewares)
- [Clock Config](#5-clock-config)
- [Pin Config](#6-pin-config)

## 1. System Core
1. We need to configure the RCC in the System Core tab to set the internal clock of the microcontroller. We can use the internal clock (HSI), external crystal (HSE) or bypass the internal clock which means that your clock signal goes directly into the clock input it can be from an external clock generator or anything which make the OSC_OUT pin floating.
2. To congifure the debug wires , select the **Serial Wire** for debugging in the SYS tab.

## 2. Analog

You can use the Timer 3 for the ADC conversion to get triggers every time the Timer resets using the interrupt generated.

If using the DMA make sure to use *volatile* datatype.
```C++
volatile uint16_t adcVal[4] = {0};
```
1. Select the appropriate ADC pins for your purpose. I'm using the IN0 IN1 IN2 IN3 for the ADC.
2. In the parameters settings select the appropriate resolution for your purpose. I'm using 12bit (15ADC clock cycle)
3. Make sure the scan conversion and continuous conversion mode in **enabled** below and the EOC flag to be set at the end of all conversions.
4. Set the number of conversions according to the number of channels used.
5. Set the rank at which the channels should be sampled and set the sampled time as the largest you can get if you are using low impedence joysticks like the common joystick module. (480 cycles). Set the channels for the appropriate pins for conversions. eg - IN0 - Channel0 , IN1 - Channel1
7. Go to the DMA setting and add the DMA.
8. Set the mode to be **circular** and direction should be **Peripheral to Memory**.
9. Set the data width appropriately according to the resolution of the ADC. eg - **8bit** resolution then the data width should be **Byte** , **12bit** then **half word**.
10. Go the NVIC and select the interrupt for the DMA and ADC.

## 3. Connectivity

1. Select the **USB_OTG_FS** and select the Mode to be device only as we are only using it as a game controller.
2. Select the USB_OTG_FS Global Interrupt

## 4. Middlewares

1. Set the class for USB to be **HID (Human Interface Device)**.
2. In the device descriptor you can change the various manufacturing string and the name to be displayed on your operating system. Dont change the VID and PID if you dont have proper permissions and documentation.

## 5. Clock Config

1. If you go to the clock configuration tab you might see that there is problem in the clock congiuration as the USB FS needs 48MHz to work properly as a standar HID device.
2. You can press on the auto clock solver or manually adjust the PLL values to divide the clock cycle to make it 48MHz
3. Make sure to select the HSE clock and then press the auto clock resolver.

## 6. Pin Config

Set the desired pin to be GPIO Input or Output. If you want to make the pins INPUT_PULLUP, go the the System Core tab and then go the the GPIO and set each pins to be INPUT_PULLUP or INPUT_PULLDOWN.

After that press ctrl+S to save the file and use the auto code generator and select OK. This will make sure that all the necessary files and code are generated.

# Codes
<details>
  <summary>DINPUT</summary>
We need to make some changes to the codes created by the STM32CubeIDE to make it work for our custom hardware.

## Table of Contents
- [Middlewares Source](#1-Changes-to-Middlewares-Source)
- [USBD_COMPOSITE](#a-USBD_COMPOSITE)
- [HID REPORT](#b-HID-REPORT)
- [Middlewares Headers](#2-Changes-to-Middlewares-Headers)


## 1. Changes to Middlewares Source

Go to Middlewares\ST\STM32_USB_Device_Library\Class\HID\Src\usbd_hid.c and open the file in the project window.

This program is created by the IDE for a USB mouse, so we need to change the report according to our specific hardware and needs.

### a. USBD_COMPOSITE
```C
#ifndef USE_USBD_COMPOSITE
/* USB HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09,                                               /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                        /* bDescriptorType: Configuration */
  USB_HID_CONFIG_DESC_SIZ,                            /* wTotalLength: Bytes returned */
  0x00,
  0x01,                                               /* bNumInterfaces: 1 interface */
  0x01,                                               /* bConfigurationValue: Configuration value */
  0x00,                                               /* iConfiguration: Index of string descriptor
                                                         describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xE0,                                               /* bmAttributes: Bus Powered according to user configuration */
#else
  0xA0,                                               /* bmAttributes: Bus Powered according to user configuration */
#endif /* USBD_SELF_POWERED */
  USBD_MAX_POWER,                                     /* MaxPower (mA) */

  /************** Descriptor of Joystick Mouse interface ****************/
  /* 09 */
  0x09,                                               /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                            /* bDescriptorType: Interface descriptor type */
  0x00,                                               /* bInterfaceNumber: Number of Interface */
  0x00,                                               /* bAlternateSetting: Alternate setting */
  0x01,                                               /* bNumEndpoints */
  0x03,                                               /* bInterfaceClass: HID */
  0x00,                                               /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x00,                                               /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0,                                                  /* iInterface: Index of string descriptor */
  /******************** Descriptor of Joystick Mouse HID ********************/
  /* 18 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_MOUSE_REPORT_DESC_SIZE,                         /* wItemLength: Total length of Report descriptor */
  0x00,
  /******************** Descriptor of Mouse endpoint ********************/
  /* 27 */
  0x07,                                               /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                             /* bDescriptorType:*/

  HID_EPIN_ADDR,                                      /* bEndpointAddress: Endpoint Address (IN) */
  0x03,                                               /* bmAttributes: Interrupt endpoint */
  HID_EPIN_SIZE,                                      /* wMaxPacketSize: 4 Bytes max */
  0x00,
  HID_FS_BINTERVAL,                                   /* bInterval: Polling Interval */
  /* 34 */
};
#endif /* USE_USBD_COMPOSITE  */
```
We can see here in the **Descriptor of Joystick Mouse Interface** to set the **bInterfaceSubClass** : *1=BOOT, 0=no boot* and **nInterfaceProtocol** : *0=none, 1=keyboard, 2=mouse*

We dont need the device to show up at boot time, it is useful for mouse when we need to make changes in the bios but for our game controller its not required.

We need to make the interface protocol as **0x00** as its a custom gamepad.

### b. HID REPORT

Now we need to make sure to create our own custom hid report, so I've created this HID report for 4 axis joysticks, 12 buttons and 1 Hat switch which is 4 Dpads

```C
__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE] __ALIGN_END =
{
	0x05, 0x01,        // Usage Page (Generic Desktop)
	0x09, 0x05,        // Usage (Game Pad)
	0xA1, 0x01,        // Collection (Application)

	// Buttons (12 x 1-bit)
	0x05, 0x09,        //   Usage Page (Button)
	0x19, 0x01,        //   Usage Minimum (Button 1)
	0x29, 0x0C,        //   Usage Maximum (Button 12)
	0x15, 0x00,        //   Logical Minimum (0)
	0x25, 0x01,        //   Logical Maximum (1)
	0x95, 0x0C,        //   Report Count (12)
	0x75, 0x01,        //   Report Size (1)
	0x81, 0x02,        //   Input (Data,Var,Abs)

	// Padding to byte-align after 12 buttons (4 bits)
	0x95, 0x01,        //   Report Count (1)
	0x75, 0x04,        //   Report Size (4)
	0x81, 0x03,        //   Input (Const,Var,Abs) — Padding

	// ---------------- 1 Hat Switch ----------------
	0x05, 0x01,        //   Usage Page (Generic Desktop)
	0x09, 0x39,        //   Usage (Hat switch)
	0x15, 0x00,        //   Logical Minimum (0)
	0x25, 0x07,        //   Logical Maximum (7)
	0x35, 0x00,        //   Physical Minimum (0)
	0x46, 0x3B, 0x01,  //   Physical Maximum (315)
	0x65, 0x14,        //   Unit (Eng Rot:Angular Pos)
	0x75, 0x08,        //   Report Size (8)
	0x95, 0x01,        //   Report Count (1)
	0x81, 0x42,        //   Input (Data,Var,Abs,Null)

	// Axes (X, Y, Rx, Ry)
	0x05, 0x01,        //   Usage Page (Generic Desktop)
	0x09, 0x30,        //   Usage (X)
	0x09, 0x31,        //   Usage (Y)
	0x09, 0x33,        //   Usage (Rx)
	0x09, 0x34,        //   Usage (Ry)
	0x16, 0x00, 0x80,  //   Logical Minimum (-32768)
	0x26, 0xFF, 0x7F,  //   Logical Maximum (+32768)
	0x75, 0x10,        //   Report Size (16)
	0x95, 0x04,        //   Report Count (4)
 	0x81, 0x02,        //   Input (Data,Var,Abs)

	0xC0               // End Collection
};
```
The size of this whole report is 72 Bytes and this report is pretty much self explainatory with comments.

## 2. Changes to Middlewares Headers

Go to Middlewares\ST\STM32_USB_Device_Library\Class\HID\Inc and open the code in the project window

As we have made changes to the source files we need to make sure the headers match perfectly. If anything is not properly defined the whole stack will be broken and your computer wont recognize the device as a vaid HID.

```C
#ifndef HID_EPIN_ADDR
#define HID_EPIN_ADDR                              0x81U
#endif /* HID_EPIN_ADDR */
#define HID_EPIN_SIZE                              0x0BU

#define USB_HID_CONFIG_DESC_SIZ                    34U
#define USB_HID_DESC_SIZ                           9U
#define HID_MOUSE_REPORT_DESC_SIZE                 72U
```
Now we need to change the descriptor size with the actuall size which is 72 Bytes

**HID_EPIN_SIZE** is the size of the actual data packet you send to the host. In our case: 0x0B → 11 bytes (88 bits). We can calculate it like this:

| Section     | Bits |
| ----------- | ---- |
| Buttons     | 12   |
| Padding     | 4    |
| Hat switch  | 8    |
| Axes (4×16) | 64   |
| **Total**   | 88   |


## 3. Changes to main.c

1. We need to create a struct to send the report

```c
typedef struct __attribute__((packed)) {
    uint16_t buttons;
    uint8_t hat;
    int16_t x, y, rx, ry;
} joystickReport;
```


2. Initialize an array for storing ADC data. **void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){}** This function handles the ADC DMA. We can keep the function empty but it needs to be present for processing by the DMA

```c
volatile uint16_t adcVal[4] = {0};
volatile uint8_t isADCFinished = 0;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        isADCFinished = 1;
    }
}
```


3. Create functions to get HAT switches and all the other reports to send data through USB

```C
uint8_t get_hat_value(void)
{
	uint8_t up    = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
	uint8_t down  = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	uint8_t left  = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	uint8_t right = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

    if (up && right)   return 1;  // Up-Right
    if (right && down) return 3;  // Down-Right
    if (down && left)  return 5;  // Down-Left
    if (left && up)    return 7;  // Up-Left
    if (up)            return 0;  // Up
    if (right)         return 2;  // Right
    if (down)          return 4;  // Down
    if (left)          return 6;  // Left

    return 0x0F;  // Neutral
}

void send_gamepad_report(void)
{
    joystickReport report = {0};

    // ---- Buttons: Read 12 GPIOs (example mapping) ----
    report.buttons |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) ? (1 << 2) : 0;		//X
    report.buttons |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) ? (1 << 3) : 0;		//Y
    report.buttons |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) ? (1 << 0) : 0;   	//A
    report.buttons |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) ? (1 << 1) : 0;		//B
    report.buttons |= !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)  ? (1 << 4) : 0;		// lt1
    report.buttons |= !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)  ? (1 << 6) : 0;		// lt2
    report.buttons |= !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) ? (1 << 5) : 0;		//rt1
    report.buttons |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)  ? (1 << 8) : 0;		// Enter
    report.buttons |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)  ? (1 << 10) : 0;	// lj / scl
    report.buttons |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)  ? (1 << 9) : 0;		// rj / sda
    report.buttons |= !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)  ? (1 << 7) : 0;		// Back
    report.buttons |= !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)  ? (1 << 11) : 0;	// rt2

    // ---- Hat switch ----
    report.hat = get_hat_value();

    // ---- Axes: ADC static data ----
    report.x  = adcVal[0];
    report.y  = adcVal[1];
    report.rx = adcVal[2];
    report.ry = adcVal[3];

    // ---- Send 11-byte report ----
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));
}
```

4. We need to start the DMA in the ***main()***

```C
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcVal, 4);
```
Then just call the ***send_gamepad_report()*** to continuously send values through USB


5. Make sure to check the ***MX_ADC1_Init()*** to see if the ADC is properly assigned to each channels with their proper ranks

```C
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
```


6. Make sure to check the ***MX_GPIO_Init()*** for the correct pin definitons and pullups or pulldowns

```C
/*Configure GPIO pins : PA4 PA8 PA9 PA10
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB3 PB4 PB5 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```


7. Also check the *SystemClock_Config()* for proper clock definitons. Here I am using HSE.

```C
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
```
<details>

<details>
  <summary>XINPUT</summary>

<details>
