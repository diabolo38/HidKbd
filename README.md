# HidKbd
STM32 USB HID keyboard std 101 key with optional  led support and volume media control implementation 

It's  a sample SW4STM32 project running on STM32Dicovery F4 board  stared from CubeMX "HID" mouse code, 

Shall work on pmst STm32 with USB 

Note the CubeMX ioc can  still be used (CubeF4 lib v1.14.0 and CubeMX 4.18.0) , but generated hid class files got copied on user src and inc folder to be fully customized. 
The generated usb class files are exclude from build 

#build config 

- Debug 

101 std + volume  control (no led) 

- Debuf_led   

101 std + led handling and volume control  
keyboard leds reflected on Discovery F4 user leds 

#how to use 
No fency user UI was done 

to send test string or volume keys debuf the f/W and place a brake point in main,  modify "send" variable to do what needed.

#TODO
For complete  keyboard support  quite a lot of work to handle key up and down management and sent over usb is still required.

Here simple sequence with delay was used.
