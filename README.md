 
# Solar Heat Controller OSS

A open source rework of the MCU firmware for a Resol Diemasol A1.

## Motivation

After changing the gas boiler, I kept the solar thermal panel and adapted it onto the hot water tank of the new boiler. However, I lost the ability to configure the parameters of the Diemasol A1 solar controller built by Resol.

From there, there were two choices:

 * the soft way: reverse the IO protocol to configure the controller (VBus or ModBus)
 * the hard way: rewrite an open source code for the controller, and use the UX to allow for reconfiguration.

Obviously, and for fun and practice, I chose the later.

Surprisingly, the whole project went smoothly and lasted only a weekend. 


## Special thanks

With no sarcasm, this is probably the most interesting part of the project. New friends!

Here is a curated list of projects I found very helping in that journey:

- For some time saved editing chip's header files and link script: https://github.com/sdrshnptl/LPC2148-GCC-compiler
- For a valid flashing solution: https://github.com/capiman/lpc21isp
- For the new fonts: https://github.com/dmadison/LED-Segment-ASCII

## Steps

Going wild reimplementing the source was done through some milestones I'd like to share.

### Hardware discovery and Active parts identification

The very first step was to open the plastic enclosure and check out the whole mess ahead of the software development. 

#### General remarks

The PCB has two planes, the top one holds all the parts and has a ground plane. The bottom one only has wires, and a Vcc plane.

#### MCU

LPC2138/01, it's written on it. Great, not that old. ARM7TDMI-S is boring but not an 8 bits...

#### Pump driving

Simple TRIAC driven by an MOC3041 optocoupler, controlled by a PWM enabled PIN.
Remark: PWM on a pump is not a feature I would recommend. The pump sounds like it's having a hardtime (weird noise). Also, in my nuclear power plant like setup, I drive 2 pump with the solar controller. Therefore, this PWM drive is definitely something I'll get rid of. 

#### Communication 

Well, I've just skipped these items, the initial scope is mainly to PUMP HEAT from the solar panel instead of blinking smuleys on my fridge to display how much watts I'm saving.

#### LCD diplay and buttons

Buttons are wired directly onto the MCU with debouncing capacitor and pull resistor. Nothing unusual.

The LCD is not an off-the-shelf part, It's a custom layout with an external driver (Holtek HT1621B). The main problem is with the mapping of LCD segments with the driver's memory bits. We'll see those details later on. 

#### Temperature sensing

Now the main part of the controller, the data source to take actions! 

Sensing of RTD (platinium based temperature sensors) can be done with various schematic and algorithms. The choice was made to use a non linear method, leveraging a simple resistor bridge to perform a voltage divider.

A single AD7790 ADC with an analog multiplexer frontend (74HC4051) is keeping the board cheap yet extensible. The global accuracy is not bad but has some noise that might require some averaging in the MCU processing.

### LCD driving: first feiled attempt

Using a simple tester, trying to power segment and common pairs and writing the results. This was not working. I've supposed because of the driver, unsoldered it, and constated it was more of my tester based procedure than because of interference of other parts.

But resoldering the driver was not a success. The screen was non functional :(. After few investigation, that was just a capacitor problem.

### Hello JTAG! hello ISP!

First step toward firmware rewrite, the big flashing test! 

Tracing debug ports of the PCB to MCU PIN wasn't that of a hassle. Only a simple tester (again), a paper, a pen, the MCU PIN descriptions and few minutes.

Two connectors are available: 

 * a JTAG port, with the same PIN ordering than the 20 pin ARM jTAG connector on the PCB, without headers 
 * an ISP port, with P0.14, RST, TXD, RXD, VCC and GND. The system's ISP bootloader is reenabled at reset when P0.14 is grounded.  

               ^
  LCD screen  /

        +---+---+
 P0.14  | . | . | Vcc
        +---+---+
  nRST  | . | . | RXD0
        +---+---+
  TXD0  | . | . | GND
        +---+---+

  On PCB battery |
                 v

#### Flashing procedure

 * Ground P0.14
 * Powercycle
 * Unpin P0.14 from the ground, the LCD shall remain blank, as the LPC bootloader does not drive it.
 * Run lpc21isp (tested with https://github.com/Senseg/lpc21isp.git reference 86c3296001d5a3b0b447f0e0fde9ee0825919257)
 * Profit

### Temperature measurement puzzle

After few paper drafting, I found out my assertions on resistor bridge were not complete. I was struggling at trying to determine precisely the voltage accross the RTD and reference resistor series. But that was not the point.

Let's do some simple maths here. Here is the basic schematic

Vcc|----{ REF }--+--[ RTD PT1000 ]------|ground
                 |                      |
                  <---------------------
                        Vsense

Vsense = Vcc * RTD / (REF + RTD)
or
RTD = REF * Vsense / (Vcc - VSense)

However, and that's the point I've missed initially, Vsense is not retrieved directly, instead, the AD7790 returns a quantized value of it. The AD7790 performs the conversion accross a [-Vref;+Vref] range. And the trick is that Vref == Vcc. Which means Vsense can be rewritten from Vcc.

And finally, the formula can totally avoid Vcc and it's accuracies troubles.

RTD = REF * (ADC- 0x8000)/ ( 0x10000 - ADC)

### LCD segments mapping

LCD segment driving was finally done using a simple test program lighting the segments one by one. The results are detailed in the ht1621B interface source of the project. 

### MCU mapping

All pins of the MCU to the various PCB parts were unveiled and noted in the pinmapping.h file

### Wrapping up and code write

The last step was to write the code :) with a simple UX

## Future steps:

A project without TODOs is not a weekend project right?

### External command of water boiler

### IoT bridge

### persist parameters (IAP call)
