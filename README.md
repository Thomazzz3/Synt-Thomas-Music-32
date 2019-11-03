# Synt-Thomas-Music-32
I started this as a private project, not intending to publish anything. At some point a thought came to me. "Wouldn't it be nice if there was some kind of framework where I could just start coding a synth in stead of setting up all of the hardware?". From then I decided to make this project public. 

## Get hyped!
The idea is that you can start coding your synth from any point in the proces and even branch off to a completely different path. Maybe want to add an audio input and turn it into a guitar pedal. Add MIDI input or create your own instrument or sequencer. Add cool effects like a delay... Do what you want!

### Let me know what you think
Is this actually usefull for you? Do you have recommendations for me? Let me know. I'm interested since this is the first project I release on Github.
If you are actually going somewhere with this project and sell or publish something I'd also apriciate a mention ofcourse ;) 

### Who made this?
I am Thomas van Beeck form The Netherlands. After finishing a Mechatronics education I started an IT bachelor. Hobby projects keep me motivated. 

## Introduction
"Synt Thomas Music 32" is a syntesizer for the STM32F401. The hardware is programmed with CMSIS only and little to no HAL
I started this project as a hobby project. One of my goals is to make the synthesizer as cheap as possible while making it easy for anyone to get the components. So no complicated board designing and optimal component selection, but of the shelf (aliexpress) components. Maybe eventually I'll design a nice PCB to mount all of the modules and such on.

## Hardware
### Board choices
At first i wanted to make an Arduino nano synth, but this proved to be very complicated for what i wanted to accomplish. With very much hassle maybe this woul've worked out, but it still is a hobby project and it is supposed to be somewhat fun, right? Next came the STM32F103 "bluepill" but it had no DAC or I2S. After some browsing on Aliexpress I stubled upon a dirt cheap STM32F401 board. It has both DAC and I2S. It even has an FPU (floating point unit).

### Audio out
DAC or I2S, which will it be? With the DAC it would be cheap but it would also require more effort to actually connect an audio jack, get the the voltage levels right and such. I2S modules handle this for you and with a little searching I found the PCM5102.


## Software
### DMA I2S
To output audio at 48kHz (will be changed to 41.5 in the future) I've setup a DMA channel to the I2S pripheral with a circular "dubble" buffer. Whoa! hold on, what does this mean? 

Normally if you want to output analog signals, or anything else, you write it at that moment to the peripheral. If you know Arduino you can compare it to AnalogWrite().

#### DMA and circular
DMA, or Direct Memory Acces, allows the STM to handle datastreams without using the processor. In this case it reads from a buffer in memory and transfers it to the I2S periperal, all on its own. Circular means that when the end of the buffer is reached, it will start automatically at the beginning. 

#### Double buffer
Now the buffer is continuously streamed to the I2S peripheral, but we want to change the data to produce different sounds. The problem is that if we change the data in the buffer while it is streaming it will get ugly. Imagine me replacing this text with the next paragraph while you are reading it. It won't make sense anymore. This is where the double buffer comes in. Use 2 buffers where you only change the the one buffer that is not used by the DMA. When the DMA is done with one buffer it will notify us with an interrupt and start reading the buffer we just filled. 
(It is in quotations marks because it is actually one buffer with halfbuffer- and buffercomplete interrupt).

#### Advantage
What have we gained by doing this? Well, the total time of writing a value to the I2S periperal or to a memory location is not that different I suspect (correct me if im wrong). The actual difference comes from handling the interrupt. Interrups are fast but do require time and if you have a samplerate of 41.5 or 48kHz and each sample requires an interrupt it adds up.

### DAC with DMA
I've also setup 4 analog inputs to be connected to potentiometers. Just like the audio output, but in reverse, the ADC values are automatically transferred to a buffer to be read at any time. No double buffer required. The same advantages apply as well.

## Development enviornment
I use Keil uVision 5 for programing and debugging.
For getting the code on the STM32 I use an ST-Link V2 dongle.
When the audio sounds wrong I plug the audio cable in my PC and use Audacity to look at the audio waves.

# Work In Progress
## more to be added
### Sorry for the messy code. In Keil U5 it looks ok, but in the Github viewer the indentation of some things is messed up. To be fixed. At some point I will also neatly split it up in seperate files.
