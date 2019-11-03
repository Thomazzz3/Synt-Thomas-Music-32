# Synt-Thomas-Music-32
"Synt Thomas Music 32" is a syntesizer for the STM32F401. The hardware is programmed with CMSIS only and little to no HAL

I started this project as a hobby project. One of my goals is to make the synthesizer as cheap as possible while making it easy for anyone to get the components. So no complicated board designing and optimal component selection, but of the shelf (aliexpress) components. Maybe eventually I'll design a nice PCB to mount all of the modules and such on.


Board choices
At first i wanted to make an Arduino nano synth, but this proved to be very complicated for what i wanted to accomplish. With very much hassle maybe this woul've worked out, but it still is a hobby project and it is supposed to be somewhat fun, right? Next came the STM32F103 "bluepill" but it had no DAC or I2S. After some browsing on Aliexpress I stubled upon a dirt cheap STM32F401 board. It has both DAC and I2S. It even has an FPU (floating point unit).

Audio out
DAC or I2S, which will it be? With the DAC it would be cheap but it would also require more effort to actually connect an audio jack, get the the voltage levels right and such. I2S modules handle this for you and with a little searching I found the PCM5102.


