# Description

This program was used to drive an ADC (ADS8863/ADS8883) and a DAC (MAX5717/MAX5719) to implement digital PID.
For the hardware, see my related repos.

To drive the SPI transactions with accurate timing, I used various timer channels to initiate DMA transfer, instead of using the SPI TX DMA. It could achieve a sampling rate of 400kSPs on MAX5719/AD5542/ADS8863. 

The transfer is currently designed as two 16-bit transfer. For 16 bit ADC/DAC combo, it can be made faster using a single 16bit transfer.

In the PID loop, I implemented a double-integrator control to account for piezo drift (seen in testing). The hardware floating point support also makes PID calculation reasonably fast, but there might be room for further improvement. The integrator and double integrator have both implemented anti-windup. Also I am not sure whether I should use zoh or bilinear when transferring continuous transfer function to discrete transfer function (which result in slightly different integrator form).


