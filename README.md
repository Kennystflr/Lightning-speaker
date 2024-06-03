# Lightning-speaker
Project of Kenny Saint Fleur, Mathys Minsac, Lavallee Alexandre, Florian Bonnet--Galand and Victor Morel. 1st Year ENSEA students.
Design of the electronic board for a speaker.
Using a STM32 microcontroller, audio codec and two class D amplifiers.

The purpose of this project is to design the electronic of a speaker. More specifically the goal is to take an entry signal from Jack 6.35 mm connector and to have sound coming out of a stereo loudspeaker. 
We could also have a parametric eq using 2 encoders 1 for the gain and 1 for the frequency we would be changing.
We also want to have the system working on battery with Usb-C power delivery to charge-up the battery and power up the system even if there is no battery connected.

There is a schematic of the projects detailed review on this repository's root. ![projectSchematics](https://github.com/Kennystflr/Lightning-speaker/projectSchematics)

Basically, we would be calcultating coeficients for the SGTL5000 audio codec with the STM32 from the encoder's command. We can command up to 4 different filters with this method.
Then we would apply filters to the audio codec by sending the filter's coeficients through I2C communication protocol to the SGTL5000.
About the signal itself, it would enter into the audio codec, pass through the filters, be transmitted to the STM32 using I2S communication protocol and then would go through the Half Bridge Class D audio amplifer.
To charge up the battery, the 9V power supply (negociated with a source by the STM32) goes through a BMS who can bypass the battery in case it is dead or missing.

An issue we have met doing this was that the Half bridge was commuting between a positive voltage and ground which was a bad for the loudspeakears as the load. To resolve that we had to cut the rms continuous voltage 
using a genuinely calculated capacity.


As IC references we have :
-Microprocessor : STM32G491CET6
-Audio codec :SGTL5000XNLA3
-Battery Management System : BQ25306RTET
- USB-C protection IC : TCPP01-M12
- USB protection IC : USBLC6-2SC6
- Mosfet driver : UCC27325DR
- LDO 5V : AZ1117-5.0
- LDO 3.3V : BU33SD5WG-TR
- LDO 1.8V : BU18SD5WG-TRe


Unfortunately, the project wouldn't work in the end and we weren't able to debug it before the due time.
So we came up with a lifeboat plan of showing off our secondary board :

Project Specifications for the Replacement Project (aims to test the different codes made during the project and the PCB secondary board):

To test the LEDs, the potentiometer, and the encoder button, the LEDs are lit one by one, faster or slower depending on the value of the potentiometer.
The button on encoder 1 allows the LEDs to light up in the opposite direction.

To test the RGB (even though we only use 2 out of 3 LEDs) and the rotation of the encoder, the brightness of the red LED of encoder 2 is increased while the brightness of the blue LED is decreased when encoder 1 is rotated in one direction and vice versa when rotated in the other direction.

As a bonus, pressing the button on encoder 1 toggles the green LED of encoder 1 on or off.

*There is an demonstration video on the repository's root* : ![FinalProjetDemo](https://github.com/Kennystflr/Lightning-speaker/blob/main/FinalProjetDemo.mp4)


To push the project further we could have :
- had a bluetooth connection with a phone app to control the features.
- used the USB-C port's FS USB connected to the STM32 to have had sound going into the loudspeaker from an extern device as a computer or even a smartphone...




