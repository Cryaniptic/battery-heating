# PAST-Avionics-05

This project is a testbed for a battery heating system designed to keep lithium ion batteries from freezing over in a space enviroment.

The code was generated in STM32cubeMX, and is built using arm-none-eabi-gcc. 

Ideally I would jlink or serial wire debug over an ST-link, however due to unknown reasons, this option is not working and so I have to make do with uploading over USART with an FTDI dongle :/ 

If another contributor takes a shot at this you may need to tweak the full_build.sh script, as this was made in and for linux, using some CLI utilities to streamline development.

Full list of tools used to build, upload and communicate
 - Make
 - arm-none-eabi-gcc + arm-none-eabi-newlib (for printing unitilites)
 - STM32cubeProgrammerCLI
 - Serust (just for serial monitor)

![notepad](https://cyber.dabamos.de/88x31/vim.vialle.love.anim.gif) ![coffee](https://cyber.dabamos.de/88x31/coffee.gif)

