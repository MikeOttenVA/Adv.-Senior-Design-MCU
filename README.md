# GMU CTHD MMS 
GMU - George Mason University, CTHD - Current Total Harmonic Distortion, MaMS - Monitor and Mitigation System
GMU ECE 493 Adv Snr Design NUCLEO F439ZI (MCU) CTHD Monitoring System Code. Uses stm32cubeide for uploading code to MCU.

## Description

Program utilizes an external Analog-to-Digital Converter (ADC - TI ADS1256) that takes 30ksamples/sec from a current transformer (CT).
Look inside ADS1256 datasheet to see how continuous read works.
Order of operations:
Initialize GPIO, SPI, USART, Timers, Interrupts
Setup ADS1256 (30ksamples/sec, channel differential inputs, initialize vars, etc.)
Perform continuous read for 400 samples
Perform auto_calibration on ADS1256
wait 1 ms - Note: I realize order seems backwards, but I ignore first read on external GUI program anyways
Process raw data, "clean" samples, whilst finding max amplitude (this is our current root mean square or IRMS)
Run CTHD algorithm (ignore even harmonics, only process odd harmonics)
  Calculate CTHD after finding up to the 25th odd harmonic
Quick reformat to proprietary communication standard
Send via USART over USB
Repeat Order of operations indefinitely

## Getting Started

### Dependencies

Everything needed should be included in the to src and inc files. The stm32cubeide should be all you need if you want to reuplaod to a F439ZI.
Only thing you will have to do is assign pins using their built in GUI, inside the code you'll find what pins you need.
Recommened to use external GUI running on laptop, link: https://github.com/MikeOttenVA/Adv.-Senior-Design-HTML
* Describe any prerequisites, libraries, OS version, etc., needed before installing program.
* ex. Windows 10

### Installing

Install stm32cubeide
* How/where to download your program
* Any modifications needed to be made to files/folders

### Executing program

After uploading to the NUCLEO F439ZI, upload program to MCU, it will run automatically. 
* How to run the program
* Step-by-step bullets
```
code blocks for commands
```

## Help

There is one known bugs such as not being able to handle current being 0 or false reads (due to transients) from ADC.
```
command to run if program contains helper info
```

## Authors

Contributors names and contact info

Michael Ottenberg
GMU School Email - Mottenb@gmu.edu
LinkedIn - https://www.linkedin.com/in/michael-ottenberg-aa491a18a/

Amirali Jahdi (CTHD Algorithm)
GMU School Email - ajahdi@gmu.edu
LinkedIn - https://www.linkedin.com/in/amirali-j-015b1314a/
## Version History

* 1.0
Initial Release
  One known bugs - Can't handle ADS1256 reporting "0" can be fixed by using ADS1256 current detector to only turn on when current detected

## License

This project is licensed under the [Michael Ottenberg] License - see the LICENSE.md file for details

## Acknowledgments

Inspiration, code snippets, etc.
* [awesome-readme](https://github.com/matiassingers/awesome-readme)
* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2)
* [dbader](https://github.com/dbader/readme-template)
* [zenorocha](https://gist.github.com/zenorocha/4526327)
* [fvcproductions](https://gist.github.com/fvcproductions/1bfc2d4aecb01a834b46)
