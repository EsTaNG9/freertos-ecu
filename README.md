<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https:///github.com/EsTaNG9/freertos-ecu">
    <img src="pics/auto_jpg.jpg" alt="Logo" width="1222" height="134">
  </a>
   <h3></h3>
  <div align="center">
  <a href="https://github.com/https://github.com/EsTaNG9/freertos-ecu">
    <img src="pics/MOTEC_DA_SHEIN.png" alt="Logo" width="250" height="250">
  </a>
</div>

  <h3 align="center">FREERTOS ECU</h3>

  <p align="center">
    This is a poor attempt at developing a basic enegine managment system based on the ESP32 microprocessor.
    <br />
    <br />
    <a href="https://github.com/EsTaNG9/freertos-ecu">View Demo (TO DO)</a>
    ·
    <a href="https://github.com/EsTaNG9/freertos-ecu/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    ·
    <a href="https://github.com/EsTaNG9/freertos-ecu/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a>
  </p>
</div>

<!-- ABOUT THE PROJECT -->
## About The Project

Our project consist on developing a simple engine management system for a 1 cylinder 4 or 2 stroke internal combustion engine.
The hear of this project was to develop a way to trigger the sparkplug at the correct time for each rotation of the engine. 

Limitações:
* The ESP32 only has 4 timers
* An PIC or ARM based microcontroller would be a better application, just because normally they have more timers available
* Falta de tempo para melhor desenvolver o projeto :smile:

Despite our efforts, the deadline came in quicly and unfortunately we didnt have time to finnish this project the our fullest capability.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Materials

This is an example of how to list things you need to use the software and how to install them.
*	1x ESP32
*	1x Hall Sensor
*	4x Potentiometers (Emulate de CLT, IAT, MAP and TPS sensors)
*	1x ILI9341 320x240 SPI TFT LCD Display with Touch Sensor
*	3x LED´s + associated Resistors (Emulate Ignition Coil, Fuel Injector and Fuel Pump)
*	2x ESP32 Dev Kits (One as the controller the other has the Hall Sensor emulator)
*	Breadboard

<!-- ROADMAP -->
## Roadmap

- [x] Hall sensor triggering an interrupt
- [x] Trigger pattern 36-1 on the interrupt
- [x] DISPLAY
    - [x] Touch
    - [x] VE table
    - [x] IGN table
    - [x] Main page
- [x] Read + interpolate value from tables
- [x] ADC
    - [x] MAP
    - [x] IAT
    - [x] CLT
    - [x] TPS

See the [open issues](https://github.com/EsTaNG9/freertos-ecu/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

João Serrada - [@estang9](https://instagram.com/estang9) - 2192349@my.ipleiria.pt

Alexandre Bernardo - 2213099@my.ipleiria.pt

Project Link: [https://github.com/EsTaNG9/freertos-ecu](https://github.com/EsTaNG9/freertos-ecu)

Youtube Video Link: [https://youtube.com/shorts/LGoChy05eWk](https://youtube.com/shorts/LGoChy05eWk)

<p align="right">(<a href="#readme-top">back to top</a>)</p>




<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[product-screenshot]: pics/MOTEC_DA_SHEIN.png
