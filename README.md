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
    This is a poor attempt at making a basic enegine managment system based on the esp32 microprocessor.
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
* ESP32 Development Board
* XXX Ω Resisor
* XXX Ω Resisor
* XXX Ω Resisor
* XXX Ω Resisor
* 4x 1N5919BG Zenner Diodes
* 12x 1N4004 Diodes
* 2x 1N5818RLG Diodes
* 2x SP721APP
* 3x 2N2222
* 1x AO9926B
* 1x STP75NS04Z MOSFET
* 1X ISL9V5036P3 IGBT
* 1X LM2940T-5.0/NOPB Voltage Regulator
* 3x TC4424EPA
* 13x 5V LED´s
* XXX uF Capacitor
* XXX uF Capacitor
* XXX uF Capacitor
* XXX uF Capacitor

<!-- ROADMAP -->
## Roadmap

- [x] Hall sensro triggering an interrupt
- [x] Trigger pattern 36-1 on the interrupt
- [ ] DISPLAY
    - [ ] Touch
    - [ ] VE table
    - [ ] IGN table
    - [ ] Main page
- [x] Read + interpolate value from tables
- [ ] ADC
    - [ ] MAP
    - [ ] IAT
    - [ ] CLT
    - [ ] TPS

See the [open issues](https://github.com/EsTaNG9/freertos-ecu/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

João Serrada - [@estang9](https://instagram.com/estang9) - 2192349@my.ipleiria.pt

Alexandre Bernardo - 2213099@my.ipleiria.pt

Project Link: [https://github.com/EsTaNG9/freertos-ecu](https://github.com/EsTaNG9/freertos-ecu)

<p align="right">(<a href="#readme-top">back to top</a>)</p>




<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[product-screenshot]: pics/MOTEC_DA_SHEIN.png
