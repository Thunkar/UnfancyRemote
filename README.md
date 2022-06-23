# The Unfancy Remote

A cheap, open, modular and reliable DIY remote with a focus on RF performance. 

**WARNING** : The software and CAD files included in this repo are in a **BETA STAGE** at the moment and **SHOULD BE USED CAREFULLY**. I am not responsible for **ANY** injures or damages you may incur as a result of using the HW or SW described in this project.

# Design considerations

* Unfancy. I use other devices for my telemetry, so no UART (even though could be easily added in the future) or OLED screens. PPM for the win.
* DIY friendly and modular. 
* Stellar RF performance is a must. The whole idea for this project came after in the FPV drone space [this project](https://www.expresslrs.org/2.0/) became super popular. It uses LoRa transcievers to achieve great range and link speeds, using a proprietary modulation that is specially robust against interference. It can also work on 433MHz/868MHz/2.4GHz bands depending on the specific module.
* Multiple shells to accomodate thumbwheels, triggers, Qi charging, battery sizes...
* Community driven

# Building the code

It has been tested on the latest (at the time of writing, 1.8.19) Arduino IDE version, using the fantastic [SX12XX Library](https://github.com/StuartsProjects/SX12XX-LoRa) by StuartsProjects.

# Documentation

1) [Ordering](./docs/ordering.md)
2) [Assembly](./docs/assembly.md)

# Contributing

All contributions must go through a process of review and validation via a Pull Request before being integrated into the codebase. If you wish to contribute, please fork the repo, perform your modifications **TEST THEM** and submit a PR for review. The place to discuss bugs, new features or suggestions is the [esk8 news forum](https://forum.esk8.news/t/the-unfancy-remote-wip-diy-reliable-remote).

# License 
```
            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
                    Version 2, December 2004
 
 Copyright (C) 2022 Gregorio Juliana Quirós
  Madrid, Spain
 Everyone is permitted to copy and distribute verbatim or modified
 copies of this license document, and changing it is allowed as long
 as the name is changed.
 
            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION
 
  0. You just DO WHAT THE FUCK YOU WANT TO.
  ```