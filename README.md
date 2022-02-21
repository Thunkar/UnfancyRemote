# The Unfancy Remote

A cheap, open, modular and reliable DIY remote with a focus on RF performance. 

**WARNING** : The software and CAD files included in this repo are in a **VERY EARLY ALPHA STAGE** at the moment and under **NO CIRCUMSTANCES** should be used on an actual ESK8 or other PEV. You have been warned, this is just a **MANUFACTURING PATHFINDER**

# Design considerations

* Unfancy. I use other devices for my telemetry, so no UART (even though could be easily added in the future) or OLED screens. PPM for the win.
* DIY friendly and modular. 
* Stellar RF performance is a must. The whole idea for this project came after in the FPV drone space [this project](https://www.expresslrs.org/2.0/) became super popular. It uses LoRa transcievers to achieve great range and link speeds, using a proprietary modulation that is specially robust against interference. It can also work on 433MHz/868MHz/2.4GHz bands depending on the specific module.
* Thumbwheel style at first, with the option to change throttle actuators.

# Building the code

It has been tested on the latest (at the time of writing, 1.8.19) Arduino IDE version, using the fantastic [SX12XX Library](https://github.com/StuartsProjects/SX12XX-LoRa) by StuartsProjects. The initial alpha coded is based on examples 35_ and 36_ (RemoteControl)

# Hardware

The initial version uses an Arduino Nano (level conversion from 5 to 3.3V required) and an [Ebyte SX1280 E28-2G4M12S LoRa module](https://es.aliexpress.com/item/32835072788.html?spm=a2g0o.productlist.0.0.26ac22d4TPH82D&algo_pvid=cda1bf4e-a2a6-409d-af58-f4f6ba2c65af&aem_p4p_detail=20220220064512753396458493180117278326&algo_exp_id=cda1bf4e-a2a6-409d-af58-f4f6ba2c65af-4&pdp_ext_f=%7B%22sku_id%22%3A%2265080145381%22%7D&pdp_pi=-1%3B7.77%3B-1%3B-1%40salePrice%3BEUR%3Bsearch-mainSearch) hooked up to the SPI bus (MISO, MOSI, NSS, DIO1, NRESET, RFBUSY) for the transmitter and an Arduino Pro mini (no level conversion required) for the receiver. The [hall sensor](https://es.aliexpress.com/item/1005002840783439.html?spm=a2g0o.productlist.0.0.49976a67WqnJ3C&algo_pvid=f27dbc70-e34d-4938-9a28-d97ef4192d3f&aem_p4p_detail=20220218155844567359948235000110942091&algo_exp_id=f27dbc70-e34d-4938-9a28-d97ef4192d3f-0&pdp_ext_f=%7B%22sku_id%22%3A%2212000022416142840%22%7D&pdp_pi=-1%3B0.87%3B-1%3B-1%40salePrice%3BEUR%3Bsearch-mainSearch) must be linear and requires calibration values hardcoded in the software at the moment.

# License 
```
            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
                    Version 2, December 2004
 
 Copyright (C) 2004 Sam Hocevar
  14 rue de Plaisance, 75014 Paris, France
 Everyone is permitted to copy and distribute verbatim or modified
 copies of this license document, and changing it is allowed as long
 as the name is changed.
 
            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION
 
  0. You just DO WHAT THE FUCK YOU WANT TO.
  ```