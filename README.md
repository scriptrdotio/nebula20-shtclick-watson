# nebula20-shtclick-watson
This module is a firmware for [Nebula 2.0](https://www.futureelectronics.com/p/development-tools--development-tool-hardware/neb1dx-02-future-electronics-dev-tools-5094171)
In order to run this you will need to 
1. Install [Wiced Studio 6.2](http://www.cypress.com/products/wiced-software) by [Cypress](http://www.cypress.com)
2. Clone this repository using
```bash
git clone https://github.com/scriptrdotio/nebula20-shtclick-watson.git
```
3. Copy cloned project into your workspace
4. Configure your wifi connection under apps/nebula/bluemix_iot_sensors/wifi_config_dct.h by setting CLIENT_AP_SSID and CLIENT_AP_PASSPHRASE
5. Copy paste your token from scriptr's workspace into apps/nebula/bluemix_iot_sensors/bluemix_dct.c (replace the word <Token> )
6. Generate a new build target by clicking on "new" under "Make Target" and naming it "nebula.scriptr-NEB1DX_02 download download_apps run"
7. Run the build

The code is built using the following
. sht click board sht3x from [click board samples](https://community.cypress.com/docs/DOC-14605) (drivers)
. mqtt code from secure_mqtt, sample code provided by default under apps/snip

## Project Files
1. mqtt.c and mqtt.h, code for connecting over mqtt to the scriptr.io mqtt broker
2. wifi_config_dct.h for wifi configuration (SSID and PASSPHRASE)

