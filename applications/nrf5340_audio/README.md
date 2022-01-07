# The nRF5340 Audio project
This readme is intended for nRF5340 Audio developers.
The folder structure contains resources for building the nRF5340 Audio applications (APP core), the NET core is pre-compiled.

Some non-disclosable code is located in a separate repo (lc3).
Developers outside Nordic Semiconductor will be warned when running `west update`
that this repo is unavailable. If you have access run 'west config manifest.group-filter +nrf5340_audio' and then 'west update'. If access is denied, contact local Nordic Semiconductor support or use SBC codec.

# How to build
## Automatic build and flash
In `nrf5340_audio/tools/buildprog`, there's a python script `buildprog.py` which can build app core, and flash both cores. This script must be run from nrf5340_audio/tools in order to find the related files for building and programming.
Run `python buildprog.py -h` for additional usage information.
Please also update the `nrf5340_audio_dk_snr` in `nrf5340_audio/tools/buildprog/nrf5340_audio_dk_devices.json`. The `nrf5340_audio_dk_snr` is the Segger serial number on the nRF5340 Audio DK, and can be found by running `nrfjprog -i`. You can assign a specific nRF5340 Audio DK to be a headset or a gateway, then `buildprog.py` will program the nRF5340 Audio DK according to the settings in `nrf5340_audio_dk_devices.json`. You can also set the channel you wish each headset to be, left is default if no channel is set.

The project can also be built directly using 'west build -b nrf5340_audio_dk_nrf5340_cpuapp' but then it is important to remember to add compile flags for debug/release as well as flags for what device you want to build for; headset or gateway.
Examples of flags:
  -DDEV_HEADSET=ON
  -DDEV_GATEWAY=ON
  -DCMAKE_BUILD_TYPE=DEBUG
  -DCMAKE_BUILD_TYPE=RELEASE

Note that the net core is precompiled and is found in `nrf5340_audio/nrf5340_audio_net`. It is recommended to flash net core directly with nrfjprog:
`nrfjprog --program [hex_file] -f NRF53 --sectorerase --coprocessor CP_NETWORK`

# Running on target
## Running on nRF5340 Audio DK
This project will only run on the nRF5340 Audio DK (PCA10121).

# Projects
All projects reside within `nrf5340_audio\`.

# More information
More information about the application can be found in `doc/nRF5340_Audio_DK_user_guide.rst`
