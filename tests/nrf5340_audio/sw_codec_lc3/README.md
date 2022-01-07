# LC3 test
This test can only be run on nRF53 APP due to qemu_cortex_m3 target not supporting FPU.

In order to run tests on target:
/zephyr/scripts/sanitycheck --device-testing --device-serial /dev/ttyACM2 --platform nrf5340dk_nrf5340_cpuapp -T . -v

Set the COM port to the APP port of your development kit.
