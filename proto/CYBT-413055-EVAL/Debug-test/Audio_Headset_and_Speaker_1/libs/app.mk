#
# This file is generated by ModusToolbox during the 'make getlibs' operation
# Any edits to this file will be lost the next time the library manager is run or
# the next time 'make getlibs' is run.
#
bsp-assistant capsense-configurator capsense-tuner config device-configurator config_ezpd ez-pd-configurator modlibs library-manager qspi-configurator seglcd-configurator smartio-configurator config_usbdev usbdev-configurator :
	make -C Audio_Headset_and_Speaker_1 $@

config_bt bt-configurator config_lin lin-configurator :
	$(error $@ configurator cannot be executed at the application level. Run this command from the desired project directory.)

.PHONY: bsp-assistant config_bt bt-configurator capsense-configurator capsense-tuner config device-configurator config_ezpd ez-pd-configurator modlibs library-manager config_lin lin-configurator qspi-configurator seglcd-configurator smartio-configurator config_usbdev usbdev-configurator