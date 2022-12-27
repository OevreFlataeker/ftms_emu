#!/bin/bash

NRFUTIL=$(which nrfutil)
if [ -z ${NRFUTIL} ]; then
	echo "nrfutil not found. Trying to pip it"
	pip3 install nrfutil
	NRFUTIL=$(which nrfutil)
	if [ -z ${NRFUTIL} ]; then
		echo "nrfutil still not found. Please fix"
		exit 1
	fi
fi

VERSION=$(grep '#define VERSION_NR' main.c | cut -d '"' -f 2 | cut -d "v" -f 2)
if [ -z ${VERSION} ]; then
	echo "VERSION_NR define missing from main.c. Please check"
	exit 1
fi

echo Building package for version ${VERSION}
nrfutil pkg generate --application pca10040/s132/ses/Output/Debug/Exe/ble_app_cps_relay_pca10040_s132.hex --key-file nrf_priv.pem --application-version 7 --app-boot-validation NO_VALIDATION --hw-version 52 --sd-req 0x0101 dfu_zwift_version${VERSION}_debug.zip
nrfutil pkg generate --application pca10040/s132/ses/Output/Release/Exe/ble_app_cps_relay_pca10040_s132.hex --key-file nrf_priv.pem --application-version 7 --app-boot-validation NO_VALIDATION --hw-version 52 --sd-req 0x0101 dfu_zwift_version${VERSION}_release.zip
