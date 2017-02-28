#include <Arduino.h>

#include "serial.h"

int main(void)
{
	init();

#if defined(USBCON)
	USBDevice.attach();
#endif

	setup();

	for (;;) {
		loop();

		sSim->poll();
		// TODO: MSerial->checkRx();

		if (serialEventRun) serialEventRun();
	}

	return 0;
}

