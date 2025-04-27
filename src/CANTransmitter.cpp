// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.



#include <Arduino.h>
#include <CAN.h>

void setup() {
	Serial.begin(115200);
	while (!Serial);

	pinMode(GPIO_NUM_2, OUTPUT);

	Serial.println("CAN Sender");
	CAN.setPins(GPIO_NUM_12, GPIO_NUM_13);
	if (!CAN.begin(250E3)) {
		Serial.println("Starting CAN failed!");
		while (1);
	}
}

void loop() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
	Serial.print("Sending packet ... ");

	CAN.beginPacket(0x12);
	CAN.write('h');
	CAN.write('e');
	CAN.write('l');
	CAN.write('l');
	CAN.write('o');
	if (!CAN.endPacket()) {
		Serial.printf("TX failed\n");
	}

	Serial.println("done");
	digitalWrite(GPIO_NUM_2, !digitalRead(GPIO_NUM_2));

	delay(1000);

	// send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
	Serial.print("Sending extended packet ... ");

	CAN.beginExtendedPacket(0xabcdef);
	CAN.write('w');
	CAN.write('o');
	CAN.write('r');
	CAN.write('l');
	CAN.write('d');
	CAN.endPacket();

	Serial.println("done");

	digitalWrite(GPIO_NUM_2, !digitalRead(GPIO_NUM_2));
	delay(1000);
}

