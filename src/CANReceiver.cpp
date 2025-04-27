// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <Arduino.h>
#include <CAN.h>

static void ledBlinker(void *parameter) {

	pinMode(GPIO_NUM_2, OUTPUT);
	while (true) {
		digitalWrite(GPIO_NUM_2, HIGH);
		delay(500);
		digitalWrite(GPIO_NUM_2, LOW);
		delay(500);
	}
}

static void send_daly_frame() {
	//CAN.beginPacket(0x18, -1, true);
	//CAN.write(0x90);
	//CAN.write(0x01);
	//CAN.write(0x40);
	//if (!CAN.endPacket()) {
	//	Serial.printf("TX failed\n");
	//}

	CAN.beginExtendedPacket(0x18900140, -1, true);
	CAN.endPacket();
}

void send_daly_frame(int interval) {
	static unsigned long last_time = 0;
	if (millis() - last_time < interval) {
		return;
	}
	last_time = millis();
	send_daly_frame();
	Serial.printf("Sent\n");
}

void setup() {
	xTaskCreate(ledBlinker, "ledBlinker", 2048, NULL, 1, NULL);

	Serial.begin(115200);
    while (!Serial);


    Serial.println("CAN Receiver");
    CAN.setPins(GPIO_NUM_12, GPIO_NUM_13);
    if (!CAN.begin(250E3)) {
        Serial.println("Starting CAN failed!");
        while (1);
    }


}

void loop() {
    // try to parse packet
    int packetSize = CAN.parsePacket();

    if (packetSize) {
		// received a packet
		Serial.print("Received ");

		if (CAN.packetExtended()) {
			Serial.print("extended ");
		}

		if (CAN.packetRtr()) {
			// Remote transmission request, packet contains no data
			Serial.print("RTR ");
		}

		Serial.print("packet with id 0x");
		Serial.print(CAN.packetId(), HEX);

		if (CAN.packetRtr()) {
			Serial.print(" and requested length ");
			Serial.println(CAN.packetDlc());
		} else {
			Serial.print(" and length ");
			Serial.println(packetSize);

			// only print packet data for non-RTR packets

			while (CAN.available()) {
				Serial.print((char)CAN.read());
			}
			Serial.println();
		}

		Serial.println();
    }

	send_daly_frame(1000);
}

