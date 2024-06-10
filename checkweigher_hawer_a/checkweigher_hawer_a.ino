#include <SoftwareSerial.h>

#define MAX485_RE_NEG_PIN       2
#define MAX485_DE_PIN           3
#define SERIAL_RX_PIN           8
#define SERIAL_TX_PIN           9

#define BAUD_RATE               9600
#define READ_DELAY_MS           10

#define STX                     0x02
#define ETX                     0x03

float weight;
bool weightError;

SoftwareSerial weightSerial(SERIAL_RX_PIN, SERIAL_TX_PIN);

bool readWeightData();
void preTransmission();
void postTransmission();

void setup() {
    pinMode(MAX485_RE_NEG_PIN, OUTPUT);
    pinMode(MAX485_DE_PIN, OUTPUT);

    digitalWrite(MAX485_RE_NEG_PIN, LOW);
    digitalWrite(MAX485_DE_PIN, LOW);

    Serial.begin(BAUD_RATE);
    weightSerial.begin(BAUD_RATE);

    Serial.println(F("Setup complete."));
}

void loop() {
    unsigned long startTime = millis();
    
    weightError = !readWeightData();
    
    unsigned long endTime = millis();
    unsigned long loopTime = endTime - startTime;
    
    if (weightError) {
        Serial.println(F("Error reading weight."));
    } else {
        Serial.print(F("Weight: "));
        Serial.println(weight);
    }
    Serial.print(F("Loop time (ms): "));
    Serial.println(loopTime);

    delay(READ_DELAY_MS);
}

bool readWeightData() {
    while (weightSerial.available()) {
        weightSerial.read();  // Flush the buffer
    }

    while (true) {
        if (weightSerial.available() >= 12) {
            char startByte = weightSerial.read();
            if (startByte != STX) continue;  // Synchronize to start byte

            char sign = weightSerial.read();
            char weightString[7];  // Include space for null terminator
            for (int i = 0; i < 6; i++) {
                weightString[i] = weightSerial.read();
            }
            weightString[6] = '\0';
            
            char decimalScale = weightSerial.read();
            char xorHigh = weightSerial.read();
            char xorLow = weightSerial.read();
            char endByte = weightSerial.read();
            if (endByte != ETX) continue;  // Ensure correct end byte

            // Calculate XOR checksum
            char checksum = sign;
            for (int i = 0; i < 6; i++) {
                checksum ^= weightString[i];
            }
            checksum ^= decimalScale;

            char calculatedXorHigh = (checksum >> 4) & 0x0F;
            char calculatedXorLow = checksum & 0x0F;

            if (calculatedXorHigh != (xorHigh & 0x0F) || calculatedXorLow != (xorLow & 0x0F)) {
                continue;  // Retry on checksum error
            }

            weight = atof(weightString) / 100;

            if (decimalScale > 0 && decimalScale <= 4) {
                weight /= pow(10, decimalScale);
            }

            if (sign == '-') {
                weight = -weight;
            }

            return true;
        }
    }
    return false;
}

void preTransmission() {
    digitalWrite(MAX485_RE_NEG_PIN, HIGH);
    digitalWrite(MAX485_DE_PIN, HIGH);
}

void postTransmission() {
    digitalWrite(MAX485_RE_NEG_PIN, LOW);
    digitalWrite(MAX485_DE_PIN, LOW);
}
