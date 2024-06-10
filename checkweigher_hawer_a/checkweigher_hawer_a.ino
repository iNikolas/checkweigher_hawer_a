#include <SoftwareSerial.h>

#define MAX485_RE_NEG_PIN       2
#define MAX485_DE_PIN           3
#define SERIAL_RX_PIN           8
#define SERIAL_TX_PIN           9

#define BELT_START_SENSOR_PIN   4
#define BELT_MIDDLE_SENSOR_PIN  5
#define BELT_END_SENSOR_PIN     6
#define REJECTION_SIGNAL_PIN    7

#define BAUD_RATE               9600
#define READ_DELAY_MS           10

#define STX                     0x02
#define ETX                     0x03

float weight;
float tare = 0;
float targetWeight = 25.0;
float tolerance = 0.5;
bool weightError;
bool endSensorTrippedLast = false;

SoftwareSerial weightSerial(SERIAL_RX_PIN, SERIAL_TX_PIN);

bool readWeightData();
void preTransmission();
void postTransmission();

void setup() {
    pinMode(BELT_START_SENSOR_PIN, INPUT_PULLUP);
    pinMode(BELT_MIDDLE_SENSOR_PIN, INPUT_PULLUP);
    pinMode(BELT_END_SENSOR_PIN, INPUT_PULLUP);
    pinMode(REJECTION_SIGNAL_PIN, OUTPUT);

    digitalWrite(MAX485_RE_NEG_PIN, LOW);
    digitalWrite(MAX485_DE_PIN, LOW);

    Serial.begin(BAUD_RATE);
    weightSerial.begin(BAUD_RATE);

    Serial.println(F("Setup complete."));
}

void loop() {
    unsigned long startTime = millis();

    weightError = !readWeightData();
    
    bool startSensorTripped = !digitalRead(BELT_START_SENSOR_PIN);
    bool middleSensorTripped = !digitalRead(BELT_MIDDLE_SENSOR_PIN);
    bool endSensorTripped = !digitalRead(BELT_END_SENSOR_PIN);

    if (startSensorTripped && middleSensorTripped && endSensorTripped) {
        tare = weight;
    }

    if (startSensorTripped && middleSensorTripped && endSensorTripped && endSensorTrippedLast) {
        float bagWeight = weight - tare;

        if (abs(bagWeight - targetWeight) > tolerance) {
            digitalWrite(REJECTION_SIGNAL_PIN, HIGH);
            delay(500);
            digitalWrite(REJECTION_SIGNAL_PIN, LOW);
        }
    }

    endSensorTrippedLast = endSensorTripped;

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
        weightSerial.read();
    }

    while (true) {
        if (weightSerial.available() >= 12) {
            char startByte = weightSerial.read();
            if (startByte != STX) continue;

            char sign = weightSerial.read();
            char weightString[7];
            for (int i = 0; i < 6; i++) {
                weightString[i] = weightSerial.read();
            }
            weightString[6] = '\0';
            
            char decimalScale = weightSerial.read();
            char xorHigh = weightSerial.read();
            char xorLow = weightSerial.read();
            char endByte = weightSerial.read();
            if (endByte != ETX) continue;

            char checksum = sign;
            for (int i = 0; i < 6; i++) {
                checksum ^= weightString[i];
            }
            checksum ^= decimalScale;

            char calculatedXorHigh = (checksum >> 4) & 0x0F;
            char calculatedXorLow = checksum & 0x0F;

            if (calculatedXorHigh != (xorHigh & 0x0F) || calculatedXorLow != (xorLow & 0x0F)) {
                continue;
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
