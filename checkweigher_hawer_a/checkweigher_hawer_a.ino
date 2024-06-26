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
#define READ_DELAY_MS           200
#define REJECTION_DELAY_MS      500
#define TARE_DELAY_MS           100
#define CONFIRM_TARE_DELAY_MS   5000
#define READ_WEIGHT_TIMEOUT_MS  150
#define TARE_DIFFERENCE_KG      0.1

#define STX                     0x02
#define ETX                     0x03

#define RELAY_ON                LOW
#define RELAY_OFF               HIGH

float tare = 0;
float emptyBagWeight = 0.12;
float targetWeight = 25.0;
float tolerance = 0.5;
bool bagWeighed = false;
bool rejectionConfirmed = false;
bool tareConfirmed = false;
unsigned long endSensorTrippedTime = 0;
unsigned long rejectionConfirmedTime = 0;
unsigned long tareConfirmedTime = 0;
bool endSensorTrippedTiming = false;
float weight;
bool weightError;

class WeightReader {
public:
    WeightReader(uint8_t rxPin, uint8_t txPin, uint32_t baudRate)
        : serial(rxPin, txPin) {
        serial.begin(baudRate);
    }

    bool readWeight(float &weight, bool &weightError) {
        unsigned long startMillis = millis();

        while (serial.available()) {
            serial.read();
        }

        while (millis() - startMillis < READ_WEIGHT_TIMEOUT_MS) {
            if (serial.available() >= 12) {
                if (parseWeightData(weight)) {
                    weightError = false;
                    return true;
                } else {
                    weightError = true;
                    return false;
                }
            }
        }

        weightError = true;
        return false;
    }

private:
    SoftwareSerial serial;

    bool parseWeightData(float &weight) {
        if (serial.read() != STX) return false;

        char sign = serial.read();
        char weightString[7];
        for (int i = 0; i < 6; i++) {
            weightString[i] = serial.read();
        }
        weightString[6] = '\0';

        char decimalScale = serial.read();
        char xorHigh = serial.read();
        char xorLow = serial.read();
        if (serial.read() != ETX) return false;

        char checksum = sign;
        for (int i = 0; i < 6; i++) {
            checksum ^= weightString[i];
        }
        checksum ^= decimalScale;

        char calculatedXorHigh = (checksum >> 4) & 0x0F;
        char calculatedXorLow = checksum & 0x0F;

        if (calculatedXorHigh != (xorHigh & 0x0F) || calculatedXorLow != (xorLow & 0x0F)) {
            return false;
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
};

class SensorManager {
public:
    SensorManager(uint8_t startPin, uint8_t middlePin, uint8_t endPin)
        : startPin(startPin), middlePin(middlePin), endPin(endPin) {
        pinMode(startPin, INPUT);
        pinMode(middlePin, INPUT);
        pinMode(endPin, INPUT);
    }

    void update() {
        startSensorTripped = digitalRead(startPin) == LOW;
        middleSensorTripped = digitalRead(middlePin) == LOW;
        endSensorTripped = digitalRead(endPin) == LOW;

        trackEndSensorTrippedLast();
    }

    bool isStartSensorTripped() const {
        return startSensorTripped;
    }

    bool isMiddleSensorTripped() const {
        return middleSensorTripped;
    }

    bool isEndSensorTripped() const {
        return endSensorTripped;
    }

    bool isEndSensorTrippedLast() const {
        return endSensorTrippedLast && !endSensorTripped;
    }

private:
    uint8_t startPin, middlePin, endPin;
    bool startSensorTripped, middleSensorTripped, endSensorTripped;
    bool endSensorTrippedLast = true;

    void trackEndSensorTrippedLast() {
        if (startSensorTripped || middleSensorTripped) {
            endSensorTrippedLast = false;
            return;
        }

        if (endSensorTripped) {
            endSensorTrippedLast = true;
        }
    }
};

class RejectionSystem {
public:
    RejectionSystem(uint8_t pin) : pin(pin) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, RELAY_OFF);
    }

    void activate() {
        digitalWrite(pin, RELAY_ON);
        delay(REJECTION_DELAY_MS);
        digitalWrite(pin, RELAY_OFF);
    }

private:
    uint8_t pin;
};

class RS485Transceiver {
public:
    RS485Transceiver(uint8_t reNegPin, uint8_t dePin)
        : reNegPin(reNegPin), dePin(dePin) {
        pinMode(reNegPin, OUTPUT);
        pinMode(dePin, OUTPUT);
        disableTransmission();
    }

    void preTransmission() {
        digitalWrite(reNegPin, HIGH);
        digitalWrite(dePin, HIGH);
    }

    void postTransmission() {
        disableTransmission();
    }

private:
    uint8_t reNegPin, dePin;

    void disableTransmission() {
        digitalWrite(reNegPin, LOW);
        digitalWrite(dePin, LOW);
    }
};

WeightReader weightReader(SERIAL_RX_PIN, SERIAL_TX_PIN, BAUD_RATE);
SensorManager sensorManager(BELT_START_SENSOR_PIN, BELT_MIDDLE_SENSOR_PIN, BELT_END_SENSOR_PIN);
RejectionSystem rejectionSystem(REJECTION_SIGNAL_PIN);
RS485Transceiver transceiver(MAX485_RE_NEG_PIN, MAX485_DE_PIN);

void setup() {
    Serial.begin(BAUD_RATE);
}

void loop() {
    readAndProcessWeight(weight, weightError);
    sensorManager.update();
    handleSensors(weight);
}

void readAndProcessWeight(float &weight, bool &weightError) {
    weightReader.readWeight(weight, weightError);
}

void handleSensors(float weight) {
    if (sensorManager.isEndSensorTrippedLast()) {
        if (!endSensorTrippedTiming) {
            endSensorTrippedTiming = true;
            endSensorTrippedTime = millis();
        } else if (millis() - endSensorTrippedTime >= TARE_DELAY_MS) {
            float tareDifference = abs(tare - weight);

            if (tareDifference <= TARE_DIFFERENCE_KG) {
                tare = weight;
                tareConfirmed = false;
            } else {
                if (tareConfirmed) {
                  if ((millis() - tareConfirmedTime) > CONFIRM_TARE_DELAY_MS) {
                      tare = weight;
                      tareConfirmed = false;
                  }
                } else {
                  tareConfirmed = true;
                  tareConfirmedTime = millis();
                }
            }
        }
    } else {
        endSensorTrippedTiming = false;
        tareConfirmed = false;
    }

    if (sensorManager.isMiddleSensorTripped() && !bagWeighed) {
        processBagWeight(weight);
    } else {
        bagWeighed = sensorManager.isMiddleSensorTripped();
    }
}

void processBagWeight(float weight) {
    float bagWeight = weight - tare - emptyBagWeight;
    float weightDifference = abs(bagWeight - targetWeight);

    if (weightDifference > tolerance) {
        if (rejectionConfirmed) {
            if ((millis() - rejectionConfirmedTime) > READ_DELAY_MS) {
                rejectionConfirmed = false;
                Serial.print(F("Rejected: "));
                Serial.println(weightDifference);
                rejectionSystem.activate();
            }
        } else {
            Serial.println(F("Delay before rejection..."));
            rejectionConfirmedTime = millis();
            rejectionConfirmed = true;
        }
    } else {
        rejectionConfirmed = false;
    }

    if (!rejectionConfirmed) {
        bagWeighed = true;
        
        Serial.print(F("Tare: "));
        Serial.println(tare);

        Serial.print(F("Bag weight: "));
        Serial.println(bagWeight);
    }
}
