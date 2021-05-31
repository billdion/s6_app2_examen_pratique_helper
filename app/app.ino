// This #include statement was automatically added by the Particle IDE.
#include <google-maps-device-locator.h>

// This #include statement was automatically added by the Particle IDE.
#include <ArduinoJson.h>

// This #include statement was automatically added by the Particle IDE.
#include <Wire.h>


// GPS
GoogleMapsDeviceLocator locator;
double latitude, longitude = 0;


// HUMIDITY AND TEMPERATURE
int temp_dbl, humidity_dbl;

// LIGHT LEVEL
double currentLightLevel;

// BAROMETER
double pressure;
#define BAROMETER_ADDR 0x77

// WIND DIRECTION
double analogValue;
double windDirection;
double voltageValues[] = {3.84, 1.98, 2.25, 0.41, 0.45, 0.32, 0.90, 0.62, 1.40, 1.19, 3.08, 2.93, 4.62, 4.04, 4.33, 3.43};
double directionValues[] = {0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5};


// WIND SPEED
double averageSpeed;
double  speedPerSec = 2.4;
volatile unsigned long windContactTime;
int windContact = 0;


// RAIN
double totalRain = 0;
double rainPerContact = 0.2794;
int rainContact = 0;
volatile unsigned long rainContactTime;

// TCP CLIENT
TCPClient client;
byte server[] = { 192, 168, 50, 210 }; // Google


void setup() {
    Serial.begin(9600);
    
    setupLocation();
    
    setupDht();
    setupLight();
    setupBarometer();
    setupWindSpeed();
    setupWindDirection();
    setupRain();
}

void loop() {
    
    
    //Wifi.on();
    //delay(10000);
    
    locator.loop();
    
    dhtSensor();
    lightSensor();
    barometerSensor();
    windDirectionSensor();
    windSpeedSensor();
    rainSensor();
    delay(5000);
    
    //buildJson();
    //delay(60000);
    
    //Wifi.off();
}


void dhtSensor() {
    float temp, humidity;
    uint8_t checkSum = 0;
    uint16_t data = 0;
    unsigned long startTime;
    
    uint8_t humidity_i;
    uint8_t humidity_d;
    
    uint8_t temp_i;
    uint8_t temp_d;
    
    
    pinMode(D2, OUTPUT);
    digitalWrite(D2, LOW); 
    delay(18);
    digitalWrite(D2, HIGH);
    pinMode(D2, INPUT);
    digitalWrite(D2, HIGH); 
    
    while (digitalRead(D2) == HIGH) {
        
    }
    
    while (digitalRead(D2) == LOW) {
        
    }
    
    while (digitalRead(D2) == HIGH) {
        
    }
    
    for ( int8_t i = 0 ; i < 40; i++ ) {
        while (digitalRead(D2) == LOW) {
            // Waiting for high (50us)
        }
        
        byte live;
        startTime = micros();

        while (digitalRead(D2) == HIGH){
          live = (unsigned long)(micros() - startTime);
          if ( live > 90 ) {
            Serial.println("ERROR_TIMEOUT");
            return;
          }
        } 
    
        data <<= 1;
        
        if ( live > 30 ) {
            data |= 1; // we got a one
        }
    
        switch ( i ) {
        case 15:
            humidity_dbl = (data >> 8);
            break;
        case 31:
            temp_dbl = (data >> 8);
        case 39: 
            checkSum = data;
            data = 0;
            break;
        }
    }
    
    
    
    Serial.printlnf("Temp: %d", temp_dbl);
    Serial.printlnf("Humidity %d", humidity_dbl);
}

void lightSensor() {
    double lightAnalogVal = analogRead(A0);
    
    // Value is on 4096 (12bit)
    // value/4096 = (x/3.3)
    // value / 68 000
    
    
    //Serial.printlnf("analog sensor read: %f", lightAnalogVal);
    currentLightLevel = map(lightAnalogVal, 0.0, 4096.0, 0.0, 100.0);
    
    Serial.printlnf("Current light level %f", currentLightLevel);
}

void barometerSensor() {
    uint32_t reading = 0;
    int16_t c0, c1, c01, c11, c20, c21, c30;
    int32_t c00, c10;
    uint8_t coeffs[18];
    float dps_temperature = 0;
    float raw_pressure = 0;
    float raw_temperature = 0;
    
    
    // Demande the 3 bytes of pressure data
    Wire.beginTransmission(BAROMETER_ADDR);
    Wire.write(byte(0x00));
    Wire.endTransmission();
    
    Wire.requestFrom(BAROMETER_ADDR, 3);
    
    if (3 <= Wire.available()) {
        reading = Wire.read();
        reading = reading << 8;
        reading |= Wire.read();
        reading = reading << 8;
        reading |= Wire.read();
    }

    raw_pressure = (float)twosComplement(reading, 24) / 1572864.0;
    
    // Demand the 3 bytes of temperature data
    Wire.beginTransmission(BAROMETER_ADDR);
    Wire.write(byte(0x03));
    Wire.endTransmission();
    
    Wire.requestFrom(BAROMETER_ADDR, 3);
    reading = 0;
    
    if (3 <= Wire.available()) {
        reading = Wire.read();
        reading = reading << 8;
        reading |= Wire.read();
        reading = reading << 8;
        reading |= Wire.read();
    }
    raw_temperature = (float)twosComplement(reading, 24) / 1572864.0;
    
    

    // Demande the 3 bytes of pressure data
    Wire.beginTransmission(BAROMETER_ADDR);
    Wire.write(byte(0x10));
    Wire.endTransmission();
    
    Wire.requestFrom(BAROMETER_ADDR, 18);
    
    for (uint8_t addr = 0; addr < 18; addr++) {
        coeffs[addr] = Wire.read();
    }
    
    c0 = ((uint16_t)coeffs[0] << 4) | (((uint16_t)coeffs[1] >> 4) & 0x0F);
    c0 = twosComplement(c0, 12);
    
    c1 = twosComplement((((uint16_t)coeffs[1] & 0x0F) << 8) | coeffs[2], 12);
    
    c00 = ((uint32_t)coeffs[3] << 12) | ((uint32_t)coeffs[4] << 4) |
         (((uint32_t)coeffs[5] >> 4) & 0x0F);
    c00 = twosComplement(c00, 20);
    
    c10 = (((uint32_t)coeffs[5] & 0x0F) << 16) | ((uint32_t)coeffs[6] << 8) |
         (uint32_t)coeffs[7];
    c10 = twosComplement(c10, 20);
    
    c01 = twosComplement(((uint16_t)coeffs[8] << 8) | (uint16_t)coeffs[9], 16);
    c11 = twosComplement(((uint16_t)coeffs[10] << 8) | (uint16_t)coeffs[11], 16);
    c20 = twosComplement(((uint16_t)coeffs[12] << 8) | (uint16_t)coeffs[13], 16);
    c21 = twosComplement(((uint16_t)coeffs[14] << 8) | (uint16_t)coeffs[15], 16);
    c30 = twosComplement(((uint16_t)coeffs[16] << 8) | (uint16_t)coeffs[17], 16);
    
    
    

    //Serial.printlnf("c00: %d", c00);
    //Serial.printlnf("c10: %d", c10);
    //Serial.printlnf("c20: %d", c20);
    //Serial.printlnf("c30: %d", c30);
    //Serial.printlnf("c01: %d", c01);
    //Serial.printlnf("c11: %d", c11);
    //Serial.printlnf("c21: %d", c21);
    //Serial.printlnf("raw_pressure: %f", raw_pressure);
    //Serial.printlnf("raw_temperature: %f", raw_temperature);
    
    //pressure = c00 + raw_pressure*(c10 + raw_pressure * (c20 + raw_pressure * c30)) + raw_temperature * c01 + raw_temperature * raw_pressure * (c11 + raw_pressure*c21);
    
    pressure =
      (int32_t)c00 +
      raw_pressure * ((int32_t)c10 +
                   raw_pressure * ((int32_t)c20 + raw_pressure * (int32_t)c30)) +
      raw_temperature *
          ((int32_t)c01 +
           raw_pressure * ((int32_t)c11 + raw_pressure * (int32_t)c21));
    
    Serial.printlnf("Pressure: %f", pressure);
}

int32_t twosComplement(int32_t val, uint8_t bits) {
  if (val & ((uint32_t)1 << (bits - 1))) {
    val -= (uint32_t)1 << bits;
  }
  return val;
}


void windDirectionSensor() {
    int nearestIndex = 0;
    analogValue = (analogRead(A4) * 3.3 / 4096);
    
    //Serial.printlnf("Wind direction voltage: %f", analogValue);
    double diff = abs(voltageValues[0] - analogValue);
    
    for (int i = 0; i < 10; i++) {
        if (abs(voltageValues[i] - analogValue) < diff ) {
            nearestIndex = i;
            diff = abs(voltageValues[i] - analogValue);
        }
    }
    
    windDirection = directionValues[nearestIndex];
    
    Serial.printlnf("Wind direction: %f", windDirection);
}


void windSpeedSensor() {
    float diff = millis() - windContactTime;
    //Serial.printlnf("nb de tick de vent: %d", windContact);
    
    if (diff > 3000) {
        averageSpeed = windContact * 2.4 / (diff / 1000.0); // 2.4kmh / 3
        windContact = 0;
        Serial.printlnf("Speed average for 3 seconds: %f", averageSpeed);
        windContactTime = millis();
    }
}

void rainSensor() {
    float diff = millis() - rainContactTime;
    
    if(diff > 60000) { 
        totalRain = rainContact * (rainPerContact / (diff / 1000.0)); // 0.2794mm / 60
        rainContactTime = millis(); 
    }
    
    Serial.printlnf("Total rain for 1 minute: %f", totalRain);
}

void windTick() {
    windContact++; 
}

void rainTick() {
    rainContact++; 
}

void locationCallback(float lat, float lon, float accuracy) {
    latitude = lat;
    longitude = lon;
    Serial.printlnf("Latitude: %f", latitude);
    Serial.printlnf("Longitude: %f", longitude);
}

void buildJson() {
    Serial.println("Trying to build");
    StaticJsonDocument<300> root;
    root["temperature"] = temp_dbl;
    root["humidity"] = humidity_dbl;
    root["pressure"] = pressure;
    root["rain"] = totalRain;
    root["sun"] = currentLightLevel;
    root["wind_direction"] = windDirection;
    root["wind_speed"] = averageSpeed;
    root["lat"] = latitude;
    root["lng"] = longitude;
    
    
    Serial.println("Trying to connect");
    if (client.connect(server, 3000))
    {
        Serial.println("connected");
        serializeJson(root, client);
        client.println();
        client.stop();
    }
    else
    {
        Serial.println("connection failed");
    }
}



// ###########################
// ###### Setup sensors ######
// ###########################


void setupDht() {
    Particle.variable("temp", temp_dbl);
    Particle.variable("humidity", humidity_dbl);
}

void setupLight() {
    pinMode(A0, INPUT);
    
    Particle.variable("lightLevel", currentLightLevel);
}

void setupBarometer() {
    Wire.begin();
    
    
    // Set pressure sensor to low power
    Wire.beginTransmission(BAROMETER_ADDR);
    Wire.write(byte(0x06));
    Wire.write(byte(0x01));
    Wire.endTransmission();
    
    delay(550);
    
    
    // Set temperature sensor to low power
    Wire.beginTransmission(BAROMETER_ADDR);
    Wire.write(byte(0x07));
    Wire.write(byte(0x01));
    Wire.endTransmission();
    
    delay(550);
    
    // Continuous pressure measurement
    Wire.beginTransmission(BAROMETER_ADDR);
    Wire.write(byte(0x08));
    Wire.write(byte(0x07));
    Wire.endTransmission();
    
    delay(550);
    
    Particle.variable("pressure", pressure);
}

void setupWindDirection() {
    Particle.variable("windDirection", windDirection);
}

void setupWindSpeed() {
    attachInterrupt(digitalPinToInterrupt(A4), windTick, FALLING);
    
    Particle.variable("windSpeed", averageSpeed);
}

void setupRain() {
    attachInterrupt(digitalPinToInterrupt(A2), rainTick, FALLING);
    
    Particle.variable("totalRain", totalRain);
}

void setupLocation() {
    locator.withSubscribe(locationCallback).withLocateOnce();
    
    Particle.variable("lat", latitude);
    Particle.variable("lng", longitude);
}





