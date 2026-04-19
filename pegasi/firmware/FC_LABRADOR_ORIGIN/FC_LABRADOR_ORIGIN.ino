#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <LoRa.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <MPU6050_Lib.h>
// ==================== CONFIGURACIÓN BME280 ====================
#define BME280_I2C_ADDRESS 0x76
#define PRECIONNIVELDELMAR (867.0)
Adafruit_BME280 bme;
// ==================== CONFIGURACIÓN MPU6050 ====================
#define MPU6050_AXOFFSET 147
#define MPU6050_AYOFFSET -57
#define MPU6050_AZOFFSET -407
#define MPU6050_GXOFFSET -38
#define MPU6050_GYOFFSET 1
#define MPU6050_GZOFFSET 2
MPU6050_Lib mpu;
// ==================== CONFIGURACIÓN PINES ====================
#define LED_STATUS PC14
#define PYRO_DROGUE1 PC7
#define PYRO_DROGUE2 PC6
#define PYRO_MAIN1 PB15
#define PYRO_MAIN2 PB14
#define BUZZ PA15
// ==================== DEFINICIONES MELODÍA ====================
#define NOTE_AS4 466
#define NOTE_F5 698
#define NOTE_C6 1047
#define NOTE_AS5 932
#define NOTE_A5 880
#define NOTE_G5 784
#define NOTE_F6 1397
// ==================== PARÁMETROS DE VUELO ====================
#define ACC_THRESHOLD 0.2
#define REQUIRED_LOW_COUNT 3
#define MICROGRAVITY_ACTIVATION_TIME 1000
#define LAUNCH_THRESHOLD 1.5
#define DESCENT_THRESHOLD 3 
const int numMediciones = 2000;
const unsigned long SAMPLE_INTERVAL = 30;
#define LANDED_ALTITUDE_THRESHOLD 3 
#define LANDED_TIME_THRESHOLD 5000 
// ==================== VARIABLES GLOBALES ====================
float referenciaAltitud = NAN;
long sampling_timer;
float roll, pitch, yaw;
unsigned long startTime;
float previousAltitude = 0;
float maxAltitude = 0;
float sumaAlturas = 0;
bool LAUNCH = false;
bool drogueDeployed = false;
bool mainDeployed = false;
bool landed = false;
unsigned long pyroDrogueStartTime = 0;
unsigned long pyroMainStartTime = 0;
unsigned long landedDetectTime = 0;
float deploymentAltitudeMain = 0;
unsigned long lastSampleTime = 0;
bool blinkState = false;
unsigned long lastBeepTime = 0;
int accLowCounter = 0;
bool microgravityDetected = false;
unsigned long microgravityStartTime = 0;
// ==================== CONFIGURACIÓN LoRa ====================
uint8_t mosiPin = PC3;
uint8_t misoPin = PC2;
uint8_t sclkPin = PB13;
int LORA_CS = PA8;
int LORA_RST = PC8;
int LORA_DIO0 = PC9;
SPIClass MySPI;
// ==================== CONFIGURACIÓN SD ====================
const int SDCS = PA4;
File logFile;
#define SD_BUFFER_SIZE 2048
char sdBuffer[SD_BUFFER_SIZE];
int sdBufferIndex = 0;
char lineBuffer[256];
// ==================== CONFIGURACIÓN GPS ====================
HardwareSerial SerialGPS(PA10, PA9);
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsAltitude = 0.0;
uint8_t gpsSatellites = 0;
bool gpsValid = false;
float gpsSpeed = 0.0;
// ==================== ESTRUCTURAS DE DATOS ====================
struct DataPackage {
  bool LAUNCH, pyroActivated;
  float realTime, q0, q1, q2, q3, azg, altitudRelativa;
  double gpsLatitude;
  double gpsLongitude;
  float gpsAltitud;
  uint8_t gpsSatellites;
  bool gpsValid;
};
DataPackage data;
struct CompactDataPackage {
  uint32_t realTime;
  uint8_t flightState;
  int32_t q0, q1, q2, q3;
  int16_t axg;
  int16_t altitudRelativa;
  int32_t gpsLatitude;
  int32_t gpsLongitude;
  int16_t gpsAltitude;
  int16_t gpsSpeed;
};
CompactDataPackage compressedData;
// ==================== SETUP ====================
void setup() {
  Wire.begin();
  Serial.begin(115200);
  SerialGPS.begin(GPSBaud);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(PYRO_DROGUE1, OUTPUT);
  pinMode(PYRO_MAIN1, OUTPUT);
  pinMode(PYRO_DROGUE2, OUTPUT);
  pinMode(PYRO_MAIN2, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  gpsLatitude = 0.0;
  gpsLongitude = 0.0;
  gpsAltitude = 0.0;
  gpsSatellites = 0;
  gpsValid = false;
  gpsSpeed = 0.0;
  digitalWrite(PYRO_DROGUE1, LOW);
  digitalWrite(PYRO_DROGUE2, LOW);
  digitalWrite(PYRO_MAIN2, LOW);
  digitalWrite(PYRO_MAIN2, LOW);
  digitalWrite(LED_STATUS, HIGH);
  if (!mpu.begin()) {
    Serial.println("Error: No se pudo inicializar MPU6050");
    tone(BUZZ, 500);
    delay(1000);
    noTone(BUZZ);
    while (1);
  }
  mpu.setOffsets(MPU6050_AXOFFSET, MPU6050_AYOFFSET, MPU6050_AZOFFSET, MPU6050_GXOFFSET, MPU6050_GYOFFSET, MPU6050_GZOFFSET);
  mpu.setMahonyGains(0.5f, 0.0f);
  tone(BUZZ, 1500);
  digitalWrite(LED_STATUS, LOW);
  delay(500);
  digitalWrite(LED_STATUS, HIGH);
  noTone(BUZZ);
  if (!bme.begin(BME280_I2C_ADDRESS)) {
    Serial.println("Error: No se encontró el sensor BME280.");
    tone(BUZZ, 500);
    delay(1000);
    noTone(BUZZ);
    while (1);
  }
  bme.setSampling(Adafruit_BME280::MODE_NORMAL, Adafruit_BME280::SAMPLING_X4, Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::SAMPLING_NONE, Adafruit_BME280::FILTER_X16, Adafruit_BME280::STANDBY_MS_0_5);
  for (int i = 0; i < numMediciones; i++) {
    sumaAlturas += bme.readAltitude(PRECIONNIVELDELMAR);
    delay(1);
  }
  referenciaAltitud = sumaAlturas / numMediciones;
  tone(BUZZ, 1500);
  digitalWrite(LED_STATUS, HIGH);
  delay(500);
  digitalWrite(LED_STATUS, LOW);
  noTone(BUZZ);
  if (!SD.begin(SDCS)) {
    tone(BUZZ, 500);
    delay(1000);
    noTone(BUZZ);
    while (1);
  }
  char fileName[20];
  int fileIndex = 0;
  while (true) {
    snprintf(fileName, sizeof(fileName), "golo%d.txt", fileIndex);
    if (!SD.exists(fileName)) break;
    fileIndex++;
  }
  logFile = SD.open(fileName, FILE_WRITE);
  if (!logFile) {
    tone(BUZZ, 500);
    delay(1000);
    noTone(BUZZ);
    while (1);
  }
  Serial.print("Archivo de log creado: ");
  Serial.println(fileName);
  logFile.println("realTime,LAUNCH,drogueDeployed,mainDeployed,q0,q1,q2,q3,axg,altitudRelativa,gpsLatitude,gpsLongitude,gpsAltitude,gpsSpeed");
  logFile.flush();
  tone(BUZZ, 1500);
  digitalWrite(LED_STATUS, HIGH);
  delay(500);
  digitalWrite(LED_STATUS, LOW);
  noTone(BUZZ);
  MySPI.setMOSI(mosiPin);
  MySPI.setMISO(misoPin);
  MySPI.setSCLK(sclkPin);
  MySPI.begin();
  LoRa.setSPI(MySPI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  LoRa.setSyncWord(0xC1);
  if (!LoRa.begin(433E6)) {
    Serial.println("ERROR: LoRa no inicializado");
    while (1) {
      tone(BUZZ, 500);
      delay(1000);
      noTone(BUZZ);
    }
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(250E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(20);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();
  playSuccessMelody();
  digitalWrite(LED_STATUS, LOW);
  sampling_timer = micros();
  startTime = millis();
}
// ==================== LOOP PRINCIPAL ====================
void loop() {
  procesarGPS();
  unsigned long currentTime = millis();
  unsigned long realTime = currentTime - startTime;
  float altitudAbsoluta = bme.readAltitude(PRECIONNIVELDELMAR);
  float altitudRelativa = altitudAbsoluta - referenciaAltitud;
  if (LAUNCH && !landed) {
    if (!drogueDeployed) {
      if (altitudRelativa > maxAltitude) maxAltitude = altitudRelativa;
      float altitudeDrop = maxAltitude - altitudRelativa;
      if (altitudeDrop >= DESCENT_THRESHOLD && maxAltitude > 1.5) {
        tone(BUZZ, 1000, 500);
        drogueDeployed = true;
        pyroDrogueStartTime = currentTime;
        digitalWrite(PYRO_DROGUE1, HIGH);
        digitalWrite(PYRO_DROGUE2, HIGH);
        deploymentAltitudeMain = maxAltitude * 0.55;
      }
    }
    if (drogueDeployed && !mainDeployed) {
      if (altitudRelativa <= deploymentAltitudeMain) {
        tone(BUZZ, 2000, 500);
        mainDeployed = true;
        pyroMainStartTime = currentTime;
        digitalWrite(PYRO_MAIN1, HIGH);
        digitalWrite(PYRO_MAIN2, HIGH);
      }
    }
  }
  if (drogueDeployed && (currentTime - pyroDrogueStartTime >= 300)) {
    digitalWrite(PYRO_DROGUE1, LOW);
    digitalWrite(PYRO_DROGUE2, LOW);
  }
  if (mainDeployed && (currentTime - pyroMainStartTime >= 300)) {
    digitalWrite(PYRO_MAIN1, LOW);
    digitalWrite(PYRO_MAIN2, LOW);
  }
  if (drogueDeployed && !landed) {
    if (altitudRelativa < LANDED_ALTITUDE_THRESHOLD) {
      if (landedDetectTime == 0) landedDetectTime = currentTime;
      else if (currentTime - landedDetectTime > LANDED_TIME_THRESHOLD) {
        landed = true;
        if (logFile && sdBufferIndex > 0) {
          logFile.write(sdBuffer, sdBufferIndex);
          sdBufferIndex = 0;
        }
      }
    } else {
      landedDetectTime = 0;
    }
  }
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;
    if (landed) {
      blinkState = !blinkState;
      digitalWrite(LED_STATUS, blinkState);
      if (currentTime - lastBeepTime > 1000) {
        lastBeepTime = currentTime;
        tone(BUZZ, 1500, 250);
      }
    }
    mpu.readSensorData();
    mpu.updateQuaternions();
    float axg = mpu.getAccelX();
    float ayg = mpu.getAccelY();
    float azg = mpu.getAccelZ();
    float gxrs = mpu.getGyroX();
    float gyrs = mpu.getGyroY();
    float gzrs = mpu.getGyroZ();
    float accTotal = sqrt(pow(axg, 2) + pow(ayg, 2) + pow(azg, 2));
    if (!LAUNCH && axg > LAUNCH_THRESHOLD) LAUNCH = true;
    mpu.MahonyAHRSupdateIMU(gxrs, gyrs, gzrs, axg, ayg, azg);
    float q0, q1, q2, q3;
    mpu.getQuaternions(q0, q1, q2, q3);
    mpu.getRollPitchYaw(roll, pitch, yaw);
    char tempFloat[16];
    snprintf(lineBuffer, sizeof(lineBuffer), "%lu,%d,%d,%d,", realTime, LAUNCH, drogueDeployed, mainDeployed);
    dtostrf(q0, 9, 6, tempFloat); strcat(lineBuffer, tempFloat); strcat(lineBuffer, ",");
    dtostrf(q1, 9, 6, tempFloat); strcat(lineBuffer, tempFloat); strcat(lineBuffer, ",");
    dtostrf(q2, 9, 6, tempFloat); strcat(lineBuffer, tempFloat); strcat(lineBuffer, ",");
    dtostrf(q3, 9, 6, tempFloat); strcat(lineBuffer, tempFloat); strcat(lineBuffer, ",");
    dtostrf(axg, 6, 2, tempFloat); strcat(lineBuffer, tempFloat); strcat(lineBuffer, ",");
    dtostrf(altitudRelativa, 7, 2, tempFloat); strcat(lineBuffer, tempFloat); strcat(lineBuffer, ",");
    dtostrf(gpsLatitude, 10, 6, tempFloat); strcat(lineBuffer, tempFloat); strcat(lineBuffer, ",");
    dtostrf(gpsLongitude, 11, 6, tempFloat); strcat(lineBuffer, tempFloat); strcat(lineBuffer, ",");
    dtostrf(gpsAltitude, 7, 2, tempFloat); strcat(lineBuffer, tempFloat); strcat(lineBuffer, ",");
    dtostrf(gpsSpeed, 6, 2, tempFloat); strcat(lineBuffer, tempFloat); strcat(lineBuffer, "\n");
    int lineLen = strlen(lineBuffer);
    if (sdBufferIndex + lineLen > SD_BUFFER_SIZE) {
      if (logFile) {
        logFile.write(sdBuffer, sdBufferIndex);
        logFile.flush();
      }
      sdBufferIndex = 0;
    }
    memcpy(sdBuffer + sdBufferIndex, lineBuffer, lineLen);
    sdBufferIndex += lineLen;
    compressedData.realTime = realTime;
    compressedData.flightState = (LAUNCH ? 0x01 : 0) | (drogueDeployed ? 0x02 : 0) | (mainDeployed ? 0x04 : 0);
    compressedData.q0 = (int32_t)(q0 * 1e6);
    compressedData.q1 = (int32_t)(q1 * 1e6);
    compressedData.q2 = (int32_t)(q2 * 1e6);
    compressedData.q3 = (int32_t)(q3 * 1e6);
    compressedData.axg = (int16_t)(axg * 100);
    compressedData.altitudRelativa = (int16_t)(altitudRelativa * 100);
    compressedData.gpsLatitude = (int32_t)(gpsLatitude * 1e6);
    compressedData.gpsLongitude = (int32_t)(gpsLongitude * 1e6);
    compressedData.gpsAltitude = (int16_t)(gpsAltitude * 100);
    compressedData.gpsSpeed = (int16_t)(gpsSpeed * 100);
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&compressedData, sizeof(compressedData));
    LoRa.endPacket();
    blinkState = !blinkState;
    digitalWrite(LED_STATUS, blinkState);
  }
}
// ==================== FUNCIONES AUXILIARES ====================
void procesarGPS() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        gpsLatitude = gps.location.lat();
        gpsLongitude = gps.location.lng();
        gpsValid = true;
      }
      if (gps.altitude.isValid()) gpsAltitude = gps.altitude.meters();
      if (gps.satellites.isValid()) gpsSatellites = gps.satellites.value();
      if (gps.speed.isValid()) gpsSpeed = gps.speed.mps();
    }
  }
}
void playSuccessMelody() {
  int melody[] = {NOTE_AS4, NOTE_AS4, NOTE_AS4, NOTE_F5, NOTE_C6, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6};
  int durations[] = {8, 8, 8, 2, 2, 8, 8, 8, 2, 4};
  int size = 10;
  for (int note = 0; note < size; note++) {
    int duration = 1000 / durations[note];
    tone(BUZZ, melody[note], duration);
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZ);
  }
}