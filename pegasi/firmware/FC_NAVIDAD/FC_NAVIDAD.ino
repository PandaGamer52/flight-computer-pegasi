#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// ==================== DEFINICIONES DE NOTAS ====================
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988
#define NOTE_C6  1047

// ==================== CONFIGURACIÓN BME280 ====================
#define BME280_I2C_ADDRESS 0x76
#define PRECIONNIVELDELMAR (1025.0) 
Adafruit_BME280 bme;

// ==================== CONFIGURACIÓN PINES ====================
#define LED_STATUS PC14

// Asignación lógica de salidas:
#define PYRO_SALIDA_1 PC7  // 40 metros
#define PYRO_SALIDA_2 PC6  // 80 metros
#define PYRO_SALIDA_3 PB15 // 120 metros
#define PYRO_SALIDA_4 PB14 // Solo recuperación
#define BUZZ PA15

// ==================== PARÁMETROS DE VUELO ====================
#define ALT_EVENTO_1 40.0   
#define ALT_EVENTO_2 80.0   
#define ALT_EVENTO_3 120.0  

// Lógica original: 3 metros de caída para confirmar apogeo
#define DESCENT_THRESHOLD 3.0       
#define PYRO_ON_TIME 500            

const unsigned long SAMPLE_INTERVAL = 100; // 10Hz

// Parámetros de detección de aterrizaje
#define LANDED_ALTITUDE_THRESHOLD 5 
#define LANDED_TIME_THRESHOLD 5000  

// ==================== VARIABLES GLOBALES ====================
float referenciaAltitud = NAN;
unsigned long startTime;
float maxAltitude = 0; // Apogeo

// Variable corregida (declarada globalmente)
float sumaAlturas = 0; 

// Banderas de estado
bool event40mTriggered = false;
bool event80mTriggered = false;
bool event120mTriggered = false;
bool recoveryDeployed = false; 
bool landed = false;

// Tiempos
unsigned long timeEvent40m = 0;
unsigned long timeEvent80m = 0;
unsigned long timeEvent120m = 0;
unsigned long timeRecovery = 0;
unsigned long landedDetectTime = 0;

unsigned long lastSampleTime = 0;
bool blinkState = false;
unsigned long lastBeepTime = 0;

// ==================== CONFIGURACIÓN LoRa ====================
uint8_t mosiPin = PC3;
uint8_t misoPin = PC2;
uint8_t sclkPin = PB13;
int LORA_CS = PA8;
int LORA_RST = PC8;
int LORA_DIO0 = PC9;
SPIClass MySPI;

// ==================== CONFIGURACIÓN GPS ====================
HardwareSerial SerialGPS(PA10, PA9);
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
float gpsLatitude = 0.0;
float gpsLongitude = 0.0;

// ==================== ESTRUCTURA DE DATOS LORA ====================
struct CompactDataPackage {
  int32_t gpsLatitude;     
  int32_t gpsLongitude;    
  int16_t altitudRelativa; 
};
CompactDataPackage compressedData;

// ==================== SETUP ====================
void setup() {
  pinMode(LED_STATUS, OUTPUT);
  pinMode(PYRO_SALIDA_1, OUTPUT);
  pinMode(PYRO_SALIDA_2, OUTPUT);
  pinMode(PYRO_SALIDA_3, OUTPUT);
  pinMode(PYRO_SALIDA_4, OUTPUT);
  pinMode(BUZZ, OUTPUT);

  // Seguridad: Todo apagado al inicio
  digitalWrite(PYRO_SALIDA_1, LOW);
  digitalWrite(PYRO_SALIDA_2, LOW);
  digitalWrite(PYRO_SALIDA_3, LOW);
  digitalWrite(PYRO_SALIDA_4, LOW);
  
  digitalWrite(LED_STATUS, HIGH);
  delay(1000);

  Wire.begin();
  Serial.begin(115200);
  SerialGPS.begin(GPSBaud);

  // --- INICIALIZAR BME280 ---
  if (!bme.begin(BME280_I2C_ADDRESS)) {
    Serial.println("Error BME280");
    while (1) { tone(BUZZ, 200, 500); delay(500); }
  }
  
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X4,
                  Adafruit_BME280::SAMPLING_X16,
                  Adafruit_BME280::SAMPLING_NONE,
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_0_5);

  // --- CALIBRAR ALTITUD ---
  sumaAlturas = 0; 
  int numMediciones = 100;
  for (int i = 0; i < numMediciones; i++) {
    sumaAlturas += bme.readAltitude(PRECIONNIVELDELMAR);
    delay(10);
  }
  referenciaAltitud = sumaAlturas / numMediciones;

  // --- INICIALIZAR LORA ---
  MySPI.setMOSI(mosiPin);
  MySPI.setMISO(misoPin);
  MySPI.setSCLK(sclkPin);
  MySPI.begin();
  LoRa.setSPI(MySPI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Error LoRa");
    while(1){ tone(BUZZ, 200, 500); delay(500); }
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setTxPower(20);
  
  playJingleBells();
  digitalWrite(LED_STATUS, LOW);
  startTime = millis();
}

// ==================== LOOP PRINCIPAL ====================
void loop() {
  // 1. GPS Continuo
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        gpsLatitude = gps.location.lat();
        gpsLongitude = gps.location.lng();
      }
    }
  }

  unsigned long currentTime = millis();

  // 2. Muestreo 10Hz
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;

    // --- MODO ATERRIZAJE ---
    if (landed) {
      blinkState = !blinkState;
      digitalWrite(LED_STATUS, blinkState);
      if (currentTime - lastBeepTime > 2000) { // Cada 2 segundos
        lastBeepTime = currentTime;
        playJingleBells(); 
        enviarLoRa(0);     
      }
      return; 
    }

    // --- LECTURA ALTITUD ---
    float altitudAbsoluta = bme.readAltitude(PRECIONNIVELDELMAR);
    float altitudRelativa = altitudAbsoluta - referenciaAltitud;

    // --- DETECCIÓN DE APOGEO ---
    if (altitudRelativa > maxAltitude) {
      maxAltitude = altitudRelativa;
    }

    // --- EVENTOS DE ASCENSO ---
    // Solo si NO estamos bajando ya (Recovery no activado)
    if (!recoveryDeployed) {
        if (!event40mTriggered && altitudRelativa >= ALT_EVENTO_1) {
          digitalWrite(PYRO_SALIDA_1, HIGH);
          event40mTriggered = true;
          timeEvent40m = currentTime;
        }
        if (!event80mTriggered && altitudRelativa >= ALT_EVENTO_2) {
          digitalWrite(PYRO_SALIDA_2, HIGH);
          event80mTriggered = true;
          timeEvent80m = currentTime;
        }
        if (!event120mTriggered && altitudRelativa >= ALT_EVENTO_3) {
          digitalWrite(PYRO_SALIDA_3, HIGH);
          event120mTriggered = true;
          timeEvent120m = currentTime;
        }
    }

    // --- RECUPERACIÓN (Lógica Original) ---
    // Caída de 3 metros desde el punto más alto
    float altitudeDrop = maxAltitude - altitudRelativa;
    
    // Condición: Caída >= 3m Y Altura Máxima > 5m (para no disparar en el suelo por error)
    if (!recoveryDeployed && altitudeDrop >= DESCENT_THRESHOLD && maxAltitude > 5.0) {
      recoveryDeployed = true;
      timeRecovery = currentTime;
      
      // DISPARAR TODO
      digitalWrite(PYRO_SALIDA_1, HIGH);
      digitalWrite(PYRO_SALIDA_2, HIGH);
      digitalWrite(PYRO_SALIDA_3, HIGH);
      digitalWrite(PYRO_SALIDA_4, HIGH);
    }

    // --- APAGADO DE SEGURIDAD ---
    if (event40mTriggered && (currentTime - timeEvent40m > PYRO_ON_TIME)) {
      digitalWrite(PYRO_SALIDA_1, LOW);
    }
    if (event80mTriggered && (currentTime - timeEvent80m > PYRO_ON_TIME)) {
      digitalWrite(PYRO_SALIDA_2, LOW);
    }
    if (event120mTriggered && (currentTime - timeEvent120m > PYRO_ON_TIME)) {
      digitalWrite(PYRO_SALIDA_3, LOW);
    }
    if (recoveryDeployed && (currentTime - timeRecovery > PYRO_ON_TIME)) {
      digitalWrite(PYRO_SALIDA_1, LOW);
      digitalWrite(PYRO_SALIDA_2, LOW);
      digitalWrite(PYRO_SALIDA_3, LOW);
      digitalWrite(PYRO_SALIDA_4, LOW);
    }

    // --- DETECCIÓN ATERRIZAJE ---
    if (recoveryDeployed && altitudRelativa < LANDED_ALTITUDE_THRESHOLD) {
       if (landedDetectTime == 0) landedDetectTime = currentTime;
       else if (currentTime - landedDetectTime > LANDED_TIME_THRESHOLD) {
         landed = true;
       }
    } else {
       landedDetectTime = 0;
    }

    enviarLoRa(altitudRelativa);
    blinkState = !blinkState;
    digitalWrite(LED_STATUS, blinkState);
  }
}

// ==================== FUNCIONES AUXILIARES ====================

void enviarLoRa(float altitud) {
  if (LoRa.beginPacket()) {
    compressedData.gpsLatitude = (int32_t)(gpsLatitude * 1e6);
    compressedData.gpsLongitude = (int32_t)(gpsLongitude * 1e6);
    compressedData.altitudRelativa = (int16_t)(altitud * 100);

    LoRa.write((uint8_t*)&compressedData, sizeof(compressedData));
    LoRa.endPacket(true); 
  }
}

void playJingleBells() {
  // Melodía COMPLETA según tu solicitud
  int melody[] = {
    NOTE_E5, NOTE_E5, NOTE_E5,
    NOTE_E5, NOTE_E5, NOTE_E5,
    NOTE_E5, NOTE_G5, NOTE_C5, NOTE_D5,
    NOTE_E5,
    NOTE_F5, NOTE_F5, NOTE_F5, NOTE_F5,
    NOTE_F5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_E5,
    NOTE_E5, NOTE_D5, NOTE_D5, NOTE_E5,
    NOTE_D5, NOTE_G5
  };

  int durations[] = {
    8, 8, 4,
    8, 8, 4,
    8, 8, 8, 8,
    2,
    8, 8, 8, 8,
    8, 8, 8, 16, 16,
    8, 8, 8, 8,
    4, 4
  };

  int size = sizeof(durations) / sizeof(int);

  for (int note = 0; note < size; note++) {
    int duration = 1000 / durations[note];
    tone(BUZZ, melody[note], duration);
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZ);
  }
}