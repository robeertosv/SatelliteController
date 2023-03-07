#include <SPI.h>
#include <LoRa.h>
#include <SFE_BMP180.h>
#include <HMC5883L.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <string>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define cs 5
#define rst 13
#define dio0 14

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

double Po;
char status;
double T, P, PA, A;

MPU6050 mpu(Wire);
SFE_BMP180 bmp;
HMC5883L compass;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
int16_t mx, my, mz;


void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  Wire.begin();
  compass.initialize();

  mpu.begin();       
  mpu.calcOffsets(); 
  delay(1000);

  if (bmp.begin()) {
    status = bmp.startTemperature();
    if (status != 0) {
      delay(status);
      status = bmp.getTemperature(T);
      if (status != 0) {
        status = bmp.startPressure(3);
        if (status != 0) {
          delay(status);
          status = bmp.getPressure(P, T);
          if (status != 0) {
            Po = P;
          }
        }
      }
    }
  }
  
  LoRa.setPins(cs, rst, dio0);
  if (!LoRa.begin(866E6)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

  LoRa.beginPacket();
  LoRa.print("LV-IGN");
  LoRa.endPacket();
}

String IRS() {
  String s = "-PIT-";
  compass.getHeading(&mx, &my, &mz);
  float hdg = atan2(my, mx);

  if (hdg < 0) {
    hdg += 2 * M_PI;
  }

  hdg = hdg * 180 / M_PI;

  s.concat(mpu.getAngleX());
  s.concat("-ROL-");
  s.concat(mpu.getAngleY());
  s.concat("-HDG-");
  s.concat(hdg);
  return s;
}

String BMP() {
  status = bmp.startTemperature();
  if (status != 0) {
    delay(status);
    status = bmp.getTemperature(T);
    String s = "-TAT-";
    s.concat(T);
    s.concat("-PRS-");

    status = bmp.startPressure(3);
    if (status != 0) {
      delay(status);
      status = bmp.getPressure(P, T);
      s.concat(P);

      status = bmp.startPressure(3);
      if (status != 0) {
        delay(status);
        status = bmp.getPressure(P, T);
        if (status != 0) {
          A = bmp.altitude(P, Po);
          s.concat("-ALT-");
          s.concat(A);
          return s;
        }
      }
    }
  }
}

double TARGET_LAT = 41.8902102, TARGET_LON = 12.4922309; //COLISEO ROMA



String LX() {
  String s = "-LAT-";
  double lat = gps.location.lat();
  double lon = gps.location.lng();
  String lats = String(lat, 6);
  String longs = String(lon,6);

  double crs =
  TinyGPSPlus::courseTo(
    gps.location.lat(),
    gps.location.lng(),
    TARGET_LAT,
    TARGET_LON);

  s.concat(lats);
  s.concat("-LON-");
  s.concat(longs);
  s.concat("-SPD-");
  s.concat(gps.speed.kmph());
  s.concat("-CRS-");
  s.concat(crs);
  return s;
  
}

void loop() {
  mpu.update();
  Serial.print(IRS() + BMP());
  Serial.println(LX());
  smartDelay(0);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}