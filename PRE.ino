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
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
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
  if (!LoRa.begin(868050E3)) {
    while (1)
      ;
  }
}

String IRS() {
  String s = "_PIT";
  compass.getHeading(&mx, &my, &mz);
  float hdg = atan2(my, mx);

  if (hdg < 0) {
    hdg += 2 * M_PI;
  }

  hdg = hdg * 180 / M_PI;
  mpu.update();
  s.concat(mpu.getAngleX());
  s.concat("_ROL");
  s.concat(mpu.getAngleY());
  s.concat("_HDG");
  s.concat(hdg);
  return s;
}

String BMP() {
  status = bmp.startTemperature();
  if (status != 0) {
    delay(status);
    status = bmp.getTemperature(T);
    String s = "_TAT";
    s.concat(T);
    s.concat("_PRS");
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
          s.concat("_ALT");
          s.concat(A);
          return s;
        }
      }
    }
  }
}

double TARGET_LAT = 41.8902102, TARGET_LON = 12.4922309; //COLISEO ROMA



String LX() {
  String s = "_SPD";
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

  s.concat(gps.speed.kmph());
  s.concat("_CRS");
  s.concat(crs);
  return s;
  
}

void loop() {
  String msg = "LVSAT";
  msg.concat(IRS());
  msg.concat(BMP());
  msg.concat(LX());

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.print("\r\n");
  LoRa.endPacket();
  Serial.println(msg);
  
  smartDelay(75);
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