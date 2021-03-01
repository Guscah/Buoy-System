#include "I2Cdev.h"
#include "MPU6050.h"
#include "BMP085.h"
#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;
MPU6050 accelgyro;
BMP085 barometer;

float temperature;
float pressure;
int32_t altitude;
float lalu;
float sekarang;
float perpindahan, perpindahan1;
long kecepatan1, kecepatan2;
int16_t ax, ay, az;
int16_t gx, gy, gz;

void ketinggianaccel();
void compas();
void bmp();

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_ACCELGYRO

void setup() {

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(38400);

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    barometer.initialize();
    Serial.println("Testing device connections...");
    Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    lalu=0;
    perpindahan=0;
    perpindahan1=0;
    kecepatan2=0;
    while (!compass.begin()){
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
    }
    
  compass.setRange(HMC5883L_RANGE_1_3GA);

  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  compass.setSamples(HMC5883L_SAMPLES_8);

  compass.setOffset(0, 0);
}

void loop() {
 ketinggianaccel();
 bmp(); 
 compas();
 delay(500);
}

void compas(){
  Vector norm = compass.readNormalize();
  float heading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;
    if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
    }
  float headingDegrees = heading * 180/M_PI; 
      
  Serial.print("Kompas x = ");
  Serial.print(norm.XAxis);
  Serial.print(" Kompas y = ");
  Serial.print(norm.YAxis);
  Serial.print(" Kompas z = ");
  Serial.print(norm.ZAxis);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);

  if (headingDegrees <= 22){
    Serial.println(" Arah : Utara");
  }

  if (headingDegrees <= 67 && headingDegrees > 22){
    Serial.println(" Arah : Timur Laut");
  }
  
  if (headingDegrees <= 122 && headingDegrees > 67){
    Serial.println(" Arah : Timur");
  }

  if (headingDegrees <= 157 && headingDegrees > 122){
    Serial.println(" Arah : Tenggara");
  }

  if (headingDegrees <= 202 && headingDegrees > 157){
    Serial.println(" Arah : Selatan");
  }

  if (headingDegrees <= 247 && headingDegrees > 202){
    Serial.println(" Arah : Barat Daya");
  }

  if (headingDegrees <= 292 && headingDegrees > 247){
    Serial.println(" Arah : Barat");
  }

  if (headingDegrees <= 337 && headingDegrees > 292){
    Serial.println(" Arah : Barat Laut");
  }

  if (headingDegrees > 337){
    Serial.println(" Arah : Utara");
  }


}

void ketinggianaccel(){
    // Output

  #ifdef OUTPUT_READABLE_ACCELGYRO
    long sumbux=ax;
    long sumbuy=ay;
    long sumbuz=az;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sekarang=millis();
    float delay1 = (sekarang-lalu);
    lalu=sekarang;
    kecepatan2=kecepatan2+(sumbuy*0.05);
    perpindahan1=perpindahan1+(kecepatan2*0.05);
    float selisih=perpindahan1-perpindahan;
    perpindahan1=perpindahan;
    Serial.print("Accel x = ");
    Serial.print(sumbux);
    Serial.print(" Accel y = ");
    Serial.print(sumbuy);
    Serial.print(" Accel x = ");
    Serial.print(sumbuz);
    Serial.print(" perubahan Ketinggian = ");
    Serial.print(selisih);
    #endif
    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

}

void bmp(){
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    temperature = barometer.getTemperatureC();
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    pressure = barometer.getPressure();

    altitude = barometer.getAltitude(pressure);

    Serial.print(" Ketinggian Barometer = ");
    Serial.print(altitude);
    Serial.println("");
}
