#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <SFE_BMP180.h>
#include <HMC5883L.h>

SFE_BMP180 pressure;
double baseline; 
HMC5883L compass;
MPU6050 accelgyro;



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
    if (pressure.begin())
      Serial.println("BMP180 init success");
    else
    {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
    while(1);
    }
    baseline = getPressure();
    
    Serial.print("baseline pressure: ");
    Serial.print(baseline);
    Serial.println(" mb"); 
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
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
 double a,P;
  P = getPressure();
  a = pressure.altitude(P,baseline);
  
  Serial.print("relative altitude: ");
  if (a >= 0.0) Serial.print(" "); 
  Serial.print(a,1);
  Serial.print(" meters, ");
//  if (a >= 0.0) Serial.print(" "); 
//  Serial.print(a*3.28084,0);
//  Serial.println(" feet");
  delay(500);
}

double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
