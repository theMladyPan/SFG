#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include "WiFi.h"
#include "esp_adc_cal.h"

Adafruit_MPU6050 mpu;
TFT_eSPI tft = TFT_eSPI(135, 240);

const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 500; //Hz
const double interval = 1/samplingFrequency;
const double sampling_period_us = round(interval*1000000);
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

void setup() {
  Wire.begin(21, 22, 800000);
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Start");

  // MPU6050 setup
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // mpu.setClock(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_DISABLE);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setTemperatureStandby(true);
  /*
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  */

  // TFT setup
  tft.init();
  tft.fontHeight(3);
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
}

void loop() {
  int j = 0;
  tft.drawString("AccelX:", 10, 30, 2);
  tft.drawString("AccelY:", 10, 60, 2);
  tft.drawString("AccelZ:", 10, 90, 2);
  tft.drawString("GyroX:", 10, 120, 2);
  tft.drawString("GyroY:", 10, 150, 2);
  tft.drawString("GyroZ:", 10, 180, 2);
  tft.drawString("FPS:", 10, 210, 2);
  sensors_event_t a, g, temp;
  while(1) {
    // get timestamp in us
    mpu.getEvent(&a, &g, &temp);
    mpu._read();

    /*SAMPLING*/
    uint64_t timestamp = esp_timer_get_time();  
    for(int i=0; i<1000; i++)
    {
        vReal[i] = analogRead(CHANNEL);
        vImag[i] = 0;
        while(micros() - microseconds < sampling_period_us){
          //empty loop
        }
        microseconds += sampling_period_us;
    }
    uint16_t fps = 1000000 / (esp_timer_get_time() - timestamp);
    /*
    a.acceleration.x = mpu.accX;
    a.acceleration.y = mpu.accY;
    a.acceleration.z = mpu.accZ;
    g.gyro.x = mpu.gyroX;
    g.gyro.y = mpu.gyroY;
    g.gyro.z = mpu.gyroZ;
    */


    if(j++%100==0) {
      // Clear the TFT screen
      // tft.fillScreen(TFT_BLACK);

      // Display accelerometer and gyroscope values
      tft.drawString(String(a.acceleration.x) + String("  "), 80, 30, 2);
      tft.drawString(String(a.acceleration.y) + String("  "), 80, 60, 2);
      tft.drawString(String(a.acceleration.z) + String("  "), 80, 90, 2);
      tft.drawString(String(g.gyro.x) + String("  "), 80, 120, 2);
      tft.drawString(String(g.gyro.y) + String("  "), 80, 150, 2);
      tft.drawString(String(g.gyro.z) + String("  "), 80, 180, 2); 

      // display fps
      tft.drawString(String(fps), 80, 210, 2);
    }
  }
}

{

  /* Print the results of the sampling according to time */
  Serial.println("Data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  Serial.println("Weighed data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  Serial.println("Computed Real values:");
  PrintVector(vReal, samples, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  Serial.println(x, 6); //Print out what frequency is the most dominant.
  while(1); /* Run Once */
  // delay(2000); /* Repeat after delay */
}
