#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include "esp_adc_cal.h"
#include "arduinoFFT.h"
#include "math.h"


#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

Adafruit_MPU6050 mpu;
TFT_eSPI tft = TFT_eSPI(135, 240);

const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 1024; //Hz
const double interval = 1/samplingFrequency;
const double sampling_period_us = round(interval*1000000);

const double max_amplitude = 2;
const double warn_amplitude = 1.5;
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
	break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

void setup() {
  Wire.begin(21, 22, (int)1e6);
  Serial.begin(1500000);
  while (!Serial)
    delay(10);

  Serial.println("Start");

  // MPU6050 setup
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(100);
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
  tft.setTextFont(1);
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
}

void update_min_max(double *vReal, uint16_t samples, double *max, double *min)
{
  for (uint16_t i = 0; i < samples; i++)
  {
    if(vReal[i] > *max)
      *max = vReal[i];
    if(vReal[i] < *min)
      *min = vReal[i];
  }
}

void loop() {
  int j = 0;
  sensors_event_t a, g, temp;
  uint64_t timestamp;
  double x_null = 0, x_clean = 0, y_null = 0, y_clean = 0, z_null = 0, z_clean = 0;
  ESP_LOGI("MPU6050", "Calibrating...");
  for(int i=0; i<1000; i++)
  {
    mpu.getEvent(&a, &g, &temp);
    x_null += a.acceleration.x;
    y_null += a.acceleration.y;
    z_null += a.acceleration.z;
  }
  x_null /= 1000;
  y_null /= 1000;
  z_null /= 1000;
  ESP_LOGI("MPU6050", "Calibration done! Xnull: %f, Ynull: %f, Znull: %f", x_null, y_null, z_null);

  while(1) {
    // get timestamp in us

    /*SAMPLING*/
    ESP_LOGI("MPU6050", "Sampling...");
    for(int i=0; i<samples; i++)
    {
      timestamp = esp_timer_get_time();  
      mpu.getEvent(&a, &g, &temp);
      x_clean = a.acceleration.x - x_null;
      y_clean = a.acceleration.y - y_null;
      z_clean = a.acceleration.z - z_null;
      double acc = std::sqrt(x_clean*x_clean + y_clean*y_clean + z_clean*z_clean);
      vReal[i] = acc;
      /*
      vRealGX[i] = g.gyro.x;
      vRealGY[i] = g.gyro.y;
      vRealGZ[i] = g.gyro.z;
      */
      vImag[i] = 0;
      while(esp_timer_get_time() - timestamp < sampling_period_us){
        // do nothing
      }
    }

    double max = -INFINITY, min = INFINITY;
    update_min_max(vReal, samples, &max, &min);
    ESP_LOGI("MPU6050", "Sampling done! max: %.3fm/s, min: %.3fm/s", max, min);
    /*
    uint16_t fps = 1000000 / (esp_timer_get_time() - timestamp);
    a.acceleration.x = mpu.accX;
    a.acceleration.y = mpu.accY;
    a.acceleration.z = mpu.accZ;
    g.gyro.x = mpu.gyroX;
    g.gyro.y = mpu.gyroY;
    g.gyro.z = mpu.gyroZ;
    */

    ESP_LOGI("MPU6050", "Computing FFT...");

    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
    double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
    ESP_LOGI("MPU6050", "Major peak frequency AX: %f, Magnitude: %f", x, vReal[(uint16_t)(x*samples/samplingFrequency)]);
    ESP_LOGI("MPU6050", "#############################################");
    PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
    ESP_LOGI("MPU6050", "#############################################");

    double ampl = max - min;
    if(ampl >= max_amplitude) 
      tft.setTextColor(TFT_RED, TFT_BLACK);
    else if (ampl >= warn_amplitude)
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    else
      tft.setTextColor(TFT_GREEN, TFT_BLACK);

    tft.drawString("Peak f : ", 0, 0, 4);
    tft.drawString(String(x, 1) + "Hz    ", 100, 0, 4);
    tft.drawString("Magn.  : ", 0, 40, 4);
    tft.drawString(String(vReal[(uint16_t)(x*samples/samplingFrequency)], 3) + "    ", 100, 40, 4);
    tft.drawString("Ampl.  : ", 0, 80, 4);
    tft.drawString(String(ampl, 3) + "m.s^2    ", 100, 80, 4);
  }
}
