#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include "esp_adc_cal.h"
#include "arduinoFFT.h"
#include "math.h"
#include "rubint.h"

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03


Adafruit_MPU6050 mpu;
TFT_eSPI tft = TFT_eSPI(135, 240);

const uint16_t samples = 1024; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = 1024; // Hz
const double interval = 1/samplingFrequency;
const double sampling_period_us = round(interval*1000000);

const double max_amplitude = 2;
const double warn_amplitude = 1.5;


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
  Wire.begin(21, 22, (int)4e5);
  Serial.begin(1500000);

  // TFT setup
  tft.init();
  tft.setTextFont(1);
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  // draw image
  tft.pushImage(0, 0, 240, 135, (uint16_t *)epd_bitmap_rubint);

  // MPU6050 setup
  while (!mpu.begin()) {
    ESP_LOGE("MPU6050", "Could not find a valid MPU6050 sensor, check wiring!");
    delay(100);
  }
  ESP_LOGI("MPU6050", "Found MPU6050 sensor!");

  // mpu.setClock(MPU6050_CLOCK_PLL_XGYRO);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_DISABLE);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setTemperatureStandby(true);
  delay(1e3);
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
  tft.fillScreen(TFT_BLACK);
  tft.drawString("Calibrating ...", 0, 50, 4);
  ESP_LOGI("MPU6050", "Calibrating...");
  int j = 0;
  sensors_event_t a, g, temp;
  uint64_t timestamp;
  const uint16_t n_calib_samples = 200;
  std::array<double, n_calib_samples> x_calib, y_calib, z_calib;

  // get calibration data - device should be still
  ESP_LOGI("MPU6050", "Sampling calibration data...");
  for(int i=0; i<n_calib_samples; i++)
  {
    mpu.getEvent(&a, &g, &temp);
    x_calib[i] = a.acceleration.x;
    y_calib[i] = a.acceleration.y;
    z_calib[i] = a.acceleration.z;
    delay(10);
  }

  ESP_LOGI("MPU6050", "Calibration data sampled! Sorting...");
  // sort arrays
  std::sort(x_calib.begin(), x_calib.end());
  std::sort(y_calib.begin(), y_calib.end());
  std::sort(z_calib.begin(), z_calib.end());

  ESP_LOGI("MPU6050", "Calibration data sorted! Computing mean...");
  // compute mean from the middle of 80% of the sorted data
  double x_null = 0, y_null = 0, z_null = 0;
  for(int i = n_calib_samples / 10; i < (n_calib_samples - (n_calib_samples / 10)); i++)
  {
    x_null += x_calib[i];
    y_null += y_calib[i];
    z_null += z_calib[i];
  }

  x_null /= n_calib_samples * .8;
  y_null /= n_calib_samples * .8;
  z_null /= n_calib_samples * .8;
  
  ESP_LOGI("MPU6050", "Calibration done! Xnull: %f, Ynull: %f, Znull: %f", x_null, y_null, z_null);
  ESP_LOGI("MPU6050", "Gravity: %f", sqrt(x_null*x_null + y_null*y_null + z_null*z_null));
  tft.drawString("Calibration done!", 0, 50, 4);
  delay(5e2);
  tft.fillScreen(TFT_BLACK);

  while(1) {
    /*
    These are the input and output vectors
    Input vectors receive computed results from FFT
    */
    double vReal[samples];
    double vImag[samples];
    /*SAMPLING*/
    ESP_LOGI("MPU6050", "Sampling...");
    // get timestamp in us
    double t_start = esp_timer_get_time();
    for(int i=0; i<samples; i++)
    {
      timestamp = esp_timer_get_time();  
      mpu.getEvent(&a, &g, &temp);
      double x_clean, y_clean, z_clean;
      x_clean = a.acceleration.x - x_null;
      y_clean = a.acceleration.y - y_null;
      z_clean = a.acceleration.z - z_null;
      double acc = std::sqrt(x_clean*x_clean + y_clean*y_clean + z_clean*z_clean);
      vReal[i] = acc;
      vImag[i] = 0;
      while(esp_timer_get_time() - timestamp < sampling_period_us){
        // do nothing
      }
    }

    double max = -INFINITY, min = INFINITY;
    update_min_max(vReal, samples, &max, &min);
    ESP_LOGI("MPU6050", "Sampling done in %.3fs! max: %.3fm/s, min: %.3fm/s", esp_timer_get_time() - t_start, max, min);
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

    arduinoFFT FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency);
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    FFT.Compute(FFT_FORWARD); /* Compute FFT */
    FFT.DCRemoval(); /* DC Removal */
    FFT.ComplexToMagnitude(); /* Compute magnitudes */
    double f, v;
    FFT.MajorPeak(&f, &v); /* Compute peak frequency and its corresponding magnitude */
    ESP_LOGI("MPU6050", "Major peak frequency: %f, Magnitude: %f", f, v);
    ESP_LOGI("MPU6050", "#############################################");
    // PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
    // ESP_LOGI("MPU6050", "#############################################");

    double ampl = max - min;
    if(ampl >= max_amplitude) 
      tft.setTextColor(TFT_RED, TFT_BLACK);
    else if (ampl >= warn_amplitude)
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    else
      tft.setTextColor(TFT_GREEN, TFT_BLACK);

    tft.drawString("Peak f : ", 0, 0, 4);
    tft.drawString(String(f, 1) + "Hz       ", 100, 0, 4);
    tft.drawString("Magn.  : ", 0, 40, 4);
    tft.drawString(String(v, 3) + "       ", 100, 40, 4);
    tft.drawString("Ampl.  : ", 0, 80, 4);
    tft.drawString(String(ampl, 3) + "m.s^2       ", 100, 80, 4);
  }
}
