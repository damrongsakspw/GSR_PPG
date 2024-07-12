#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>

#define TASK_INTERVAL_MS 60000//600000//900000 // 15 minutes in milliseconds
#define RUNTIME_MS 320000 // 5 minutes in milliseconds

//#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define SERVICE_UUID        "5fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define SAMPLE_RATE     20       // อัตราการเก็บข้อมูลตัวอย่างของ PPG (Hz)
#define N               4       // ออร์เดอร์ของ Butterworth filter
#define CUTOFF_FREQ     0.1    // ความถี่ตัดของ Butterworth filter (Hz)
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
char* gatt_val = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
String value_write = "";
const int GSR = A0;
const int PPG = A3;
uint32_t value = 0;

// Variables for peak detection and RR interval calculation
float threshold = 1000.0;//2.0; // Adjust as needed
boolean is_peak = false;
unsigned long last_peak_time = 0;
unsigned long rr_intervals[100]; // Assuming a maximum of 100 RR intervals
int rr_interval_count = 0;

//int PPG_RAW[4000] = {};
float rr_interval_data[1000];

//hw_timer_t *timer = NULL;
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//volatile bool taskRunning = false;

//void IRAM_ATTR onTimer() {
//  portENTER_CRITICAL_ISR(&timerMux);
//  taskRunning = true;
//  portEXIT_CRITICAL_ISR(&timerMux);
//}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      //pServer->getAdvertising()->start();
    }
};


class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      //value = new std::string(pCharacteristic->getValue());
      for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0); // Write 0 to each EEPROM address
      }
      EEPROM.commit();
      //uint8_t *value = pCharacteristic->getData();
      value_write = "";
      if (value.length() > 0) {
        //Serial.println("*********");
        //Serial.print("New value: ");

        for (int i = 0; i < value.length(); i++) {
          //gatt_val[i] = value[i];
          //Serial.print(value[i]);
          value_write += value[i];
        }
        Serial.println(value_write);
        if (value_write == "reboot")  ESP.restart();
        //Serial.println();
        //Serial.println("*********");
      }
    }
};


class Butterworth {
  private:
    float a[N + 1];
    float b[N + 1];
    float x[N + 1];
    float y[N + 1];
  public:
    Butterworth() {
      for (int i = 0; i <= N; i++) {
        x[i] = 0.0;
        y[i] = 0.0;
      }
    }

    void init(float sample_rate, float cutoff_freq) {
      float dt = 1.0 / sample_rate;
      float RC = 1.0 / (2.0 * PI * cutoff_freq);
      float alpha = dt / (RC + dt);

      float tau = 1.0 / (2.0 * PI * cutoff_freq);
      float a0 = 1.0 / (1.0 + 2.0 * tau * sample_rate + pow(tau * sample_rate, 2.0));
      a[0] = a0;
      a[1] = 2.0 * a0;
      a[2] = a0;

      b[0] = 1.0;
      b[1] = (2.0 * pow(tau * sample_rate, 2.0) - 2.0) * a0;
      b[2] = (1.0 - 2.0 * tau * sample_rate + pow(tau * sample_rate, 2.0)) * a0;
    }

    float filter(float input) {
      // ใช้กรอง Butterworth filter
      float output = 0.0;
      x[0] = input;

      for (int i = 0; i <= N; i++) {
        output += b[i] * x[i] - a[i] * y[i];
      }

      for (int i = N; i > 0; i--) {
        x[i] = x[i - 1];
        y[i] = y[i - 1];
      }

      y[0] = output;

      return output;
    }
};

Butterworth butter;

int detectPeaks(float* data, int size, int num_value) {
  int peaks[size];
  int numPeaks = 0;
  float avgBefore = 0;

  for (int i = 1; i < size - 1; i++) {
    if (data[i] > data[i - 1] && data[i] >= data[i + 1]) {
      //Serial.println(i);
      peaks[numPeaks++] = i;
      avgBefore += data[i - 1];
    }
  }
  if (numPeaks > 0) {
    avgBefore /= numPeaks;
  }
  //for
  Serial.print("Number of peaks: ");
  Serial.println(numPeaks);
  Serial.print("Average of peaks before: ");
  Serial.println(avgBefore);
  //for (int i = 0; i < numPeaks; i++)Serial.println(peaks[i]);

  float min_height = avgBefore - (avgBefore * 0.5);
  float max_height = avgBefore + (avgBefore * 0.5);

  numPeaks = 0;
  for (int i = 1; i < size - 1; i++) {
    if (data[i] > min_height && data[i] < max_height && data[i] > data[i - 1] && data[i] >= data[i + 1]) {
      peaks[numPeaks++] = i;
    }
  }
  //Serial.println("clean peak");
  //for (int i = 0; i < numPeaks; i++)Serial.println(peaks[i]);

  if (numPeaks > 1) {
    float rr_intervals[numPeaks - 1];
    for (int i = 1; i < numPeaks; i++) {
      rr_intervals[i - 1] = peaks[i] - peaks[i - 1];
      //Serial.println(rr_intervals[i - 1]);
      rr_interval_data[num_value + (i - 1)] = rr_intervals[i - 1];

    }
    //for (int i = 0; i < numPeaks - 1; i++)Serial.println(rr_intervals[i]);

    //    float sum_rr_intervals = 0;
    //    for (int i = 0; i < numPeaks - 1; i++) {
    //      sum_rr_intervals += rr_intervals[i];
    //    }
    //
    //    //    float average_rr_interval = sum_rr_intervals / (numPeaks - 1);
    //    //    Serial.print("Average RR interval: ");
    //    //    Serial.println(average_rr_interval);

    //Serial.println("cal rr");

    //    for (int i = 0; i < numPeaks - 1; i++) {
    //      if  (rr_intervals[i + 1] - rr_intervals[i]  > average_rr_interval) {
    //        Serial.println(rr_intervals[i]);
    //
    //      }

    //}
    //    long rMSSD = calculateRMSSD(rr_intervals, numPeaks - 1);
    //    Serial.print("rMSSD (ms): ");
    //    Serial.println(rMSSD);
    return numPeaks - 1;
  }


}

void task(int LONG_RUNTIME_MS, uint16_t size__rr_interval) {
  unsigned long startTime = millis(); // Get the current time
  //Serial.println(startTime);
  Serial.println("Task Start");
  uint8_t sum_rnd_select = 0;
  uint16_t size_window_min = 5;
  uint16_t size_split = size__rr_interval / size_window_min;
  char RAW_VAL[50]; // Buffer to hold the converted string
  float window_interval[size_split];
  uint32_t cnt_window = 0;
  int sum_value_rr = 0;
  int count_gsr = 0;
  // Run the task for 5 minutes
  while ((millis() - startTime) < LONG_RUNTIME_MS) {
    if (value_write == "stop")break;
    int GSR_val = analogRead(GSR);
    float PPG_RAW = analogRead(PPG) / 1.0;
    float PPG_val = butter.filter(analogRead(PPG));
    window_interval[cnt_window] = PPG_val;
    cnt_window++;
    //Serial.println(GSR_val);
    //Serial.println(cnt_window);
    //Serial.println(size_window);
    if (cnt_window == size_split) {
      //float data_ppg[size_split] = {4698.69, 5995.37, 5832.58, 6147.94, 6870.84, 7093.32, 6708.36, 6265.88, 5994.30, 5979.66, 6386.61, 7023.29, 7155.74, 6711.43, 6346.34, 6279.78, 6435.79, 6878.18, 7151.07, 6880.50, 6564.17, 6470.68, 6462.00, 6693.47, 7050.12, 7030.65, 6675.10, 6388.80, 6324.30, 6575.68, 6134.63, 4548.01, 5458.27, 8790.91, 9581.54, 8129.59, 7290.71, 7159.49, 6862.23, 6420.57, 6219.76, 6197.54, 6233.11, 6307.19, 6425.91, 6530.75, 6628.63, 6755.57, 6837.62, 7105.95, 7509.75, 7413.42, 6936.59, 6634.57, 6536.61, 6727.90, 7088.11, 7077.35, 6758.09, 6483.07, 6330.34, 6550.90, 7049.38, 7145.91, 6781.70, 6460.17, 6342.03, 6493.77, 6908.34, 7109.23, 6861.28, 6555.07, 6463.94, 6672.89, 6947.64, 6873.91, 6641.36, 6575.36, 6603.06, 6653.76, 6943.05, 7285.65, 7183.15, 6823.10, 6584.57, 6506.31, 6690.94, 7029.03, 7067.27, 6795.06, 6564.61, 6502.55, 6738.52, 7131.67, 7116.44, 6748.72, 6503.23, 6427.78, 6686.02, 7161.64, 7169.30, 6818.14, 6584.87, 6451.16, 6649.87, 7094.93, 7163.45, 6916.44, 6627.40, 6351.45, 6472.64, 6935.87, 7045.96, 6740.18, 6463.96, 6362.35, 6530.10, 6981.37, 7205.80, 6924.94, 6574.50, 6478.36, 6716.31, 6982.59, 6839.13, 6534.11, 6408.17, 6433.95, 6773.61, 7290.43, 7285.86, 6853.79, 6569.49, 6456.93, 6680.29, 7096.63, 7053.25, 6699.24, 6458.62, 6378.35, 6684.27, 7133.12, 7085.37, 6724.97, 6487.33, 6419.80, 6671.31, 7074.18, 7059.81, 6702.14, 6435.23, 6372.75, 6675.72, 7157.74, 7151.94, 6746.78, 6502.21, 6464.90, 6732.24, 7108.29, 7006.75, 6654.40, 6496.39, 6589.35, 6737.88, 6648.41, 6474.84, 6445.86, 6481.63, 6660.93, 7170.62, 7475.92, 7142.10, 6690.56, 6445.51, 6489.47, 6865.58, 7064.93, 6846.29, 6573.42, 6440.48, 6630.13, 7068.07, 7156.00, 6822.00, 6537.61, 6476.60, 6718.76, 7081.09, 7057.52, 6745.57, 6536.35, 6538.65, 6841.32, 7122.80, 6931.33, 6572.99, 6415.37, 6580.19, 7009.22, 7116.75, 6761.65, 6453.94, 6351.36, 6618.34, 7119.45, 7099.47, 6664.13, 6408.81, 6311.69, 6545.42, 7093.02, 7173.01, 6772.74, 6478.13, 6375.77, 6590.64, 7060.24, 7177.37, 6855.14, 6538.23, 6386.54, 6391.94, 6482.09, 6588.15, 6680.40, 6747.56, 6828.08, 6980.71, 7366.25, 7608.43, 7269.67, 6805.23, 6555.08, 6591.96, 6959.07, 7101.67, 6796.81, 6507.56, 6382.85, 6650.44, 7154.05, 7109.07, 6689.00, 6425.72, 6408.29, 6806.89, 7163.97, 6946.42, 6590.98, 6414.04, 6431.35, 6837.48, 7202.47, 7007.91, 6611.40, 6391.39, 6433.63, 6830.97, 7182.68, 7020.37, 6643.53, 6420.18, 6333.00, 6639.62, 7212.93, 7221.21, 6736.38, 6399.35, 6302.45, 6477.75, 6990.83, 7276.87, 6972.96, 6558.96, 6341.33, 6343.28, 6720.07, 7183.38, 7121.80, 6693.53, 6369.22, 6252.79, 6532.11, 7089.37, 7167.38, 6723.19, 6393.84, 6231.08, 6213.96, 6690.08, 7227.70, 7075.26, 6577.73, 6254.92, 6163.24, 6225.03, 6622.28, 7248.14, 7276.30, 6774.22, 6436.34, 6300.44, 6218.47, 6436.05, 7064.12, 7379.96, 6986.48, 6497.82, 6311.84, 6289.84, 6346.28, 6737.00, 7331.82, 7412.90, 6952.78, 6499.91, 6299.16, 6281.16, 6409.20, 6865.41, 7264.81, 7088.78, 6696.90, 6443.46, 6351.10, 6538.03, 6974.59, 7154.37, 6845.83, 6502.92, 6364.60, 6412.90, 6797.81, 7151.40, 6959.81, 6586.94, 6388.49, 6345.15, 6673.49, 7191.73, 7221.81, 6833.99, 6530.48, 6407.16, 6522.79, 6937.96, 7209.60, 6964.02, 6608.48, 6474.95, 6442.71, 6485.87, 6623.42, 6734.49, 6786.31, 6842.01, 6896.58, 7012.50, 7368.71, 7656.16, 7430.98, 6978.33, 6677.88, 6592.26, 6834.04, 7155.08, 7025.92, 6672.16, 6523.05, 6495.14, 6766.31, 7235.89, 7196.97, 6781.17, 6541.01, 6481.74, 6749.89, 7193.26, 7176.03, 6830.24, 6603.03, 6676.62, 6893.51, 6843.28, 6666.07, 6625.93, 6647.05, 6693.92, 6758.41, 7106.44, 7625.99, 7550.85, 7011.08, 6617.54, 6420.75, 6536.07, 6978.72, 7192.38, 6982.99, 6710.46, 6554.86, 6570.03, 6918.05, 7335.33, 7239.70, 6829.13, 6556.35, 6434.84, 6709.42, 7266.14, 7296.51, 6854.06, 6568.23, 6493.41, 6534.28, 6670.10, 6803.41, 6880.34, 6916.53, 6936.82, 6970.43, 7202.34, 7659.50, 7761.09, 7302.02, 6826.10, 6587.65, 6618.97, 7015.29, 7283.29, 7041.05, 6741.11, 6602.96, 6632.33, 7038.58, 7359.16, 7104.46, 6702.03, 6509.23, 6592.99, 7040.58, 7364.33, 7126.86, 6740.84, 6531.38, 6605.69, 7047.11, 7287.74, 6945.83, 6534.10, 6382.52, 6562.19, 7116.81, 7385.19, 7003.79, 6558.13, 6362.76, 6399.98, 6909.12, 7436.45, 7206.22, 6661.12, 6354.61, 6262.93, 6631.80, 7350.37, 7499.05, 6911.75, 6366.42, 6167.66, 6126.98, 6189.89, 6372.64, 6536.83, 6651.48, 6746.05, 6798.14, 6980.46, 7575.99, 7960.74, 7425.48, 6662.60, 6286.62, 6168.42, 6541.30, 7254.12, 7283.12, 6686.54, 6306.04, 6183.75, 6254.86, 6908.66, 7576.90, 7296.52, 6614.52, 6237.14, 6156.15, 6388.80, 7024.42, 7451.40, 7054.62, 6469.47, 6242.28, 6196.07, 6404.25, 7056.04, 7455.60, 7069.48, 6517.84, 6214.55, 6124.77, 6378.74, 7057.23, 7460.59, 7052.01, 6480.42, 6190.84, 6132.75, 6337.07, 6940.94, 7482.88, 7235.17, 6632.18, 6299.78, 6222.83, 6309.29, 6483.99, 6618.29, 6686.69, 6776.43, 6862.54, 6903.57, 7238.58, 7828.81, 7850.35, 7230.13, 6661.06, 6406.46, 6463.14, 6913.31, 7274.81, 7044.87, 6673.09, 6500.63, 6556.02, 7010.35, 7287.77, 6925.06, 6491.59, 6252.56, 6401.55, 7001.88, 7207.81, 6820.54, 6463.29, 6307.21, 6623.71, 7175.85, 7102.82, 6665.00, 6405.83, 6311.79, 6711.50, 7308.38, 7210.04, 6696.18, 6391.52, 6346.40, 6762.55, 7300.53, 7154.46, 6633.65, 6367.26, 6367.32, 6803.35, 7338.59, 7141.32, 6593.98, 6358.01, 6310.98, 6692.73, 7355.69, 7335.24, 6798.83, 6451.05, 6306.37, 6564.64, 7124.23, 7194.63, 6807.20, 6502.13, 6432.35, 6719.88, 6919.25, 6702.21, 6490.97, 6421.88, 6445.01};
      int value_peak = detectPeaks(window_interval, size_split, sum_value_rr);
      sum_value_rr = value_peak + sum_value_rr;
      delay(10);
      //        Serial.print(window_interval[i] * 100);
      //        Serial.print(",");
      //      }
      //      for (int i = 0; i < size_split; i++) {
      //
      //      Serial.println();
      //detectPeaks(ppg_data, cnt_window);

      memset(window_interval, 0, sizeof(window_interval));
      cnt_window = 0;
      sum_rnd_select++;
      if (sum_rnd_select == 5) {
        //int cnt_ppg = 0;
        for (int i = 0; i < sum_value_rr; i++)
        {
          memset(RAW_VAL, 0, sizeof(RAW_VAL));
          sprintf(RAW_VAL, "%d,%d,%.2f,%d", 0, 0, rr_interval_data[i] * 100, i + 1);
          //Serial.println(RAW_VAL);
          if (deviceConnected) {
            std::string send_data(RAW_VAL, strlen(RAW_VAL));
            pCharacteristic->setValue(RAW_VAL);
            pCharacteristic->notify();
          } else {
            delay(500); // give the bluetooth stack the chance to get things ready
            ESP.restart();
            pServer->startAdvertising(); // restart advertising
            Serial.println("start advertising");
            oldDeviceConnected = deviceConnected;

          }
          delay(100);
        }
        memset(RAW_VAL, 0, sizeof(RAW_VAL));
        sprintf(RAW_VAL, "%d,%d,%.2f,%d", 0, 0, 0.0, 0);
        //Serial.println(RAW_VAL);
        if (deviceConnected) {
          std::string send_data(RAW_VAL, strlen(RAW_VAL));
          pCharacteristic->setValue(RAW_VAL);
          pCharacteristic->notify();
        } else {
          delay(500); // give the bluetooth stack the chance to get things ready
          ESP.restart();
          pServer->startAdvertising(); // restart advertising
          Serial.println("start advertising");
          oldDeviceConnected = deviceConnected;

        }
        break;
      }
    }
    // Your task code goes here
    // This will run for 5 minutes

    memset(RAW_VAL, 0, sizeof(RAW_VAL));
    sprintf(RAW_VAL, "%d,%d,%.2f,%d", GSR_val, count_gsr + 1, PPG_RAW,  count_gsr + 1);
    //Serial.println(analogRead(PPG));
    //Serial.println(PPG_val
    count_gsr++;
    //    Serial.print("RAW_VAL:\t");
    //    Serial.println(RAW_VAL);
    if (deviceConnected) {
      std::string send_data(RAW_VAL, strlen(RAW_VAL));
      pCharacteristic->setValue(RAW_VAL);
      pCharacteristic->notify();
    } else {
      delay(500); // give the bluetooth stack the chance to get things ready
      ESP.restart();
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;

    }
    delay(100); // Simulate some task execution
  }
  Serial.println("Task Stop");
  value_write = "";
}

int RunTime(uint16_t size__rr_interval) {
  unsigned long startTime = millis(); // Get the current time
  uint8_t sum_rnd_select = 0;
  uint16_t size_window_min = 5;
  int count_gsr = 0;
  uint16_t size_split = size__rr_interval / size_window_min;
  char RAW_VAL[50]; // Buffer to hold the converted string
  float window_interval[size_split];
  uint32_t cnt_window = 0;
  int sum_value_rr = 0;
  Serial.println("Task Start");
  // Run the task for 5 minutes
  while (millis() - startTime < RUNTIME_MS) {
    if (value_write == "stop")break;
    int GSR_val = analogRead(GSR);
    float PPG_RAW = analogRead(PPG) / 1.0;
    float PPG_val = butter.filter(analogRead(PPG));
    window_interval[cnt_window] = PPG_val;
    cnt_window++;
    //Serial.println(cnt_window);
    //Serial.println(size_window);
    if (cnt_window == size_split) {
      //float data_ppg[size_split] = {4698.69, 5995.37, 5832.58, 6147.94, 6870.84, 7093.32, 6708.36, 6265.88, 5994.30, 5979.66, 6386.61, 7023.29, 7155.74, 6711.43, 6346.34, 6279.78, 6435.79, 6878.18, 7151.07, 6880.50, 6564.17, 6470.68, 6462.00, 6693.47, 7050.12, 7030.65, 6675.10, 6388.80, 6324.30, 6575.68, 6134.63, 4548.01, 5458.27, 8790.91, 9581.54, 8129.59, 7290.71, 7159.49, 6862.23, 6420.57, 6219.76, 6197.54, 6233.11, 6307.19, 6425.91, 6530.75, 6628.63, 6755.57, 6837.62, 7105.95, 7509.75, 7413.42, 6936.59, 6634.57, 6536.61, 6727.90, 7088.11, 7077.35, 6758.09, 6483.07, 6330.34, 6550.90, 7049.38, 7145.91, 6781.70, 6460.17, 6342.03, 6493.77, 6908.34, 7109.23, 6861.28, 6555.07, 6463.94, 6672.89, 6947.64, 6873.91, 6641.36, 6575.36, 6603.06, 6653.76, 6943.05, 7285.65, 7183.15, 6823.10, 6584.57, 6506.31, 6690.94, 7029.03, 7067.27, 6795.06, 6564.61, 6502.55, 6738.52, 7131.67, 7116.44, 6748.72, 6503.23, 6427.78, 6686.02, 7161.64, 7169.30, 6818.14, 6584.87, 6451.16, 6649.87, 7094.93, 7163.45, 6916.44, 6627.40, 6351.45, 6472.64, 6935.87, 7045.96, 6740.18, 6463.96, 6362.35, 6530.10, 6981.37, 7205.80, 6924.94, 6574.50, 6478.36, 6716.31, 6982.59, 6839.13, 6534.11, 6408.17, 6433.95, 6773.61, 7290.43, 7285.86, 6853.79, 6569.49, 6456.93, 6680.29, 7096.63, 7053.25, 6699.24, 6458.62, 6378.35, 6684.27, 7133.12, 7085.37, 6724.97, 6487.33, 6419.80, 6671.31, 7074.18, 7059.81, 6702.14, 6435.23, 6372.75, 6675.72, 7157.74, 7151.94, 6746.78, 6502.21, 6464.90, 6732.24, 7108.29, 7006.75, 6654.40, 6496.39, 6589.35, 6737.88, 6648.41, 6474.84, 6445.86, 6481.63, 6660.93, 7170.62, 7475.92, 7142.10, 6690.56, 6445.51, 6489.47, 6865.58, 7064.93, 6846.29, 6573.42, 6440.48, 6630.13, 7068.07, 7156.00, 6822.00, 6537.61, 6476.60, 6718.76, 7081.09, 7057.52, 6745.57, 6536.35, 6538.65, 6841.32, 7122.80, 6931.33, 6572.99, 6415.37, 6580.19, 7009.22, 7116.75, 6761.65, 6453.94, 6351.36, 6618.34, 7119.45, 7099.47, 6664.13, 6408.81, 6311.69, 6545.42, 7093.02, 7173.01, 6772.74, 6478.13, 6375.77, 6590.64, 7060.24, 7177.37, 6855.14, 6538.23, 6386.54, 6391.94, 6482.09, 6588.15, 6680.40, 6747.56, 6828.08, 6980.71, 7366.25, 7608.43, 7269.67, 6805.23, 6555.08, 6591.96, 6959.07, 7101.67, 6796.81, 6507.56, 6382.85, 6650.44, 7154.05, 7109.07, 6689.00, 6425.72, 6408.29, 6806.89, 7163.97, 6946.42, 6590.98, 6414.04, 6431.35, 6837.48, 7202.47, 7007.91, 6611.40, 6391.39, 6433.63, 6830.97, 7182.68, 7020.37, 6643.53, 6420.18, 6333.00, 6639.62, 7212.93, 7221.21, 6736.38, 6399.35, 6302.45, 6477.75, 6990.83, 7276.87, 6972.96, 6558.96, 6341.33, 6343.28, 6720.07, 7183.38, 7121.80, 6693.53, 6369.22, 6252.79, 6532.11, 7089.37, 7167.38, 6723.19, 6393.84, 6231.08, 6213.96, 6690.08, 7227.70, 7075.26, 6577.73, 6254.92, 6163.24, 6225.03, 6622.28, 7248.14, 7276.30, 6774.22, 6436.34, 6300.44, 6218.47, 6436.05, 7064.12, 7379.96, 6986.48, 6497.82, 6311.84, 6289.84, 6346.28, 6737.00, 7331.82, 7412.90, 6952.78, 6499.91, 6299.16, 6281.16, 6409.20, 6865.41, 7264.81, 7088.78, 6696.90, 6443.46, 6351.10, 6538.03, 6974.59, 7154.37, 6845.83, 6502.92, 6364.60, 6412.90, 6797.81, 7151.40, 6959.81, 6586.94, 6388.49, 6345.15, 6673.49, 7191.73, 7221.81, 6833.99, 6530.48, 6407.16, 6522.79, 6937.96, 7209.60, 6964.02, 6608.48, 6474.95, 6442.71, 6485.87, 6623.42, 6734.49, 6786.31, 6842.01, 6896.58, 7012.50, 7368.71, 7656.16, 7430.98, 6978.33, 6677.88, 6592.26, 6834.04, 7155.08, 7025.92, 6672.16, 6523.05, 6495.14, 6766.31, 7235.89, 7196.97, 6781.17, 6541.01, 6481.74, 6749.89, 7193.26, 7176.03, 6830.24, 6603.03, 6676.62, 6893.51, 6843.28, 6666.07, 6625.93, 6647.05, 6693.92, 6758.41, 7106.44, 7625.99, 7550.85, 7011.08, 6617.54, 6420.75, 6536.07, 6978.72, 7192.38, 6982.99, 6710.46, 6554.86, 6570.03, 6918.05, 7335.33, 7239.70, 6829.13, 6556.35, 6434.84, 6709.42, 7266.14, 7296.51, 6854.06, 6568.23, 6493.41, 6534.28, 6670.10, 6803.41, 6880.34, 6916.53, 6936.82, 6970.43, 7202.34, 7659.50, 7761.09, 7302.02, 6826.10, 6587.65, 6618.97, 7015.29, 7283.29, 7041.05, 6741.11, 6602.96, 6632.33, 7038.58, 7359.16, 7104.46, 6702.03, 6509.23, 6592.99, 7040.58, 7364.33, 7126.86, 6740.84, 6531.38, 6605.69, 7047.11, 7287.74, 6945.83, 6534.10, 6382.52, 6562.19, 7116.81, 7385.19, 7003.79, 6558.13, 6362.76, 6399.98, 6909.12, 7436.45, 7206.22, 6661.12, 6354.61, 6262.93, 6631.80, 7350.37, 7499.05, 6911.75, 6366.42, 6167.66, 6126.98, 6189.89, 6372.64, 6536.83, 6651.48, 6746.05, 6798.14, 6980.46, 7575.99, 7960.74, 7425.48, 6662.60, 6286.62, 6168.42, 6541.30, 7254.12, 7283.12, 6686.54, 6306.04, 6183.75, 6254.86, 6908.66, 7576.90, 7296.52, 6614.52, 6237.14, 6156.15, 6388.80, 7024.42, 7451.40, 7054.62, 6469.47, 6242.28, 6196.07, 6404.25, 7056.04, 7455.60, 7069.48, 6517.84, 6214.55, 6124.77, 6378.74, 7057.23, 7460.59, 7052.01, 6480.42, 6190.84, 6132.75, 6337.07, 6940.94, 7482.88, 7235.17, 6632.18, 6299.78, 6222.83, 6309.29, 6483.99, 6618.29, 6686.69, 6776.43, 6862.54, 6903.57, 7238.58, 7828.81, 7850.35, 7230.13, 6661.06, 6406.46, 6463.14, 6913.31, 7274.81, 7044.87, 6673.09, 6500.63, 6556.02, 7010.35, 7287.77, 6925.06, 6491.59, 6252.56, 6401.55, 7001.88, 7207.81, 6820.54, 6463.29, 6307.21, 6623.71, 7175.85, 7102.82, 6665.00, 6405.83, 6311.79, 6711.50, 7308.38, 7210.04, 6696.18, 6391.52, 6346.40, 6762.55, 7300.53, 7154.46, 6633.65, 6367.26, 6367.32, 6803.35, 7338.59, 7141.32, 6593.98, 6358.01, 6310.98, 6692.73, 7355.69, 7335.24, 6798.83, 6451.05, 6306.37, 6564.64, 7124.23, 7194.63, 6807.20, 6502.13, 6432.35, 6719.88, 6919.25, 6702.21, 6490.97, 6421.88, 6445.01};
      int value_peak = detectPeaks(window_interval, size_split, sum_value_rr);
      sum_value_rr = value_peak + sum_value_rr;
      delay(10);
      //      for (int i = 0; i < size_split; i++) {
      //
      //        Serial.print(window_interval[i] * 100);
      //        Serial.print(",");
      //      }
      //Serial.println();
      //detectPeaks(ppg_data, cnt_window);
      memset(window_interval, 0, sizeof(window_interval));
      cnt_window = 0;
      sum_rnd_select++;
      if (sum_rnd_select == 5) {
        //int cnt_ppg = 0;
        for (int i = 0; i < sum_value_rr; i++)
        {
          memset(RAW_VAL, 0, sizeof(RAW_VAL));
          sprintf(RAW_VAL, "%d,%d,%.2f,%d", 0, 0, rr_interval_data[i] * 100, i + 1);
          //Serial.println(RAW_VAL);
          if (deviceConnected) {
            std::string send_data(RAW_VAL, strlen(RAW_VAL));
            pCharacteristic->setValue(RAW_VAL);
            pCharacteristic->notify();
          } else {
            delay(500); // give the bluetooth stack the chance to get things ready
            ESP.restart();
            pServer->startAdvertising(); // restart advertising
            Serial.println("start advertising");
            oldDeviceConnected = deviceConnected;

          }
          delay(100);
        }
        memset(RAW_VAL, 0, sizeof(RAW_VAL));
        sprintf(RAW_VAL, "%d,%d,%.2f,%d", 0, 0, 0.0, 0);
        //Serial.println(RAW_VAL);
        if (deviceConnected) {
          std::string send_data(RAW_VAL, strlen(RAW_VAL));
          pCharacteristic->setValue(RAW_VAL);
          pCharacteristic->notify();
        } else {
          delay(500); // give the bluetooth stack the chance to get things ready
          ESP.restart();
          pServer->startAdvertising(); // restart advertising
          Serial.println("start advertising");
          oldDeviceConnected = deviceConnected;

        }
        break;
      }
    }
    // Your task code goes here
    // This will run for 5 minutes

    memset(RAW_VAL, 0, sizeof(RAW_VAL));
    sprintf(RAW_VAL, "%d,%d,%.2f,%d", GSR_val, count_gsr + 1, PPG_RAW,  count_gsr + 1);
    count_gsr++;
    //    Serial.print("RAW_VAL:\t");
    //    Serial.println(RAW_VAL);
    if (deviceConnected) {
      std::string send_data(RAW_VAL, strlen(RAW_VAL));
      pCharacteristic->setValue(RAW_VAL);
      pCharacteristic->notify();
    } else {
      delay(500); // give the bluetooth stack the chance to get things ready
      ESP.restart();
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
    }
    delay(100); // Simulate some task execution
  }
  return sum_value_rr * 100;
  Serial.println("Task Stop");
  // Task finished, reset flag
  //taskRunning = false;
}


void setup() {

  Serial.begin(115200);
  delay(50);
  //Serial.println(sizeof(PPG_RAW));
  if (!EEPROM.begin(16)) {
    Serial.println("Failed to initialize EEPROM");
    while (1);
  }
  if (EEPROM.read(0) != 0x00) {
    for (int address = 0; address < EEPROM.length(); address++) {
      value_write += EEPROM.read(address); // Write 0 to each EEPROM address
    }
  }
  BLEDevice::init("Burapa-watch-3");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      //BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                      //BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  butter.init(SAMPLE_RATE, CUTOFF_FREQ);
  Serial.println("Waiting a client connection to notify...");

  // Set up the 15-minute interval timer
  //  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up
  //  timerAttachInterrupt(timer, &onTimer, true);
  //  timerAlarmWrite(timer, TASK_INTERVAL_MS, true);
  //  timerAlarmEnable(timer);

}

void loop() {
  //  // Print the value of the string
  if (deviceConnected) {
    if ((value_write != "") && (value_write != "start")) {
      //if (value_write == "cooldown")task(1210000, 12000);
      if (value_write == "sleep")task(320000, 3000);
      if (value_write == "sit")task(320000, 3000);
      if (value_write == "stand")task(320000, 3000);
      if (value_write == "run")task(320000, 3000);
      if (value_write == "post1")task(320000, 3000);
      if (value_write == "post2")task(320000, 3000);
      //Serial.println(value_write);
      //delay(20);

    } else if ((value_write != "") && (value_write == "start")) {
      Serial.println("runtime");
      //runtask 5 min every 10 min
      int time_late =  RunTime(3000);
      int time_wait = 600000 - time_late;
      unsigned long startTime = millis(); // Get the current time
      while (millis() - startTime < time_wait) {
        if (deviceConnected) {
          if (value_write == "stop") {
            value_write = "";
            break;
          }
        } if (!deviceConnected && oldDeviceConnected) {
          delay(500); // give the bluetooth stack the chance to get things ready
          ESP.restart();
          pServer->startAdvertising(); // restart advertising
          Serial.println("start advertising");
          oldDeviceConnected = deviceConnected;
        }
        delay(5000);
      }
      //delay(time_wait);
    } else {
      delay(10);
    }
  }

  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    ESP.restart();
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

  //
  //  // Delay for a short period to prevent flooding the serial port
  //  delay(1000);
  //  if (taskRunning) {
  //    // Task is already running, don't start another one
  //    return;
  //  }
  //
  //  // Wait for the 15-minute interval
  //  delay(TASK_INTERVAL_MS);
  //
  //  // Run the task
  //  task();
}
