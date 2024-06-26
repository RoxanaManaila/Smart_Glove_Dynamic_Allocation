#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

// Define the number of sensors
const int numSensors = 5;
int* flexSensors = nullptr;

// Define reference voltage
const float VCC = 5;     

// Fixed resistor value
const float rezistor_fix = 10000.0; 

// Filtering parameters
const float alpha = 0.5;
float* filteredAccel = nullptr;

String text = ""; 
String textInit = "";
String eroare = "EROARE";
char lastChar = '\0';

void setup() {
  // Dynamically allocate memory for the sensor readings
  flexSensors = new int[numSensors];
  
  // Dynamically allocate memory for filtered accelerometer values
  filteredAccel = new float[3];
  for (int i = 0; i < 3; ++i) {
    filteredAccel[i] = 0;
  }

  // Configure sensor pins
  for (int i = 0; i < numSensors; ++i) {
    pinMode(A0 + i, INPUT);
  }

  Serial.begin(9600);

  // Wait for serial port to be available
  while (!Serial);

  // Initialize I2C communication with MPU6050
  Wire.begin();

  // Check if MPU6050 is connected
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  // Set MPU6050 accelerometer and gyro range and filter bandwidth
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // Read ADC values for each sensor
  for (int i = 0; i < numSensors; ++i) {
    flexSensors[i] = analogRead(A0 + i);
  }

  // Calculate voltages and resistances for each sensor
  float flexVoltages[numSensors];
  float flexResistances[numSensors];
  for (int i = 0; i < numSensors; ++i) {
    flexVoltages[i] = flexSensors[i] * VCC / 1023.0;
    flexResistances[i] = (rezistor_fix * (VCC / flexVoltages[i] - 1.0)) / 1000;
  }

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Apply low-pass filter to accelerometer data
  filteredAccel[0] = alpha * filteredAccel[0] + (1 - alpha) * accel.acceleration.x;
  filteredAccel[1] = alpha * filteredAccel[1] + (1 - alpha) * accel.acceleration.y;
  filteredAccel[2] = alpha * filteredAccel[2] + (1 - alpha) * accel.acceleration.z;

  textInit = text;

  // Decision making based on sensor readings and filtered accelerometer data
  if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
      flexResistances[2] > 2 && flexResistances[2] < 3 && flexResistances[3] > 2 && flexResistances[3] < 3.07 &&
      flexResistances[4] > 3 && flexResistances[4] < 4 && filteredAccel[1] > 0 && filteredAccel[2] > 0) {
    text += 'A';
    lastChar = 'A';
  } else if (flexResistances[0] > 2 && flexResistances[0] < 3 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 3 && flexResistances[2] < 4 && flexResistances[3] > 3 && flexResistances[3] < 4 &&
             flexResistances[4] > 3 && flexResistances[4] < 4.20) {
    text += 'B';
    lastChar = 'B';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 2 && flexResistances[2] < 3 && flexResistances[3] > 3 && flexResistances[3] < 4 &&
             flexResistances[4] > 3 && flexResistances[4] < 4 && filteredAccel[2] > 0) {
    text += 'C';
    lastChar = 'C';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 3 && flexResistances[4] < 4 && filteredAccel[0] > 0 && filteredAccel[2] < 0) {
    text += 'D';
    lastChar = 'D';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 2 && flexResistances[4] < 3 && filteredAccel[2] < 0 && filteredAccel[2] > -4) {
    text += 'E';
    lastChar = 'E';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 3 && flexResistances[2] < 4 && flexResistances[3] > 3 && flexResistances[3] < 4 &&
             flexResistances[4] > 3 && flexResistances[4] < 4) {
    text += 'F';
    lastChar = 'F';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 2 && flexResistances[4] < 3 && filteredAccel[2] < 0) {
    text += 'G';
    lastChar = 'G';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 3 && flexResistances[2] < 4 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 2 && flexResistances[4] < 3 && filteredAccel[2] < 0) {
    text += 'H';
    lastChar = 'H';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 4 && flexResistances[4] < 5 && filteredAccel[2] > 0) {
    text += 'I';
    lastChar = 'I';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 4 && flexResistances[4] < 5 && filteredAccel[0] > 0) {
    text += 'J';
    lastChar = 'J';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 2 && flexResistances[2] < 3 && flexResistances[3] > 3 && flexResistances[3] < 4 &&
             flexResistances[4] > 4 && flexResistances[4] < 5 && filteredAccel[2] < 0 && filteredAccel[0] > 0) {
    text += 'K';
    lastChar = 'K';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 2 && flexResistances[2] < 3 && flexResistances[3] > 3 && flexResistances[3] < 4 &&
             flexResistances[4] > 4 && flexResistances[4] < 5 && filteredAccel[2] < 0) {
    text += 'L';
    lastChar = 'L';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 3 && flexResistances[2] < 4 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 4 && flexResistances[4] < 5) {
    text += 'M';
    lastChar = 'M';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 1 && flexResistances[3] < 2 &&
             flexResistances[4] > 3 && flexResistances[4] < 4) {
    text += 'N';
    lastChar = 'N';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 2 && flexResistances[2] < 3 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 4 && flexResistances[4] < 5) {
    text += 'O';
    lastChar = 'O';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 2 && flexResistances[4] < 3 && filteredAccel[2] < 0 && filteredAccel[0] > 0) {
    text += 'P';
    lastChar = 'P';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 2 && flexResistances[2] < 3 && flexResistances[3] > 3 && flexResistances[3] < 4 &&
             flexResistances[4] > 2 && flexResistances[4] < 3 && filteredAccel[2] < 0 && filteredAccel[1] > 0) {
    text += 'Q';
    lastChar = 'Q';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 3 && flexResistances[4] < 4 && filteredAccel[2] < 0) {
    text += 'R';
    lastChar = 'R';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 3 && flexResistances[4] < 4 && filteredAccel[2] < 0) {
    text += 'S';
    lastChar = 'S';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 3 && flexResistances[4] < 4 && filteredAccel[2] < 0 && filteredAccel[2] > -4) {
    text += 'T';
    lastChar = 'T';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 3 && flexResistances[4] < 4 && filteredAccel[2] > 0) {
    text += 'U';
    lastChar = 'U';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 3 && flexResistances[3] < 4 &&
             flexResistances[4] > 4 && flexResistances[4] < 5 && filteredAccel[2] > 0) {
    text += 'V';
    lastChar = 'V';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 3 && flexResistances[3] < 4 &&
             flexResistances[4] > 4 && flexResistances[4] < 5) {
    text += 'W';
    lastChar = 'W';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 2 && flexResistances[2] < 3 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 4 && flexResistances[4] < 5 && filteredAccel[2] < 0) {
    text += 'X';
    lastChar = 'X';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 1 && flexResistances[1] < 2 &&
             flexResistances[2] > 2 && flexResistances[2] < 3 && flexResistances[3] > 3 && flexResistances[3] < 4 &&
             flexResistances[4] > 4 && flexResistances[4] < 5 && filteredAccel[2] > 0) {
    text += 'Y';
    lastChar = 'Y';
  } else if (flexResistances[0] > 3 && flexResistances[0] < 4 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
             flexResistances[2] > 1 && flexResistances[2] < 2 && flexResistances[3] > 2 && flexResistances[3] < 3 &&
             flexResistances[4] > 4 && flexResistances[4] < 5 && filteredAccel[2] > 0) {
    text += 'Z';
    lastChar = 'Z';
  } // Additional gestures for deleting the last character and adding a white space character
    else if (flexResistances[0] > 4 && flexResistances[0] < 5 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
    flexResistances[2] > 3 && flexResistances[2] < 4 && flexResistances[3] > 3 && flexResistances[3] < 4 &&
    flexResistances[4] > 4 && flexResistances[4] < 5 && filteredAccel[0] > 0 && filteredAccel[1] > 0 && filteredAccel[2] > 0 && lastChar != ' ') {
    // Add a white space character
    text += ' ';
    lastChar = ' ';
    }
    else if (flexResistances[0] > 3.80 && flexResistances[0] < 5 && flexResistances[1] > 2 && flexResistances[1] < 3 &&
           flexResistances[2] > 3 && flexResistances[2] < 4 && flexResistances[3] > 3 && flexResistances[3] < 4 &&
           flexResistances[4] > 3 && flexResistances[4] < 5 && filteredAccel[0] < 0 && filteredAccel[1] < 0 && filteredAccel[2] < 0) {
    // Delete the last character
    if (text.length() > 0) {
        text.remove(text.length() - 1);
    }
  }
  else{
    text = eroare;
    lastChar = '\0';
  }

  // Output the results
  Serial.print("Flex sensor values: ");
  for (int i = 0; i < numSensors; ++i) {
    Serial.print(flexSensors[i]);
    if (i < numSensors - 1) Serial.print(", ");
  }
  Serial.println();

  Serial.print("Flex resistances: ");
  for (int i = 0; i < numSensors; ++i) {
    Serial.print(flexResistances[i]);
    if (i < numSensors - 1) Serial.print(", ");
  }
  Serial.println();

  Serial.print("Filtered accelerometer data: ");
  Serial.print(filteredAccel[0]);
  Serial.print(", ");
  Serial.print(filteredAccel[1]);
  Serial.print(", ");
  Serial.print(filteredAccel[2]);
  Serial.println();

  Serial.print("Current text: ");
  Serial.println(text);

  // Small delay before next loop
  delay(500);
}

// Clean up allocated memory on exit (though this is rarely necessary in Arduino sketches)
void cleanup() {
  if (flexSensors) {
    delete[] flexSensors;
  }
  if (filteredAccel) {
    delete[] filteredAccel;
  }
}
