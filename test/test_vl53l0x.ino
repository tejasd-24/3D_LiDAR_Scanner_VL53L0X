// Simple program that just measures the distance

#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// The distance from the scanner to the rotation axis is fixed by the
// design of the scanner.
#define DIST_TO_AXIS    92 // Distance from sensor to axis = 92mm

// Holds the distance data
#define OUT_OF_RANGE (uint16_t) -1 // -1 means out of range

#define NUM_DIST 4

// Function to do one distance measurement
uint16_t GetDist() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.getSingleRangingMeasurement(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    return measure.RangeMilliMeter;
  } else {
    return OUT_OF_RANGE;
  }
//   return 0;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) // This waits for the serial interface to initialise
    delay(100);

  // Initialise the VL53L0X
  Serial.println("Initialising VL53L0X ...");
//  if (!lox.begin()) {
  if (!lox.begin(VL53L0X_I2C_ADDR, false, &Wire, lox.VL53L0X_SENSE_HIGH_ACCURACY)) {
    Serial.println(F("Failed to initialise VL53L0X"));
    vTaskSuspend(NULL);
  }

  Serial.println("Initialised");
}

// The loop just measures the distance and prints it to the serial monitor
void loop()
{
  uint16_t dist[NUM_DIST];

  // We measure the distance four times then print all 8
  // We do it this way to see if there is a limit to the measurement speed
  for (int i = 0; i < NUM_DIST; i++)
    dist[i] = GetDist();

  // Now print the measurements
  for (int i = 0; i < NUM_DIST; i++)
    Serial.printf("%d ", dist[i]);
  Serial.println("");

  delay(1000);
}