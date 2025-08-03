// *******************************************************
// LIDAR scanner
// *******************************************************

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

// Use high res mode
// #define HIGHRESMODE 1

#include <Adafruit_VL53L0X.h>
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// *******************************************************
// Pin definitions and scan configuration
// *******************************************************

// Motor control
#define STEP_PULSE_DELAY 1000 // 1000 microseconds.

// Horizontal rotation
#define ENABLE_H 14
#define DIR_H    12 // Pin for rotation direction
#define STEP_H   13 // Pin to rotate horizontally
#define DELAY_H  20 // We need a delay when rotating or the platform slips

// Vertical motion
#define ENABLE_V 26
#define DIR_V    25 // Pin for vertical direction
#define STEP_V   33 // Pin to step vertically

#define STEPS_PER_REV 200     // 200 steps = 1 revolution
#define POINTS_PER_REV_H 50 // must be a factor of STEPS_PER_REV
#define POINTS_PER_REV_V 1   // must be a factor of STEPS_PER_REV

// Each complete revolution is 0.9cm
#define REVS_VERTICAL   18 // 18 slices = (18 - 1)*0.9 = 15cm
#define POINTS_VERTICAL REVS_VERTICAL*POINTS_PER_REV_V
#define DIST_TO_AXIS    92 // Distance from sensor to axis = 92mm

// Holds the distance data
#define OUT_OF_RANGE (uint16_t) -1 // -1 means out of range
uint16_t dist[POINTS_VERTICAL][POINTS_PER_REV_H];

// *******************************************************
// We use the ESP32 softap as a way of controlling the
// scanner.
// *******************************************************

// WiFi credentials
#define SSID     "LIDAR"
#define HOSTNAME "lidar"

// Use a global variable for the server since there is only one server
WebServer server(80);

void ConnectWiFi() {
  Serial.println("Setting up the Access Point");
  WiFi.setHostname(HOSTNAME);

  // Start the AP
  WiFi.softAP(SSID);
  IPAddress IP = WiFi.softAPIP();
  Serial.printf("Access point SSID = %s, name = %s, IP = %s, \n", SSID, HOSTNAME, IP.toString().c_str());

  // Set the paths we will handle
  server.on("/", onRoot);
  server.on("/scan", onScan);
  server.on("/data", onData);
  server.on("/polar", onDataPolar);
  server.on("/raw", onDataRaw);

  // Start the HTTP server
  server.begin();
}

// Display the control page
void onRoot() {
  String response = R"(
    <!DOCTYPE html><html>
      <body>
        <h1>LIDAR scanner</h1>
        <p><a href="/scan" target="_blank">Start scan</a></p>
        <p><a href="/data" target="_blank">Download data - Cartesian</a></p>
        <p><a href="/polar" target="_blank">Download data - Cylindrical</a></p>
        <p><a href="/raw" target="_blank">Download data - Raw format</a></p>
      </body>
    </html>
  )";
  server.send(200, "text/html", response);
}

// Do a scan
void onScan() {
  // Send the "scan started" response
  String response = R"(
    <!DOCTYPE html><html>
      <body>
        <h1>LIDAR scanner</h1>
        <p>Scan started</p>
      </body>
    </html>
  )";
  server.send(200, "text/html", response);

  // Do the scan
  DoScan();
}

// Download data as cylindrical coordinates
void onData() {
  // Append all the data to a string
  String response = "x,y,z\r\n";
  double x, y, z;

  double vstep = 9.0/POINTS_PER_REV_V;
  for (int n_v = 0; n_v < POINTS_VERTICAL; n_v++) {
    z = n_v*vstep;
    double hstep = 0.0314159*STEPS_PER_REV/POINTS_PER_REV_H;

    for (int n_h = 0; n_h < POINTS_PER_REV_H; n_h++) {
      double r, theta;

      // Ignore out of range points
      if (dist[n_v][n_h] != OUT_OF_RANGE) {
        // The distance from the axis is the sensor to axis distance
        // minus the distance from the sensor to the object
        r = DIST_TO_AXIS - dist[n_v][n_h];
        theta = n_h*hstep;
        // Now do the polar to Cartesian conversion
        x = r*cos(theta);
        y = r*sin(theta);
        response += String(x) + "," + String(y) + "," + String(z) + "\r\n";
      }
    }
  }

  // Send the response
  server.send(200, "text/plain", response);
}

// Download data as cylindrical coordinates
void onDataPolar () {
  // Append all the data to a string
  String response = "angle,distance,z\r\n";
  double vstep = 9.0/POINTS_PER_REV_V;
  for (int n_v = 0; n_v < POINTS_VERTICAL; n_v++) {
    double z = n_v*vstep;
    double hstep = 0.0314159*STEPS_PER_REV/POINTS_PER_REV_H;

    for (int n_h = 0; n_h < POINTS_PER_REV_H; n_h++) {
      double r, theta;

      // Ignore out of range points
      if (dist[n_v][n_h] != OUT_OF_RANGE) {
        r = DIST_TO_AXIS - dist[n_v][n_h];
        theta = n_h*hstep;
        response += String(theta) + "," + String(r) + "," + String(z) + "\r\n";
      }
    }
  }

  // Send the response
  server.send(200, "text/plain", response);
}

// Download raw data
void onDataRaw () {
  // Append all the data to a string
  String response = "Raw data\r\nangle,distance,z\r\n";
  double vstep = 9.0/POINTS_PER_REV_V;
  for (int n_v = 0; n_v < POINTS_VERTICAL; n_v++) {
    double z = n_v*vstep;
    double hstep = 0.0314159*STEPS_PER_REV/POINTS_PER_REV_H;

    for (int n_h = 0; n_h < POINTS_PER_REV_H; n_h++) {
      double r, theta;

      // Ignore out of range points
      if (dist[n_v][n_h] != OUT_OF_RANGE) {
        r = dist[n_v][n_h];
        theta = n_h*hstep;
        response += String(theta) + "," + String(r) + "," + String(z) + "\r\n";
      }
    }
  }

  // Send the response
  server.send(200, "text/plain", response);
}

// *******************************************************
// Function to do one distance measurement
// *******************************************************
uint16_t GetDist() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    return measure.RangeMilliMeter;
  } else {
    return OUT_OF_RANGE;
  }
}

// *******************************************************
// Fuctions to move the motors
// *******************************************************
void StepVertical() {
  for (int n_v = 0; n_v < STEPS_PER_REV/POINTS_PER_REV_V; n_v++) {
    digitalWrite(STEP_V, HIGH);
    delayMicroseconds(STEP_PULSE_DELAY);
    digitalWrite(STEP_V, LOW);
    delayMicroseconds(STEP_PULSE_DELAY);
  }
}

void StepHorizontal() {
  for (int n_h = 0; n_h < STEPS_PER_REV/POINTS_PER_REV_H; n_h++) {
    digitalWrite(STEP_H, HIGH);
    delayMicroseconds(STEP_PULSE_DELAY);
    digitalWrite(STEP_H, LOW);
    delayMicroseconds(STEP_PULSE_DELAY);

    // We need a delay otherwise the platform slips
    delay(DELAY_H);
  }
}

// *******************************************************
// Save the data to a file
// For now it just prints data to the serial port for checking
// *******************************************************
void SaveToFile() {
  double x, y, z;

  double vstep = 9.0/POINTS_PER_REV_V;
  for (int n_v = 0; n_v < POINTS_VERTICAL; n_v++) {
    z = n_v*vstep;

    // For the angle, each step is 1/200 of 2pi. There are
    // POINTS_PER_REV_H measurements each full revolution, so angle
    // step is (2pi/200) * 200 / no. measurements
    double hstep = 0.0314159*STEPS_PER_REV/POINTS_PER_REV_H;

    for (int n_h = 0; n_h < POINTS_PER_REV_H; n_h++) {
      double r, theta;

      // Ignore out of range points
      if (dist[n_v][n_h] != OUT_OF_RANGE) {
        // The distance from the axis is the sensor to axis distance
        // minus the distance from the sensor to the object
        r = DIST_TO_AXIS - dist[n_v][n_h];
        theta = n_h*hstep;
        // Now do the polar to Cartesian conversion
        x = r*cos(theta);
        y = r*sin(theta);
        // The full version of this will write the text to a file
        // For now we'll just print it to the serial port
        Serial.printf("%f %f %f\n", x, y, z);
      }
    }
  }
}

// *******************************************************
// Do the scan
// *******************************************************
void DoScan() {
  // The outer loop steps in the vertical direction
  // Direction pin low means move up
  Serial.println("Starting vertical scan");
  digitalWrite(DIR_V, LOW);

  for (int n_v = 0; n_v < POINTS_VERTICAL; n_v++) {
    Serial.printf("Vertical step %d\n", n_v);

    // The inner loop does the rotation
    for (int n_h = 0; n_h < POINTS_PER_REV_H; n_h++) {
      dist[n_v][n_h] = GetDist(); // The GetDist() function returns the distance
      StepHorizontal();
    }

    // Finished the horizontal scan so step vertically
    StepVertical();
  }

  // Wind the vertical platform back down again
  // Direction pin high means move down
  Serial.println("Winding platform down");
  digitalWrite(DIR_V, HIGH);
  for (int n_v = 0; n_v < POINTS_VERTICAL; n_v++) {
    StepVertical();
  }

  // Save the data
  Serial.println("Finished scan");
  SaveToFile();
}

// *******************************************************
// Arduino interface functions
// *******************************************************
void setup()
{
  Serial.begin(115200);
  while (!Serial) // This waits for the serial interface to initialise
    delay(1000);
  delay(2000);
  Serial.println("LIDAR scanner starting");

  // Stepper motor pins
  pinMode(ENABLE_H, OUTPUT);
  pinMode(DIR_H,    OUTPUT);
  pinMode(STEP_H,   OUTPUT);
  pinMode(ENABLE_H, OUTPUT);
  pinMode(DIR_V,    OUTPUT);
  pinMode(STEP_V,   OUTPUT);

  // Enable (Active Low)
  digitalWrite(ENABLE_H, LOW);
  digitalWrite(ENABLE_V, LOW);

  // The turntable only rotates in one direction
  digitalWrite(DIR_H, HIGH);

  // Initialise the wifi
  ConnectWiFi();

  // Initialise the VL53L0X
  Serial.println("Initialising VL53L0X ...");
// This define controls whether we use hires mode
#ifdef HIGHRESMODE
  Serial.println("Using high precision mode");
  if (!lox.begin(VL53L0X_I2C_ADDR, false, &Wire, lox.VL53L0X_SENSE_HIGH_ACCURACY)) {
#else
  Serial.println("Using standard precision mode");
  if (!lox.begin()) {
#endif
    Serial.println(F("Failed to initialise VL53L0X"));
    vTaskSuspend(NULL);
  }

  // Print the config
  Serial.println("Initialised");
  Serial.printf("Vertical step size = %.1f mm\n", 9.0/POINTS_PER_REV_V);
  Serial.printf("No. vertical points = %d\n", POINTS_VERTICAL);
  Serial.printf("No. horizontal steps = %d\n", POINTS_PER_REV_H);

  // Do a scan as soon as we start
  DoScan();
}

void loop()
{
  // Listen for commands
  server.handleClient();
  delay(100);
}