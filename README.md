# 3D Object Scanner using VL53L0X, ESP32, and Stepper Motors

This is a low-cost 3D object scanner built using an ESP32 microcontroller, time of flight (ToF) based VL53L0X distance sensor, and stepper motors. The idea is to scan a small object placed on a turntable and generate a file containing its 3D shape in the form of (x, y, z) coordinates.

## How It Works

You place an object on a rotating platform (the turntable), which is turned using a stepper motor. A distance sensor mounted on a vertical rail moves upwards slice by slice. This upward movement is done using a threaded rod connected to another stepper motor.  

At each height level, the turntable rotates 360 degrees while the sensor records distance measurements from various angles. Then the sensor moves up slightly, and the process repeats. This continues until the entire object is scanned from bottom to top.

All the distance data is converted into 3D coordinates, creating a point cloud that represents the object's surface.

## Output

The result is a `.txt` file where each line represents a point in 3D space:

x y z

## Viewing the Scanned Object

The `.txt` file containing the 3D coordinates can be opened in **MeshLab**, a free 3D visualization software. In MeshLab, the point cloud can be viewed from different angles, and with the right settings, it can appear as a solid 3D model. This gives you a visual representation of the scanned object.

## Web Interface

The ESP32 also acts as a small web server. Once it's connected to Wi-Fi, you can open a browser and go to:

http://<ESP32_IP_ADDRESS>/

This opens a control page where you can:
- Start the scan
- Download the scan data directly to your computer as a `.txt` file

There’s no need to use serial monitors or connect cables to retrieve the data.

## Components Used

- ESP32 microcontroller  
- VL53L0X Time-of-Flight distance sensor  
- 2x stepper motors (NEMA 17 or similar)  
- 2x A4988 stepper motor drivers  
- Threaded rod for vertical movement  
- Custom-built 3D-Printed turntable  
- Wi-Fi network for web interface

## Use Cases

- Understanding how 3D scanning works  
- Embedded systems and motor control practice  
- Digitizing small objects into 3D space  
- Working with point clouds and basic 3D visualization

## Limitations

- Scanning resolution depends on the step size and sensor precision  
- Works best with matte, non-reflective, opaque objects  
- The scanner produces point clouds only; surface rendering is handled in MeshLab

## Team Members

- [Tejas Vijay Dahake](https://github.com/tejasd-24)  
- [Vaibhav Deora](https://github.com/Ramprashaddeora)  
- Saagarika Ural S  
- [Jayeshwar Pratap Singh Tanwar](https://github.com/Jpst01)

## License

This project is open-source and available under the MIT License.

