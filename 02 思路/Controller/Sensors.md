Sensors are responsible for perceiving both the **external environment** and the **tractor’s internal operational state**. The data collected is processed by the Central Control Unit (CCU) for **sensor fusion** and **state estimation**, providing a unified, accurate picture of the system’s position, movement, and surroundings.

---

## 1. Environmental Sensors

These sensors detect external conditions, obstacles, and positioning data necessary for navigation and safety.

|Sensor Type|Purpose|
|---|---|
|**GNSS/RTK**|High-precision positioning (centimeter-level accuracy).|
|**Cameras**|Visual perception for object detection, classification, and row-following.|
|**LiDAR / Radar**|Obstacle detection and distance measurement, especially in low visibility conditions.|
|**Weather Sensors**|Measure wind speed/direction, humidity, temperature, and light intensity for operation planning and safety.|

---

## 2. State Sensors

These sensors monitor the tractor’s mechanical status and implement operation.

|Sensor Type|Purpose|
|---|---|
|**IMU (Inertial Measurement Unit)**|Provides orientation, acceleration, and angular velocity for navigation and stability control.|
|**Engine/Powertrain Sensors**|Engine RPM, vehicle speed, fuel level, oil temperature, and other performance parameters.|
|**Steering Angle Sensor**|Measures the current steering wheel or actuator angle for precise path tracking.|
|**Brake Pressure Sensor**|Monitors braking force and confirms actuation.|
|**Implement Feedback Sensors**|Detects the position, depth, and operational status of mounted implements (e.g., plow height).|

---

## 3. Sensor Fusion and State Estimation

The CCU integrates data from multiple sensors to create a **reliable, real-time state model**:

- **GNSS + RTK Module** – Provides absolute position with centimeter-level accuracy.
    
- **IMU** – Supplies relative movement data for high-frequency updates between GNSS fixes.
    
- **Vision + LiDAR/Radar** – Builds a 3D environmental map and identifies obstacles.
    
- **Vehicle State Sensors** – Feed internal operation data into the motion control algorithms.
    

The fusion process uses algorithms such as **Extended Kalman Filters (EKF)** or **Factor Graph Optimization** to combine absolute and relative positioning data, ensuring accuracy even when individual sensors experience dropouts or noise.

