The **controllable tractor** is a modified or purpose-built tractor that exposes its internal control mechanisms to the autonomous system through a well-defined interface. To ensure **transparency** and **universality**, the following requirements must be satisfied:

#### a. **Transparency**

- The tractor must provide **complete documentation** of its control interface.
    
- The **command set** (e.g. for throttle, steering, braking, gear shifting) must be openly accessible and well-documented.
    
- All **status and sensor feedback** must also be available for reading, including:
    
    - Engine status (RPM, temperature, fuel level)
        
    - Speed and acceleration
        
    - Gear position
        
    - PTO status
        
    - Hydraulic system state
        
    - Implement attachment feedback (if any)
        

#### b. **Universality**

- The control and feedback interface should comply with **industry-standard protocols** whenever possible.
    
- **Preferred control interfaces:**
    
    - **ISOBUS (ISO 11783)**: A widely adopted standard in agriculture for communication between tractors and implements. It defines:
        
        - Virtual Terminal (VT)
            
        - Task Controller (TC)
            
        - Tractor ECU (TECU)
            
    - **CANopen or J1939**: Common in industrial and automotive systems, especially for low-level actuator and sensor communication.
        
    - **MQTT/ROS2** (optional): For high-level planning/control when connected to edge or cloud systems.
        

#### c. **Control Path Examples**

|Function|Command Type|Suggested Protocol|
|---|---|---|
|Throttle Control|Set target RPM/speed|CANopen or J1939|
|Steering Control|Set steering angle/torque|CANopen or J1939|
|Brake Control|Set brake pressure|CANopen or J1939|
|Gear Control|Select gear|CANopen|
|PTO Control|Engage/disengage|ISOBUS TECU|
|Implement Control|Lift/lower, position|ISOBUS TC|

#### d. **Safety and Overrides**

- The system must support **manual override** at any time (emergency stop, mechanical brakes).
    
- Include **watchdog/reset interfaces** for CCU to relinquish control when health check fails.
    
- Implement **redundant feedback paths** (e.g. physical sensors + ECU signals).
    

#### e. **Preferred Implementation Notes**

- For retrofitting legacy tractors, consider using **actuator kits** with open interfaces (e.g. throttle-by-wire, steering motor with encoder feedback).
    
- Use **CAN gateways** or **signal converters** to expose legacy electrical signals to digital buses.


In a modular autonomous tractor system, the **control of mounted implements (挂载设备)** should be considered a **subsystem of the Controller**, specifically **integrated alongside the actuator and task execution layers**.



A traditional tractor (diesel or electric) modified to be externally controlled. It provides interfaces for:

- **Throttle and Brake Control**
    
- **Steering System**
    
- **Gear Shifting**
    
- **Power Take-Off (PTO) Operation**
    

The tractor must expose a standard control interface, such as **ISOBUS** or a custom actuator system, to allow external commands to actuate physical components.