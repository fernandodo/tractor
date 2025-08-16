## Workspace, Package and Node

如果把 **workspace**（工作空间）比作**一个完整的机器人**，可以这样类比：

---

### 1. **Workspace = 机器人本体**

- 里面包含了机器人需要的全部**功能模块**（package）和它们之间的依赖关系。
    
- 就像机器人的所有部件和系统组合在一起，形成一个整体。
    
- 你在一个 workspace 里 build，就像一次性组装整个机器人。
    

---

### 2. **Package = 功能系统/子系统**

- 每个 package 就像机器人上的一个**独立功能模块**：
    
    - 传感器驱动（摄像头驱动包）
        
    - 电机控制（底盘控制包）
        
    - 路径规划（导航包）
        
    - 交互接口（语音控制包）
        
- 它们是**独立开发、独立构建、可以单独复用**的单元。
    
- 一个机器人可以有多个 package，不同 package 也能在其他机器人项目中复用。
    

---

### 3. **Node = 模块里的具体执行单元**

- node 是运行时的**进程**（可执行程序），执行某个具体任务。
    
- 在一个 package 内，可以有一个或多个 node：
    
    - 摄像头驱动包 → `camera_node`
        
    - 底盘控制包 → `motor_controller_node`
        
    - 导航包 → `planner_node`、`map_server_node`
        
- node 之间通过 ROS 2 的话题、服务、动作进行通信。
    

---

### 4. 形象类比

假设一个机器人是一个**自动送货机器人**：

- **Workspace** = 整个机器人
    
- **Package** = 各功能子系统
    
    - “视觉系统”包
        
    - “导航系统”包
        
    - “动力系统”包
        
- **Node** = 子系统里的具体运行部件
    
    - 摄像头采集节点
        
    - 图像识别节点
        
    - 路径规划节点
        
    - 电机驱动节点
        

---

📌 **总结类比表**

|ROS 2 概念|机器人类比|特点|
|---|---|---|
|Workspace|机器人整体|包含所有功能模块（package），一次性构建和部署|
|Package|功能子系统|自成一体，可复用，可包含多个 node|
|Node|功能部件|运行中的程序实例，完成具体任务|

---

如果你愿意，我可以帮你画一个**“Workspace → Package → Node”关系图**，用机器人例子直观展示它们的层级关系，这样就更容易记。

## `std::shared_ptr`
The main reason ROS 2 uses `std::shared_ptr` for nodes is because of **shared ownership**.

---

## 🔑 Key feature that matters here

**`std::shared_ptr` allows multiple parts of the program to hold and use the same object, and the object will be destroyed only when the last owner is gone.**

---

### Why this is important for ROS 2 nodes

1. **`rclcpp::spin()` needs to keep the node alive**
    
    - When you pass your node to `rclcpp::spin(node)`, ROS 2 internally stores another `shared_ptr` to it.
        
    - This ensures the node stays alive **even if the variable in `main()` goes out of scope** while spinning.
        
2. **Callbacks and timers may hold their own references**
    
    - Timers, subscriptions, services, and actions often store a reference to the node internally.
        
    - Those components can hold a `std::shared_ptr` to the node so it won't be destroyed while callbacks are still pending.
        
3. **Automatic lifetime management**
    
    - No need to manually `delete` the node.
        
    - Once all `shared_ptr` copies are gone (from `main`, from `spin`, from subscriptions, etc.), the node is automatically destroyed.
        

---

### Without `shared_ptr` (e.g., using a raw pointer)

If you wrote:

```cpp
MyCustomNode* node = new MyCustomNode();
rclcpp::spin(node);
delete node;
```

You'd have to carefully manage when to delete it, and if `spin()` or any callback stored a raw pointer past that time, you'd risk **dangling pointers** and crashes.

---

**In short:**  
`std::shared_ptr` works here because **multiple owners** (main function, ROS 2 executor, subscriptions, timers, etc.) can all keep the node alive safely until no one needs it anymore. That’s exactly the lifetime model a ROS 2 node requires.

---

If you want, I can show you **a real ROS 2 call chain** where the executor actually increments the shared_ptr use count when you call `spin()`. That makes it crystal clear why raw pointers wouldn’t be safe. Would you like to see that?

## Why std::bind(&MyCustomNode::timer_callback, this))`

The short answer:  
You **can’t** just pass `&MyCustomNode::timer_callback` because **a pointer to a non-static member function** is **not callable by itself** — it needs an **object instance** to work on.

---

## 1️⃣ Why it doesn’t work directly

- `&MyCustomNode::timer_callback` is a **member function pointer**.
    
- Its type is something like:
    
    ```cpp
    void (MyCustomNode::*)()
    ```
    
- This type is **not** the same as a plain function pointer:
    
    ```cpp
    void (*)()
    ```
    
- When you pass just the member function pointer, the compiler doesn’t know **which object** it should run on — you have to tell it which `MyCustomNode` instance (`this`) to use.
    

---

## 2️⃣ Why `std::bind` (or a lambda) fixes it

`std::bind(&MyCustomNode::timer_callback, this)`

- Takes the member function pointer **and** the object instance (`this`)
    
- Produces a **callable object** (functor) that matches the callback type expected by `create_wall_timer`:
    
    ```cpp
    std::function<void()>
    ```
    
- Now ROS 2’s timer system can call it without worrying about which object it belongs to.
    

---

## 3️⃣ Alternative: Lambda

Instead of `std::bind`, you can write:

```cpp
timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() { this->timer_callback(); }
);
```

This does the same thing — it **captures** `this` and calls the member function.

---

## 4️⃣ Why ROS 2 examples often use `std::bind`

- Works well with callbacks that take arguments (bind can pre-fill them).
    
- Older C++11 ROS 2 code often used `std::bind` for consistency.
    
- Lambdas are shorter for simple cases, but `std::bind` is still common in tutorials and docs.
    

---

💡 **Key point:**  
`&MyCustomNode::timer_callback` **alone** is just a “method address.”  
`std::bind(..., this)` or a lambda “attaches” that address to an object instance, making it callable in the form ROS 2’s API expects.

---

If you want, I can diagram the **types** involved here so you can see exactly why the compiler complains when you try to pass the raw method pointer.  
Do you want me to show that?


## Summary of **ROS 2 `source setup`**, including bash shortcuts and VS Code integration:

---

### **1. Setting a Bash Shortcut for ROS 2 Source Setup**

When using ROS 2, you must run the setup script so ROS 2 environment variables are loaded into your shell.  
Normally:

```bash
source /opt/ros/<ros2_distro>/setup.bash
```

This becomes repetitive, so you can make a shortcut:

#### **Option A – Add to `.bashrc` (Auto-load for all shells)**

```bash
echo "source /opt/ros/<ros2_distro>/setup.bash" >> ~/.bashrc
```

Then reload:

```bash
source ~/.bashrc
```

#### **Option B – Create a Custom Alias (Manual trigger)**

In `~/.bashrc`:

```bash
alias ros2env="source /opt/ros/<ros2_distro>/setup.bash"
```

Usage:

```bash
ros2env
```

> Replace `<ros2_distro>` with your installed distribution, e.g. `humble`, `jazzy`.

---

### **2. VS Code Setup for ROS 2**

#### **Global Setup (Once per System/User)**

1. **Install Extensions**
    
    - **ROS** (ms-iot.vscode-ros)
        
    - **C/C++** (ms-vscode.cpptools)
        
    - **Python** (ms-python.python) – if using Python nodes
        
    - **CMake Tools** (ms-vscode.cmake-tools) – for C++ builds
        
    - **colcon helper** (optional)
        
2. **Add ROS 2 Environment to VS Code Terminal**
    
    - In VS Code, go to `Settings → Features → Terminal → Integrated > Env: Linux`
        
    - Add:
        
        ```json
        {
          "ROS_DISTRO": "jazzy",
          "BASH_ENV": "/home/<user>/.bashrc"
        }
        ```
        
        (ensures all integrated terminals have ROS 2 sourced)
        

---

#### **Per-Project Setup**

1. **Source Both Global ROS 2 and Project Workspaces**  
    In the project’s `.vscode/settings.json`:
    
    ```json
    {
      "terminal.integrated.profiles.linux": {
        "bash (ROS2)": {
          "path": "bash",
          "args": ["-c", "source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec bash"] 这里不太对，应该是条件判断
        }
      },
      "terminal.integrated.defaultProfile.linux": "bash (ROS2)"
    }
    ```
    
    This ensures your workspace overlay is active when using the VS Code terminal.
    
2. **Configure CMake Tools for ROS 2**  
    In `.vscode/settings.json`:
    
    ```json
    {
      "cmake.generator": "Ninja",
      "cmake.buildDirectory": "${workspaceFolder}/build",
      "cmake.configureArgs": [
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
      ],
      "C_Cpp.default.compileCommands": "${workspaceFolder}/build/compile_commands.json"
    }
    ```
    
3. **Debugging**
    
    - For Python nodes: set `"program": "${workspaceFolder}/path/to/script.py"` in `launch.json`.
        
    - For C++ nodes: set `"miDebuggerPath": "/usr/bin/gdb"` and `"program": "${workspaceFolder}/install/<package>/lib/<package>/<node>"`.
        

