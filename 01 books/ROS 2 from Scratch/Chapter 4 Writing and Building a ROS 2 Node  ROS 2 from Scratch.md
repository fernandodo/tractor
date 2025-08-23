---
created: 2025-08-13T20:25:48 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_04.xhtml#_idParaDest-87
author: 
---
.0000000000b
# Chapter 4: Writing and Building a ROS 2 Node | ROS 2 from Scratch

> ## Excerpt
> 4
Writing and Building a ROS 2 Node
 To write your own custom code with ROS 2, you will have to create ROS 2 programs, or in other words, nodes. You already discovered the concept...

---
## Writing and Building a ROS 2 Node

To write your own custom code with ROS 2, you will have to create ROS 2 programs, or in other words, nodes. You already discovered the concept of nodes in [_Chapter 3_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_03.xhtml#_idTextAnchor095). In this chapter, we will go deeper, and you will write your first node with Python and C++.

Before you create a node, there is a bit of setup to do: you need to create a ROS 2 workspace, in which you will build your application. In this workspace, you will then add packages to better organize your nodes. Then, in those packages, you can start to write your nodes. After you write a node, you will build it and run it.

We will do this complete process together, with hands-on code and command lines all along the way. This is the process that you will repeat for any new node you create when developing a ROS 2 application.

By the end of this chapter, you will be able to create your own packages and ROS 2 nodes with Python and C++. You will also be able to run and introspect your nodes from the terminal. This is the stepping stone you need in order to learn any other ROS 2 functionality. There is no topic, service, action, parameter, or launch file without nodes.

All explanations will start with Python, followed by C++, which we’ll cover more quickly. If you only want to learn with Python, you can skip the C++ sections. However, if you want to learn with C++, reading the previous Python explanations is mandatory for comprehension.

All the code examples for this chapter can be found in the **ch4** folder of the book’s GitHub repository ([https://github.com/PacktPublishing/ROS-2-from-Scratch](https://github.com/PacktPublishing/ROS-2-from-Scratch)).

In this chapter, we will cover the following topics:

-   Creating and setting up a ROS 2 workspace
-   Creating a package
-   Creating a Python node
-   Creating a C++ node
-   Node template for Python and C++ nodes
-   Introspecting your nodes

## Technical requirements

To follow this chapter, you need the following:

-   Ubuntu 24.04 installed (dual boot or virtual machine)
-   ROS Jazzy
-   A text editor or IDE (for example, VS Code with the ROS extension)

These requirements will be valid for all chapters in _Part 2_.

## Creating and setting up a ROS 2 workspace

Before we write any code, we need to do a bit of organization. Nodes will exist within packages, and all your packages will exist within a **ROS** **2 workspace**.

What is a ROS 2 workspace? A **workspace** is nothing more than a folder organization in which you will create and build your packages. Your entire ROS 2 application will live within this workspace.

To create one, you have to follow certain rules. Let’s create your first workspace step by step and correctly set it up.

## Creating a workspace

To create a workspace, you will simply create a new directory inside your home directory.

As for the workspace’s name, let’s keep it simple for now and use something that is recognizable: **ros2\_ws**.

Note

The name of the workspace is not important, and it will not affect anything in your application. As we are just getting started, we only have one workspace. When you make progress and start to work on several applications, the best practice is to name each workspace with the name of the application or robot. For example, if you create a workspace for a robot named **ABC V3**, then you can name it **abc\_v3\_ws**.

Open a terminal, navigate to your home directory, and create the workspace:

```
$ cd
$ mkdir ros2_ws
```

Then, enter the workspace and create a new directory named **src**. This is where you will write all the code for your ROS 2 application:

```
$ cd ros2_ws/
$ mkdir src
```

That’s really all there is to it. To set up a new workspace, you just create a new directory (somewhere in your home directory) and create an **src** directory inside it.

## Building the workspace

Even if the workspace is empty (we have not created any packages yet), we can still build it. To do that, follow these steps:

1.  Navigate to the workspace root directory. Make sure you are in the right place.
2.  Run the **colcon build** command. **colcon** is the build system in ROS 2, and it was installed when you installed the **ros-dev-tools** packages in [_Chapter 2_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_02.xhtml#_idTextAnchor051).

Let’s build the workspace:

```
$ cd ~/ros2_ws/
$ colcon build
Summary: 0 packages finished [0.73s]
```

As you can see, no packages were built, but let’s list all directories under **~/ros2\_ws**:

```
$ ls
build        install        log        src
```

As you can see, we have three new directories: **build**, **install**, and **log**. The **build** directory will contain the intermediate files required for the overall build. In **log**, you will find logs for each build. The most important directory for you is **install**, which is where all your nodes will be installed after you build the workspace.

Note

You should always run **colcon build** from the root of your workspace directory, not from anywhere else. If you make a mistake and run this command from another directory (let’s say, from the **src** directory of the workspace, or inside a package), simply remove the new **install**, **build**, and **log** directories that were created in the wrong place. Then go back to the workspace root directory and build again.

## Sourcing the workspace

If you navigate inside the newly created **install** directory, you can see a **setup.bash** file:

```
$ cd install/
$ ls
COLCON_IGNORE                         _local_setup_util_ps1.py         setup.ps1 
local_setup.bash                _local_setup_util_sh.py                setup.sh
local_setup.ps1                 local_setup.zsh                                                setup.zsh
local_setup.sh                        setup.bash
```

This might look familiar. If you remember, after we installed ROS 2, we sourced a similar bash script from the ROS 2 installation directory (**/opt/ros/jazzy/setup.bash**) so that we could use ROS 2 in our environment. We will need to do the same for our workspace.

Every time you build your workspace, you have to source it so that the environment (the session you are in) knows about the new changes in the workspace.

To source the workspace, source this **setup.bash** script:

```
$ source ~/ros2_ws/install/setup.bash
```

Then, as we previously did, we are going to add that line into our **.bashrc**. This way, you don’t need to source the workspace every time you open a new terminal.

Open your **.bashrc** (located in your home directory the path is **~/.bashrc**) using any text editor you want:

```
$ gedit ~/.bashrc
```

Add the line to source the workspace’s **setup.bash** script, just after the one to source the global ROS 2 installation. The order is very important here. You have to source the global ROS 2 installation first, and then your workspace, not the other way around:

```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Make sure to save**.bashrc**. Now, both ROS 2 and your workspace will be sourced in any new terminal you open.

Note

If you build the workspace in an already sourced environment, you will still need to source the workspace once again as there have been some changes, and the environment is not aware of that. In this case, you can either source the workspace’s **setup.bash** script directly, source the **.bashrc**, or open a new terminal.

Your workspace is now correctly set up, and you can build your application. Next step: creating a package.

## Creating a package

Any node you create will exist within a package. Hence, to create a node, you first have to create a package (inside your workspace). You will now learn how to create your own packages, and we will see the differences between Python and C++ packages.

But first, what exactly is a package?

## What is a ROS 2 package?

A ROS 2 ==package is a sub-part of your application.==

Let’s consider a robotic arm that we want to use to pick up and place objects. Before we create any node, we can try to split this application into several sub-parts, or packages.

We could have one package to handle a camera, another package for the hardware control (motors), and yet another package to compute motion planning for the robot.

![[attachments/B22403_04_1.jpg]]

Figure 4.1 – Example of a package organization for a robot

Each package is an independent unit, responsible for one sub-part of your application.

Packages are very useful for organizing your nodes, and also to correctly handle dependencies, as we will see later in this book.

Now, let’s create a package, and here you have to make a choice. If you want to create a node with Python, you will create a Python package, and if you want to create a node with C++, you will create a C++ package. The architecture for each package type is quite different.

## Creating a Python package

You will create all your packages in the **src** directory of your ROS 2 workspace. So, make sure to navigate to this directory before you do anything else:

```
$ cd ~/ros2_ws/src/
```

Here is how to construct the command to create a package:

1.  **ros2 pkg create <pkg\_name>**: This is the minimum you need to write.
2.  You can specify a build type with **\--build\_type <build\_type>**. For a Python package, we need to use **ament\_python**.
3.  You can also specify some optional dependencies with **\--dependencies <list\_of\_dependencies\_separated\_with\_spaces>**. It’s always possible to add dependencies later in the package.

Let’s create our first package named **my\_py\_pkg**. We will use this name as an example to work with the main ROS 2 concepts. Then, as we progress, we will use more meaningful names. In the **src** directory of your workspace, run the following:

```
$ ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```

With this command, we say that we want to create a package named **my\_py\_pkg**, with the **ament\_python** build type, and we specify one dependency: **rclpy**—this is the Python library for ROS 2 that you will use in every Python node.

This will print quite a few logs, showing you what files have been created. You might also get a **\[WARNING\]** log about a missing license, but as we have no intention of publishing this package anywhere, we don’t need a license file now. You can ignore this warning.

You can then see that there is a new directory named **my\_py\_pkg**. Here is the architecture of your newly created Python package:

```
/home/<user>/ros2_ws/src/my_py_pkg
├── my_py_pkg
│            └── __init__.py
├── package.xml
├── resource
│            └── my_py_pkg
├── setup.cfg
├── setup.py
└── test
                ├── test_copyright.py
                ├── test_flake8.py
                └── test_pep257.py
```

Not all the files are important right now. We’ll see how to use those files to configure and install our nodes just a bit later.

Here is a quick overview of the most important files and directories:

-   **my\_py\_pkg**: As you can see, inside the package, there is another directory with the same name. This directory already contains an **\_\_init\_\_.py** file. This is where we will create our Python nodes.
-   **package.xml**: Every ROS 2 package (Python or C++) must contain this file. We will use it to provide more information about the package as well as dependencies.
-   **setup.py**: This is where you will write the instructions to build and install your Python nodes.

## Creating a C++ package

We will work a lot with Python in this book, but for completeness, I will also include C++ code for all examples. They will either be explained in the book, or the code will be in the GitHub repository.

Creating a C++ package is very similar to creating a Python package; however, the architecture of the package will be quite different.

Make sure you navigate to the **src** directory of your workspace, and then create a new package. Let’s use a similar pattern as we did for Python and name the package **my\_cpp\_pkg**:

```
$ cd ~/ros2_ws/src/
$ ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```

We choose **ament\_cmake** for the build type (meaning this will be a C++ package), and we specify one dependency: **rclcpp**—this is the C++ library for ROS 2, which we will use in every C++ node.

Once again, you should see quite a few logs, with the newly created files, and maybe a warning about the license that you can ignore.

The architecture of your new C++ package will look like this:

```
/home/ed/ros2_ws/src/my_cpp_pkg/
├── CMakeLists.txt
├── include
│            └── my_cpp_pkg
├── package.xml
└── src
```

Here is a quick explanation of the role of each file or directory:

-   **CMakeLists.txt**: This will be used to provide instructions on how to compile your C++ nodes, create libraries, and so on.
-   **include** directory: In a C++ project, you may split your code into implementation files (**.cpp** extension) and header files (**.hpp** extension). If you split your C++ nodes into **.cpp** and **.hpp** files, you will put the header files inside the **include** directory.
-   **package.xml**: This file is required for any kind of ROS 2 package. It contains more information about the package, and dependencies on other packages.
-   **src** directory: This is where you will write your C++ nodes (**.****cpp** files).

## Building a package

Now that you’ve created one or more packages, you can build them, even if you don’t have any nodes in the packages yet.

To build the packages, go back to the root of your ROS 2 workspace and run **colcon build**. Once again, and as seen previously in this chapter, where you run this command is very important.

```
$ cd ~/ros2_ws/
$ colcon build
Starting >>> my_cpp_pkg
Starting >>> my_py_pkg
Finished <<< my_py_pkg [1.60s]
Finished <<< my_cpp_pkg [3.46s]
Summary: 2 packages finished [3.72s]
```

Both packages have been built. You will have to do that every time you add or modify a node inside a package.

The important thing to notice is this line: **Finished <<< <package\_name> \[time\]**. This means that the package was correctly built. Even if you see additional warning logs, if you also see the **Finished** line, you know the package has been built.

Note

After you build any package, you also have to source your workspace so that the environment is aware of the new changes. You can do any of the following:

\- Open a new terminal as everything is configured in the **.****bashrc** file

\- Source the **setup.bash** script directly (**source ~/ros2\_ws/install/setup.bash**)

\- Source the **.bashrc** manually (**source ~/.bashrc**)

To build only a specific package, you can use the **\--packages-select** option, followed by the name of the package. Here’s an example:

```
$ colcon build --packages-select my_py_pkg
Starting >>> my_py_pkg
Finished <<< my_py_pkg [1.01s]
Summary: 1 package finished [1.26s]
```

This way, you don’t need to build your entire application every time and can just focus on one package.

Now that we have created some packages and we know how to build them, we can create nodes in the packages. But how are we going to organize them?

## How are nodes organized in a package?

To develop a ROS 2 application, you will write code inside nodes. As seen in [_Chapter 3_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_03.xhtml#_idTextAnchor095), _node_ is simply the name of a ROS 2 program.

A node is a subprogram of your application, responsible for one thing. If you have two different functionalities to implement, then you will have two nodes. Nodes communicate with each other using ROS 2 communications (topics, services, and actions).

You will organize your nodes inside packages. For one package (sub-part of your application), you can have several nodes (functionalities). To fully understand how to organize packages and nodes, you need practice and experience. For now, let’s just get an idea with an example.

Let’s come back to the package architecture we had in _Figure 4__.1_, and add nodes inside the packages:

![[attachments/B22403_04_2.jpg]]

Figure 4.2 – Example of a package organization with nodes

As you can see, in the camera package, we could have one node responsible for handling the camera hardware. This node would send images to an image processing node, and this latter would extract the coordinates of objects for the robot to pick up.

In the meantime, a motion planning node (in the motion planning package) would compute the movements that the robot should perform, given a specific command. A path correction node can support this motion planning using the data received from the image processing node.

Finally, to make the robot move, a hardware driver node would be responsible for hardware communication (motors, encoders) and receive commands from the motion planning node. An additional state publisher node could be here to publish additional data about the robot for other nodes to use.

This node organization is purely fictitious and is here just to give you a general idea of how a ROS 2 application can be designed, and which roles a node can have in this application.

Now, you are (finally) going to write your first ROS 2 node. ROS 2 requires quite a lot of installation and configuration before you can actually write some code, but good news, we have completed all of this and can now focus on the code.

We won’t do anything too complicated for now; we won’t dive into complex features or communications. We will write a basic node that you can use as a template to start any future node. We will also build the node and see how to run it.

## Creating a Python node

Let’s create our first Python node, or in other words, our first ROS 2 Python program.

The processes of creating Python and C++ nodes are very different. That’s why I have written a separate section for each of them. We will start with Python, with complete step-by-step explanations. Then we will see how to do the same with C++. If you want to follow the C++ node section, make sure to read this one first.

To create a node, you will have to do the following:

1.  Create a file for the node.
2.  Write the node. We will use **Object Oriented Programming** (**OOP**), as officially recommended for ROS 2 (and almost every existing ROS 2 code you find uses OOP).
3.  Build the package in which the node exists.
4.  Run the node to test it.

Let’s get started with our first Python node.

## Creating a file for the node

To write a node, we first need to create a file. Where should we create this file?

If you remember, when we created the **my\_py\_pkg** package, another **my\_py\_pkg** directory was created inside the package. This is where we will write the node. For every Python package, you have to go to the directory which has the same name as the package. If your package name is **abc**, then you’ll go to **~/ros2\_ws/src/abc/abc/**.

Create a new file in this directory and make it executable:

```
$ cd ~/ros2_ws/src/my_py_pkg/my_py_pkg/
$ touch my_first_node.py
$ chmod +x my_first_node.py
```

After this, open this file to write in it. You can use any text editor or IDE you want here, as long as you don’t get lost in all the files.

If you have no idea what to use, I suggest using VS Code with the ROS extension (as explained in [_Chapter 2_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_02.xhtml#_idTextAnchor051)). This is the tool I’m using for all ROS development.

Note

If you are using VS Code, the best way to open it is to first navigate to the **src** directory of your workspace in a terminal, and then open it. This way, you have access to all the packages in your workspace, and it will make things easier with recognized dependencies and auto-completion:

**$** **cd ~/ros2\_ws/src/**

**$** **code .**

## Writing a minimal ROS 2 Python node

Here is the starting code for any Python node you will create. You can write this code into the **my\_first\_node.py** file:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class MyCustomNode(Node):
                def __init__(self):
                                super().__init__('my_node_name')
def main(args=None):
                rclpy.init(args=args)
                node = MyCustomNode()
                rclpy.spin(node)
                rclpy.shutdown()
if __name__ == '__main__':
                main()
```

As you can see, we use OOP here. OOP is everywhere in ROS 2, and this is the default (and recommended) way to write a node.

Let’s come back to this code step by step, to understand what it’s doing:

```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
```

We first import **rclpy**, the Python library for ROS 2. Inside this library, we can get the **Node** class.

We then create a new class that inherits from the **rclpy** **Node** class:

```
class MyCustomNode(Node):
                def __init__(self):
                                super().__init__('my_node_name')
```

In this class, make sure you call the parent constructor with **super()**. This is also where you will specify the node name.

This node is not doing anything for now; we will add a few functionalities in a minute. Let's finish the code:

```
def main(args=None):
                rclpy.init(args=args)
                node = MyCustomNode()
                rclpy.spin(node)
                rclpy.shutdown()
```

After the class, we create a **main()** function in which we perform the following actions:

1.  Initialize ROS 2 communications with **rclpy.init()**. This should be the first line in your **main()**function.
2.  Create an object from the **MyCustomNode** class we wrote before. This will initialize the node. There’s no need to destroy the node later, as this will happen automatically when the program exits.
3.  Make the node spin. If you omit this line, the node will be created, then the program will exit, and the node will be destroyed. Making the node spin means that we block the execution here, the program stays alive, and thus the node stays alive. In the meantime, as we will see shortly, all registered callbacks for the node can be processed. When you press _Ctrl_ + _C_, the node will stop spinning and this function will return.
4.  After the node is killed, shut down ROS 2 communications with **rclpy.shutdown()**. This will be the last line of your **main()**function.

This is how all your ROS 2 programs will work. As you can see, the node is in fact an object that we create within the program (the node is not the program in itself, but still, it is quite common to refer to the word “node” when we talk about the program). After being created, the node can stay alive and play its part while it is spinning. We will come back to this _spinning_ shortly.

Finally, we also have added these two lines:

```
if __name__ == '__main__':
                main()
```

This is a pure Python thing and has nothing to do with ROS 2. It just means that if you run the Python script directly, the **main()** function will be called, so you can try your program without having to install it with **colcon**.

Great, you have written your first minimal Python node. Before you build and run it, add one more line in the Node’s constructor, so it can do something:

```
class MyCustomNode(Node):
                def __init__(self):
                                super().__init__('my_node_name')
                                self.get_logger().info("Hello World")
```

This line will print **Hello World** when the node starts.

As the **MyCustomNode** class inherits from the **Node** class, we get access to all the ROS 2 functionalities for nodes. This will make things quite convenient for us. Here, you have an example with the logging functionality: we get the **get\_logger()** method from **Node**. Then, with the **info()**method, we can print a log with the info level.

## Building the node

You are now going to build the node so that you can run it.

You might think: why do we need to build a Python node? Python is an interpreted language; couldn’t we just run the file itself?

Yes, this is true: you could test the code just by running it in the terminal **($ python3 my\_first\_node.py**). However, what we want to do is actually install the file in our workspace, so we can start the node with **ros2 run**, and later on, from a launch file.

We usually use the word “build”, because to install a Python node, we have to run **colcon build**.

To build (install) the node, we need to do one more thing in the package. Open the **setup.py** file from the **my\_py\_pkg** package. Locate **entry\_points** and **'console\_scripts'** at the end of the file. For each node we want to build, we have to add one line inside the **'****console\_scripts'** array:

```
entry_points={
                'console_scripts': [
                                "test_node = my_py_pkg.my_first_node:main"
                ],
},
```

Here is the syntax:

```
<executable_name> = <package_name>.<file_name>:<function_name>.
```

There are a few important things to correctly write this line:

-   First, choose an executable name. This will be the name you use with **ros2 run <****pkg\_name> <executable\_name>**.
-   For the filename, skip the **.****py** extension.
-   The function name is **main**, as we have created a **main()** function in the code.
-   If you want to add another executable for another node, don’t forget to add a comma between each executable and place one executable per line.

Note

When learning ROS 2, there is a common confusion between the node name, filename, and executable name:

\- Node name: defined inside the code, in the constructor. This is what you’ll see with the **ros2** **node list**, or in **rqt\_graph**.

\- Filename: the file where you write the code.

\- Executable name: defined in **setup.py** and used with **ros2 run**.

In this first example, I made sure to use a different name for each so you can be aware that these are three different things. But sometimes all three names could be the same. For example, you could create a **temperature\_sensor.py** file, then name your node and your executable **temperature\_sensor**.

Now that you have given the instructions to create a new executable, go to your workspace root directory and build the package:

```
$ cd ~/ros2_ws/
$ colcon build
```

You can also add **\--packages-select my\_py\_pkg** to only build this package.

The executable should now be created and installed in the workspace (it will be placed inside the **install** directory). We can say that your Python node has been built, or installed.

## Running the node

Now you can run your first node, but just before that, make sure that the workspace is sourced in your environment:

```
$ source ~/.bashrc
```

This file already contains the line to source the workspace; you could also just open a new terminal, or source the **setup.bash** script from the workspace.

You can now run your node using **ros2 run** (if you have any doubts, go back to the experiments we did in [_Chapter 3_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_03.xhtml#_idTextAnchor095)):

```
$ ros2 run my_py_pkg test_node
[INFO] [1710922181.325254037] [my_node_name]: Hello World
```

Great, we see the log **Hello World**. Your first node is successfully running. Note that we wrote **test\_node** in the **ros2 run** command, as it’s the executable name we chose in the **setup.py** file.

Now, you might notice that the program is hanging there. The node is still alive because it is spinning. To stop the node, press _Ctrl_ + _C_.

## Improving the node – timer and callback

At this point, you might feel that writing, building, and running a node is a long and complicated process. It’s actually not that complex, and it gets easier with each new node that you create. On top of that, modifying an existing node is even easier. Let’s see that now.

The node we ran is very basic. Let’s add one more functionality and do something more interesting.

Our node is printing one piece of text when it’s started. We now want to make the node print a string every second, as long as it’s alive.

This behavior of “doing X action every Y seconds” is very common in robotics. For example, you could have a node that “reads a temperature every 2 seconds”, or that “gives a new motor command every 0.1 seconds”.

How to do that? We will add a **timer** to our node. A timer will trigger a **callback** function at a specified rate.

Let’s go back to the code and modify the **MyCustomNode** class. The rest of the code stays the same:

```
class MyCustomNode(Node):
                def __init__(self):
                                super().__init__('my_node_name')
                                self.counter_ = 0
                                self.timer_ = self.create_timer(1.0, self.print_hello)
                def print_hello(self):
                                self.get_logger().info("Hello " + str(self.counter_))
                                self.counter_ += 1
```

We still have the constructor with **super()**, but now the log is in a separate method. Also, instead of just printing **Hello World**, here we create a **counter\_** attribute that we increment every time we use the log.

Note

If you’re wondering why there is a trailing underscore **\_** at the end of each class attribute, this is a common OOP convention that I follow to specify that a variable is a class attribute. It’s simply a visual help and has no other function. You can follow the same convention or use another one—just make sure to stay consistent within one project.

The most important line is the one to create the timer. To create the timer we use the **create\_timer()** method from the **Node** class. We need to give two arguments: the rate at which we want to call the function (float number), and the callback function. Note that the callback function should be specified without any parenthesis.

This instruction means that we want to call the **print\_hello** method every **1.0** second.

Let’s now try the code. As we have already specified how to create an executable from this file in the **setup.py** file, we don’t need to do it again.

All we have to do is to build, source, and run. Remember: “build, source, run.” Every time you create a new node, or modify an existing one, you have to “build, source, run.”

In a terminal, go to the root directory of your ROS 2 workspace and build the package:

```
$ cd ~/ros2_ws/
$ colcon build --packages-select my_py_pkg
```

Note

On top of **\--packages-select <pkg\_name>**, you can add the **\--symlink-install** option, so you won’t have to build the package every time you modify your Python nodes; for example, **$ colcon build --packages-select** **my\_py\_pkg --symlink-install**.

You might see some warning logs, but as long as you see the line starting with **Finished <<< my\_py\_pkg**, it worked correctly. This will install the executable, but then if you modify the code, you should be able to run it without building it again.

Two important things: this only works for Python packages, and you still have to build the package for any new executable you create.

Then, from this terminal or another one, source and run the following:

```
$ source ~/.bashrc
$ ros2 run my_py_pkg test_node
[INFO] [1710999909.533443384] [my_node_name]: Hello 0
[INFO] [1710999910.533169531] [my_node_name]: Hello 1
[INFO] [1710999911.532731467] [my_node_name]: Hello 2
[INFO] [1710999912.534052411] [my_node_name]: Hello 3
```

As you can see, the process of build, source, and run is quite fast and not that complicated. Here, we can see that the node prints a log every second, and the counter increments in each new log.

Now, how is this possible? How is the **print\_hello()** method called? We have created a timer, yes, but nowhere in the code have we actually called **print\_hello()** directly.

It works because the node is spinning, thanks to **rclpy.spin(node)**. This means that the node is kept alive, and all registered callbacks can be called during this time. What we do with **create\_timer()** is simply to register a callback, which can then be called when the node is spinning.

This was your first example of a callback, and as you will see in the following chapters of the book, everything runs with callbacks in ROS 2. At this point, if you still have some trouble with the syntax, the callbacks, and the spinning, don’t worry too much. As you make progress with the book, you will repeat this process many times. When learning ROS 2, understanding comes with hands-on experience.

We are now done with this Python node. With what you’ve seen here, you should be able to create your own new Python nodes (in the same package or another package). Let’s now switch to C++. If you are only interested in learning ROS 2 with Python for now, you can skip the C++ section.

## Creating a C++ node

We are going to do exactly the same thing we did for the Python node: create a file, write the node, build, source, and run.

Make sure you have read the previous Python section as I will not repeat everything here. We will basically just see how to apply the process for a C++ node.

To create a C++ node, we first need a C++ package. We will use the **my\_cpp\_pkg** package that we created previously.

## Writing a C++ node

Let’s create a file for the node. Go to the **src** directory inside the **my\_cpp\_pkg** package and create a **.****cpp** file:

```
$ cd ~/ros2_ws/src/my_cpp_pkg/src/
$ touch my_first_node.cpp
```

You could also create the file directly from your IDE and not use the terminal.

Now, if you haven’t done this previously, open your workspace with VS Code or any other IDE:

```
$ cd ~/ros2_ws/src/
$ code .
```

Open **my\_first\_node.cpp**. Here is the minimal code to write a C++ node:

```
#include "rclcpp/rclcpp.hpp"
class MyCustomNode : public rclcpp::Node
{
public:
                MyCustomNode() : Node("my_node_name")
                {
                }
private:
};
int main(int argc, char **argv)
{
                rclcpp::init(argc, argv);
                auto node = std::make_shared<MyCustomNode>();
                rclcpp::spin(node);
                rclcpp::shutdown();
                return 0;
}
```

Note

If you are using VS Code and you type this code, you might see an include error for the **rclcpp** library. Make sure to save the file and wait a few seconds. If the include is still not recognized, go to the **Extensions** tab and disable and re-enable the ROS extension.

As you can see (and this was similar with Python), in ROS 2 we heavily use OOP with C++ nodes.

Let’s analyze this code step by step:

```
#include "rclcpp/rclcpp.hpp"
```

We first include **rclcpp**, the C++ library for ROS 2. This library contains the **rclcpp::Node** class:

```
class MyCustomNode : public rclcpp::Node
{
public:
                MyCustomNode() : Node("my_node_name")
                {
                }
private:
};
```

As we did for Python, we have created a class that inherits from the **Node** class. The syntax is different, but the principle is the same. From this **Node** class, we will be able to access all the ROS 2 functionalities: logger, timer, and so on. As you can see, we also specify the node name in the constructor. For now, the node does nothing; we will add more functionalities in a minute:

```
int main(int argc, char **argv)
{
                rclcpp::init(argc, argv);
                auto node = std::make_shared<MyCustomNode>();
                rclcpp::spin(node);
                rclcpp::shutdown();
                return 0;
}
```

You need a **main()** function if you want to be able to run your C++ program. In this function, we do exactly the same thing as for Python, with just some differences in the syntax:

1.  Initialize ROS 2 communications with **rclcpp::init()**.
2.  Create a node object from your newly written class. As you can see, we don’t create an object directly, but a shared pointer to that object. ==In ROS 2 and C++, almost everything you create will be a smart pointer (shared, unique, and so on).==
3.  We then make the node spin with **rclcpp::spin()**.
4.  Finally, when the node is stopped (_Ctrl_ + _C_), we shut down all ROS 2 communications with **rclcpp::shutdown()**.

This structure for the **main()** function will be very similar for all your ROS 2 programs. As you can see, once again, the node is not the program in itself. The node is created inside the program.

Before we go further and build, source, and run our node, let’s improve it now with a timer, a callback, and a log.

Modify the **MyCustomNode** class, and leave the rest as it is:

```c
class MyCustomNode : public rclcpp::Node
{
public:
	MyCustomNode() : Node("my_node_name"), counter_(0)
	{
		timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyCustomNode::print_hello, this));
	}
                void print_hello()
                {
                                RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
                                counter_++;
                }
private:
                int counter_;
                rclcpp::TimerBase::SharedPtr timer_;
};
```

This code example will do the same thing as for the Python node. We create a timer so that we can call a callback function every **1.0** second. In this callback function, we print **Hello** followed by a counter that we increment every time.

There are some specificities related to C++:

-   For the timer, we have to create a class attribute. As you can see we also create a shared pointer here: **rclcpp::TimerBase::SharedPtr**.
-   We use **this->create\_wall\_timer()** to create the timer. **this->** is not required here, but I have added it to emphasize that we are using the **create\_wall\_timer()** method from the **Node** class.
-   To specify the callback in the timer, as we are in a C++ class, we have to use **std::bind(&ClassName::method\_name, this)**. Make sure you don’t use any parenthesis for the method name.

The node is now finished, so we can build it.

## Building and running the node

We can’t just run the C++ file; we first have to compile it and create an executable. To do this, we will edit the **CMakeLists.txt** file. Open this file, and after a few lines, you will find something like this:

```
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
```

The line to find **rclcpp** is here because we provided **\--dependencies rclcpp** when we created the package with **ros2 pkg create**. Later on, if your nodes in this package require more dependencies, you can add the dependencies here, one per line.

Just after this line, add an extra new line, and then the following instructions:

```
add_executable(test_node src/my_first_node.cpp)
ament_target_dependencies(test_node rclcpp)
install(TARGETS
        test_node
        DESTINATION lib/${PROJECT_NAME}/
)
```

To build a C++ node, we need to do three things:

1.  Add a new executable with the **add\_executable()**function. Here, you have to choose a name for the executable (the one that will be used with **ros2 run <pkg\_name> <executable\_name>**), and we also have to specify the relative path to the C++ file.
2.  Link all dependencies for this executable with the **ament\_target\_dependencies()**function.
3.  Install the executable with the **install()**instruction, so that we can find it when we use **ros2 run**. Here, we put the executable in a **lib/<package\_name>** directory.

Then, for each new executable you create, you need to repeat _steps 1_ and _2_ and add the executable inside the **install()** instruction, one per line without any commas. There’s no need to create a new **install()** instruction for each executable.

Note

The end of your **CMakeLists.txt** will contain ==a block starting with **if(BUILD\_TESTING)**, and then **ament\_package()**. As we are not doing any build testing here, you can remove the entire **if** block. Just make sure to keep the **ament\_package()** line, ==which should be the last line of the file.

You can now build the package with **colcon build**, which is going to create and install the executable:

```
$ cd ~/ros2_ws/
$ colcon build --packages-select my_cpp_pkg
```

If you get any error during the build process, make sure to fix your code first, and then build again. Then, you can source your environment, and run your executable:

```
$ source ~/.bashrc
$ ros2 run my_cpp_pkg test_node
[INFO] [1711006463.017149024] [my_node_name]: Hello 0
[INFO] [1711006464.018055674] [my_node_name]: Hello 1
[INFO] [1711006465.015927319] [my_node_name]: Hello 2
[INFO] [1711006466.015355747] [my_node_name]: Hello 3
```

As you can see, we run the **test\_node** executable (built from **my\_first\_node.cpp** file), which is going to start the **my\_node\_name** node.

You have now successfully written a C++ node. For each new node that you create, you will have to create a new C++ file, write the node class, set the build instructions for a new executable in **CMakeLists.txt**, and build the package. Then, to start the node, source the environment and run the executable with **ros2 run**.

## Node template for Python and C++ nodes

All the nodes we start in this book will follow the same structure. As additional help to get started quickly, I have created a node template you can use to write the base of any Python or C++ node. I use these templates myself when creating new nodes, as the code can be quite repetitive.

You can copy and paste the templates either from this book directly, or download them from the GitHub repository: [https://github.com/PacktPublishing/ROS-2-from-Scratch](https://github.com/PacktPublishing/ROS-2-from-Scratch).

## Template for a Python node

Use this code to start any new Python node:

```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class MyCustomNode(Node): # MODIFY NAME
                def __init__(self):
                                super().__init__("node_name") # MODIFY NAME
def main(args=None):
```

```
                rclpy.init(args=args)
                node = MyCustomNode() # MODIFY NAME
                rclpy.spin(node)
                rclpy.shutdown()
if __name__ == "__main__":
                main()
```

All you have to do is remove the **MODIFY NAME** comments and change the class name (**MyCustomNode**) and the node name (**"node\_name"**). It’s better to use names that make sense. For example, if you are writing a node to read data from a temperature sensor, you could name the class **TemperatureSensorNode**, and the node could be **temperature\_sensor**.

## Template for a C++ node

Use this code to start any new C++ node:

```
#include "rclcpp/rclcpp.hpp"
class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
                MyCustomNode() : Node("node_name") // MODIFY NAME
                {
                }
private:
};
int main(int argc, char **argv)
{
                rclcpp::init(argc, argv);
                auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
                rclcpp::spin(node);
                rclcpp::shutdown();
                return 0;
}
```

Remove the **MODIFY NAME** comments and rename the class and the node.

Those two templates will allow you to start your nodes more quickly. I recommend you to use them as much as you can.

## Introspecting your nodes

To finish this chapter, we will practice a bit more with the **ros2 node** command line.

So far, you have seen how to write a node, build it, and run it. One missing part is to know how to introspect your nodes. Even if a node can run, it doesn’t mean it will do exactly what you want it to do.

Being able to introspect your nodes will help you fix errors that you might have made in your code. It will also allow you to easily find more information about other nodes that you are starting but didn’t write (as we did in the discovery phase in [_Chapter 3_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_03.xhtml#_idTextAnchor095)).

For each core concept in _Part 2_, we will take a bit of time to experiment with the command-line tools related to the concept. The command-line tool for nodes is **ros2 node**.

First, and before we use **ros2 node**, we have to start a node. As a recap, to start a node, we use **ros2 run <package\_name> <executable\_name>**. If we start the Python node we have created in this chapter, we use this:

```
$ ros2 run my_py_pkg test_node
```

Only after we have started a node can we do some introspection with **ros2 node**.

## ros2 node command line

To list all running nodes, use **ros2** **node list**:

```
$ ros2 node list
/my_node_name
```

We find the name of the node, which we defined in the code.

Once we have the node name, we can get more info about it with **ros2 node** **info <node\_name>**:

```
$ ros2 node info /my_node_name
/my_node_name
        Subscribers:
        Publishers:
                /parameter_events: rcl_interfaces/msg/ParameterEvent
                /rosout: rcl_interfaces/msg/Log
        Service Servers:
                /my_node_name/describe_parameters: rcl_interfaces/srv/DescribeParameters
                /my_node_name/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
                /my_node_name/get_parameters: rcl_interfaces/srv/GetParameters
                /my_node_name/get_type_description: type_description_interfaces/srv/GetTypeDescription
                /my_node_name/list_parameters: rcl_interfaces/srv/ListParameters
```

```
                /my_node_name/set_parameters: rcl_interfaces/srv/SetParameters
                /my_node_name/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
        Service Clients:
        Action Servers:
        Action Clients:
```

As you can see, there are quite a lot of things on the terminal. We will get to know all of them in the following chapters. With **ros2 node info <node\_name>** you can see all topics (publishers/subscribers), services, and actions running for this node.

## Changing the node name at run time

As we progress throughout the book, I will give you additional tips for working with ROS 2 and the command line. Here is one: when starting an executable, you can choose to use the default node name (the one defined in the code) or replace it with a new name.

To add any additional argument to **ros2 run**, first add **\--ros-args** (only once).

Then, to rename the node, add **\-r \_\_node:=<new\_name>**. **\-r** means remap; you could also use **\--remap**. For example, if we want to name the node **abc**, we could use this:

```
$ ros2 run my_py_pkg test_node --ros-args -r __node:=abc
[INFO] [1711010078.801996629] [abc]: Hello 0
[INFO] [1711010079.805748394] [abc]: Hello 1
```

As you can see from the logs, instead of **my\_node\_name**, we see **abc**.

List all running nodes:

```
$ ros2 node list
/abc
```

This functionality can be very helpful and gives you more control over how to start a node, without having to modify the code directly.

Note

When running multiple nodes, you should make sure that each node has a unique name. Having two nodes with the same name can lead to some unexpected issues that can take a long time to debug. In the future, you will see that you may want to run the same node several times, for example, three **temperature\_sensor** nodes, one each for a different sensor. You could rename them so that you have **temperature\_sensor\_1**, **temperature\_sensor\_2**, and **temperature\_sensor\_3**.

## Summary

In this chapter, you have created your first node. Let’s do a quick recap of all the steps.

Before creating any node, you need to follow these steps:

1.  You first need to create and set up a ROS 2 workspace.
2.  In this workspace, you can create several packages (Python or C++) that represent different sub-parts of your application.

Then, in one package you can create one or several nodes. For each node, you will have to do the following:

1.  Create a file inside the package.
2.  Write the node (using the OOP template as a base).
3.  Set the build instructions (**setup.py** for Python, **CMakeLists.txt** for C++).
4.  Build the package.

To run the node, don’t forget to source the workspace first, and then start the node with **ros2 run <****pkg\_name> <executable\_name>**.

Finally, you can introspect your nodes and even change their names when you start them, using the **ros2 node** command line.

Feel free to come back to this chapter anytime to see the complete process of creating a node for both Python and C++. All the code is available on GitHub at [https://github.com/PacktPublishing/ROS-2-from-Scratch](https://github.com/PacktPublishing/ROS-2-from-Scratch). There you can find the OOP template code for Python and C++, **my\_py\_pkg** package, and **my\_cpp\_pkg** package.

In this chapter, you have also seen how to create a timer and a callback function. You have a better idea of how the spin mechanism works, and how it allows the node to stay alive and run the callbacks. This will be very useful for the following chapters.

In the next chapter, we will see how nodes communicate with each other using topics. You will write your own topics (publishers/subscribers) inside nodes and experiment with them.

<table id="table001-12"><colgroup></colgroup><colgroup><col></colgroup><colgroup><col></colgroup><tbody><tr><td><h4>Unlock this book’s exclusive benefits now</h4><p>This book comes with additional benefits designed to elevate your learning experience.</p></td><td rowspan="2"><p><img alt="" width="246" height="238" src="attachments/9781835881408.png"></p><p lang="en-US" xml:lang="en-US"><a href="https://www.packtpub.com/unlock/9781835881408" target="_blank" rel="noopener noreferrer">https://www.packtpub.com/unlock/9781835881408</a></p></td></tr><tr><td><p><em>Note: Have your purchase invoice ready before </em><span><em>you begin.</em></span></p></td></tr></tbody></table>
