---
created: 2025-08-14T13:54:13 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_09.xhtml#_idParaDest-217
author: 
---

# Chapter 9: Launch Files – Starting All Your Nodes at Once | ROS 2 from Scratch

> ## Excerpt
> 9
Launch Files – Starting All Your Nodes at Once
 At this point, you know how to write nodes, how to make them communicate with topics, services, and actions, and how to make them...

---
## Launch Files – Starting All Your Nodes at Once

At this point, you know how to write nodes, how to make them communicate with topics, services, and actions, and how to make them more dynamic with parameters.

In this last chapter of _Part 2_, we will bring everything together and go one step further toward making your application more scalable. Here, we will talk about launch files, which allow you to start all your nodes and parameters at once.

To start with launch files, it’s important that you’re comfortable with the concepts seen in the previous chapters. As a starting point, we will use the code inside the **ch8** folder from the book’s GitHub repository ([https://github.com/PacktPublishing/ROS-2-from-Scratch](https://github.com/PacktPublishing/ROS-2-from-Scratch)). You can find the final code for launch files in the **ch9** folder.

First, as always, I will use a real-life example to explain why you need launch files and what they are exactly. You will then dive into the code and create your own launch file with XML and Python (we will discuss which language is more appropriate). You will also experiment with extra configurations to fully customize your nodes inside launch files, and you will practice more with a final challenge.

By the end of this chapter, you will be able to properly scale your ROS 2 applications and know how to use or modify existing launch files. Almost every ROS 2 application or stack contains one or several launch files. Being comfortable with them is key to becoming a great ROS developer.

In this chapter, we will cover the following topics:

-   What is a ROS 2 launch file?
-   Creating and installing an XML launch file
-   Creating a Python launch file – XML or Python for launch files?
-   Configuring nodes inside a launch file
-   Launch file challenge

## What is a ROS 2 launch file?

After everything you’ve learned already, understanding the concept of launch files will not be very difficult.

You have experimented a bit with launch files in [_Chapter 3_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_03.xhtml#_idTextAnchor095). We will now, as usual, start from scratch and see what a launch file is with an example. First, we’ll look at why we need launch files.

## Why launch files?

As your ROS 2 application starts to grow, so does the number of nodes and parameters. For example, a ROS stack I developed for a robotic arm had more than 15 nodes and 200 parameters. Imagine opening 15 terminals and starting all the nodes one by one with all the correct values for parameters. This would quickly become a nightmare.

For this explanation, let’s assume we have the following nodes in our application:

-   Three camera nodes with different settings
-   Two LED panel nodes with varying numbers of LEDs
-   One battery node
-   Another node with more parameters

Here is what your application would look like:

![[attachments/B22403_09_1.jpg]]

Figure 9.1 – ROS 2 application with seven nodes and sixteen parameters

To start all those nodes, you will need to open seven terminals and start the nodes one by one. For each node, you will also need to provide all the required parameters’ values (with what you’ve seen in the previous chapter, you can use YAML param files to make this easier). Not only is this not scalable, but it will make your development process much slower and frustrating. With so many terminals, it’s easy to make mistakes or to forget which terminal is doing what.

A solution you could think of is to create a script (a **bash** script, for example) to start all **ros2 run** commands from one file. That way, you could run your application from just one terminal. This would reduce development time and allow your application to scale.

Well, this is exactly what launch files are made for. There’s no need to write your own script; all you need to do is create a launch file and follow a few syntax rules. Launch files can be installed within your ROS 2 application. Let’s look at an example in the next section.

## Example of a launch file with seven nodes

If we continue with our example, here is how your nodes would be organized:

![[attachments/B22403_09_2.jpg]]

Figure 9.2 – Launch file with all nodes and parameters

Inside one file, you start all the nodes and provide the values you want for each parameter. This file can be written with XML, YAML, or Python—we will see how to do that in a moment. Then, once the launch file is written, you will install it (**colcon build**) and run it with the **ros2 launch** command-line tool.

It’s not uncommon to have a few dozen nodes and a few hundred parameters inside one application. Without launch files, it would be impossible to quickly start the application, and you would spend most of your time debugging trivial things.

Launch files allow you to customize and scale your application easily. There is not much more to say; the concept is fairly straightforward. Most of the work is about learning how to implement one and knowing the features to customize your nodes to make them more dynamic. This is what we will dive into right now.

## Creating and installing an XML launch file

You will now create your first launch file. We will start with XML. Later in this chapter, we will also write Python launch files and compare the two languages, but let’s keep things simple to get started.

To properly create, install, and start a launch file, you need to do a bit of setup. In this section, we will follow all the necessary setup steps with a minimal launch file.

What we want to do here is to start the number application (**number\_publisher** and **number\_counter** nodes) from one terminal, with just one command line. Let’s get started.

## Setting up a package for launch files

Where should you put your launch files? You could theoretically create a launch file in any existing package.

However, this method can quickly lead to a dependency mess between packages. If package A requires package B, and you create a launch file in package B to start nodes from both packages, then you have created what’s called a _dependency loop_. Package A depends on package B, and package B depends on package A. This is a very bad way to start a ROS application.

As a best practice, we will create a package dedicated to launch files. We will not modify any existing package; instead, we will create a completely independent one.

First, let’s choose a name for this package. We will follow a common naming convention. We start with the name of the robot or application, followed by the **\_bringup** suffix. As we don’t have a robot here, we will call this package **my\_robot\_bringup**. If your robot were named _abc_, you would create an **abc\_bringup** package.

Navigate to the **src** directory in your ROS 2 workspace and create this package. It will not contain any Python or C++ nodes. For the build type, you can choose **ament\_cmake** (you could even omit the build type, as **ament\_cmake** is the default anyway):

```
$ cd ~/ros2_ws/src/
$ ros2 pkg create my_robot_bringup --build-type ament_cmake
```

Alternatively, you could just run **$ ros2 pkg** **create my\_robot\_bringup**.

Once the package is created, we can remove directories that we don’t need:

```
$ cd my_robot_bringup/
$ rm -r include/ src/
```

Then, we create a **launch** directory. This is where we will put all our launch files for this application:

```
$ mkdir launch
```

Before we create a launch file, let’s finish the package configuration. Open the **CMakeLists.txt** file and add these lines:

```
find_package(ament_cmake REQUIRED)
<strong>install(DIRECTORY</strong>
<strong>&nbsp;&nbsp;launch</strong>
<strong>&nbsp;&nbsp;DESTINATION share/${PROJECT_NAME}/</strong>
<strong>)</strong>
ament_package()
```

This will install the **launch** directory when you build your package with **colcon build**.

Now, the package is correctly configured. You only need to do those steps once for each ROS 2 application. Then, to add a launch file, you just have to create a new file inside the **launch** folder. Let’s do that.

## Writing an XML launch file

Navigate to the **launch** folder you created inside the **my\_robot\_bringup** package. To create a launch file, you will first choose a name and then use the **.launch.xml** extension. Since we have named our application the _number app_, let’s create a new file named **number\_app.launch.xml**:

```
$ cd ~/ros2_ws/src/my_robot_bringup/launch/
$ touch number_app.launch.xml
```

Open the file, and let’s start to write the content for the launch file.

First, you will need to open and close a **<launch>** tag. Everything you write will be between those two lines. This is the minimum code for an XML launch file:

```
&lt;launch&gt;
&lt;/launch&gt;
```

Then, we want to start the **number\_publisher** and **number\_counter** nodes.

As a quick reminder, in the terminal, you would run this:

```
$ ros2 run my_py_pkg number_publisher
$ ros2 run my_cpp_pkg number_counter
```

Here, I started one node from the Python package and the other one from the C++ package. The two arguments we need to provide for **ros2 run** are the package name and executable name. This is the same inside a launch file. To add a node, use a **<node>** tag with the **pkg** and **exec** arguments:

```
&lt;launch&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;node pkg="my_py_pkg" exec="number_publisher"/&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;node pkg="my_cpp_pkg" exec="number_counter"/&gt;
&lt;/launch&gt;
```

With this, we start the same two nodes from the launch file. As you can see, there’s nothing very complicated. Later on in this chapter, we will see how to configure the application with remappings, parameters, namespaces, and so on. For now, let’s focus on running this minimal launch file.

## Installing and starting a launch file

You now have to install your new launch file before you can start using it.

As we are starting nodes from the **my\_py\_pkg** and **my\_cpp\_pkg** packages, we need to add the dependencies in the **package.xml** file of the **my\_robot\_bringup** package:

```
&lt;exec_depend&gt;my_py_pkg&lt;/exec_depend&gt;
&lt;exec_depend&gt;my_cpp_pkg&lt;/exec_depend&gt;
```

Note

Previously, we only used a **<depend>** tag when specifying dependencies. In this case, there is nothing to build; we only need the dependency when executing the launch file. Thus, we use a weaker tag, **<exec\_depend>**.

For each new package you use in your launch files, you will add a new **<exec\_depend>** tag in the **package.xml** file.

Now, we can install the launch file. To do so, you just need to build your package:

```
$ cd ~/ros2_ws/
$ colcon build --packages-select my_robot_bringup
```

Then, source your environment, and use the **ros2 launch** command-line tool to start the launch file. The full command is **ros2 launch <****package\_name> <launch\_file\_name>**:

```
$ ros2 launch my_robot_bringup number_app.launch.xml
```

You will see the following logs:

```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [number_publisher-1]: process started with pid [21108]
[INFO] [number_counter-2]: process started with pid [21110]
[number_counter-2] [INFO] [1716293867.204728817] [number_counter]: Number Counter has been started.
[number_publisher-1] [INFO] [1716293867.424510088] [number_publisher]: Number publisher has been started.
[number_counter-2] [INFO] [1716293868.413350769] [number_counter]: Counter: 2
[number_counter-2] [INFO] [1716293869.413321220] [number_counter]: Counter: 4
[number_counter-2] [INFO] [1716293870.413321491] [number_counter]: Counter: 6
```

What’s happening here? Let’s take a closer look:

1.  A log file is created and the logging verbosity is set.
2.  Each executable that you provided in the launch file is started as a new process. You can see the process name (for example, **number\_publisher-1**) and the process ID (denoted as **pid**).
3.  Then, as all nodes are started in the same terminal, you will see all logs from all nodes.

This example is quite simple, as we just start two executables with no additional configuration. Launch files will become quite handy when the number of nodes and settings gets bigger. Also, the **ros2 launch** command-line tool is very easy to use. There is not really much more than what we’ve seen here.

Now that you have completed the process to create, install, and start a launch file, let’s talk about Python launch files.

## Creating a Python launch file – XML or Python for launch files?

There are actually three languages you can use to create launch files in ROS 2: Python, XML, and YAML. I will not cover YAML launch files as they are not seldom used, and YAML doesn’t have any competitive advantage over XML for launch files. Here, we will be focusing on Python and XML.

We will start this section by creating a Python launch file (the same application as before). Then, I will compare XML and Python launch files and give you some guidance on how to get the best out of them both.

## Writing a Python launch file

As we already have a fully configured **my\_robot\_bringup** package for our application, there’s no need to do anything else. All we have to do is create a new file inside the **launch** directory.

For Python launch files, you will use the **.launch.py** extension. Create a new file named **number\_app.launch.py**. Here is the code required to start the **number\_publisher** and **number\_counter** nodes:

```
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
&nbsp;&nbsp;&nbsp;&nbsp;ld = LaunchDescription()
&nbsp;&nbsp;&nbsp;&nbsp;number_publisher = Node(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;package="my_py_pkg",
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;executable="number_publisher"
&nbsp;&nbsp;&nbsp;&nbsp;)
&nbsp;&nbsp;&nbsp;&nbsp;number_counter = Node(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;package="my_cpp_pkg",
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;executable="number_counter"
&nbsp;&nbsp;&nbsp;&nbsp;)
&nbsp;&nbsp;&nbsp;&nbsp;ld.add_action(number_publisher)
&nbsp;&nbsp;&nbsp;&nbsp;ld.add_action(number_counter)
&nbsp;&nbsp;&nbsp;&nbsp;return ld
```

The first thing you will notice is that the code is much, much longer than the XML one. I will come back to this in a minute when I compare Python and XML. For now, let’s focus on the required steps to write a Python launch file:

1.  The launch file must include a **generate\_launch\_description()** function. Make sure you don’t make any typos.
2.  In this function, you will need to create and return a **LaunchDescription** object. You can get this from the **launch** module.
3.  To add a node in the launch file, you create a **Node** object (from **launch\_ros.actions**) and specify the package and executable name. Then, you can add this object to the **LaunchDescription** object.

That’s it for now, but there are more options that we will explore a bit later in this chapter.

Once you have written the launch file, make sure to add all required dependencies in the **package.xml** file of the **my\_robot\_bringup** package. As we already did that with the XML launch file (and we have the same dependencies here), we can skip this step.

Finally, to install this launch file, build the **my\_robot\_bringup** package again. Since we already wrote the necessary instructions in the **CMakeLists.txt** file, the launch file will be installed. All you need to do after that is to source your environment and start the launch file with **ros2 launch**:

```
$ ros2 launch my_robot_bringup number_app.launch.py
```

To create, install, and start a Python launch file, the process is the same as for an XML launch file. Only the code is different. Let’s now compare the two languages regarding their use in launch files.

## XML versus Python for launch files

I have a strong bias toward simplicity, so, from seeing the previous code examples, you can already guess where I’m going to stand.

To answer the XML versus Python question, let’s first go back in time.

### The issue with Python launch files

In ROS 1, the first version of ROS, XML was the only language used for launch files. Python was actually also available, but due to non-existent documentation, nobody knew about it.

At the beginning of ROS 2, the development team put a stronger emphasis on Python launch files and started to write the documentation only for Python, thus making it the default language for launch files. XML (and YAML) launch files were also supported, but again, due to non-existent documentation, nobody was using them.

I was initially enthusiastic about the idea of writing Python launch files, as this meant you could take advantage of the Python logic and syntax to make launch files much more dynamic and easier to write. That’s the theory, but in practice, I realized I didn’t see any programming logic in most of the launch files I found, and it was just another—more complex and difficult—way to write a description, which is basically why XML exists in the first place.

You can already see the added complexity in the two previous examples. To start two nodes, it takes four lines in XML and twenty lines in Python (I could optimize the code and make it less than fifteen lines, but that’s still a lot more). For the same number of nodes, you can expect Python launch files to be two to five times longer than the XML version.

Also, with more functionalities (parameters, arguments from the terminal, conditions, paths, and so on), you will have to use an increasing amount of Python imports that are hard to find and use. You will realize this as we see more examples of XML and Python launch files all along this book.

Fortunately, XML is coming back, as the official documentation is starting to include it as well as Python. More and more developers have started to use XML launch files again, which is a good thing because more online tutorials and open source code will include them.

### How to combine XML and Python launch files in your application

XML launch files are much simpler and smaller to write than Python launch files. However, for some advanced use cases, Python will be the only choice, as it contains some functionalities that are not available for XML. This could be a problem because if you need just one Python functionality, it would mean that you’d need to write the entire launch file in Python.

Fortunately, there is a very easy way to solve that. As we will see in a minute, you can include any kind of launch file into any other launch file, be it Python, XML, or YAML.

So, if you absolutely need to use Python for a specific launch functionality, then go ahead and create a Python launch file for that. You can then include this launch file in your _main_ XML launch file. You can also include any other existing Python launch file (from an already installed package) that contains the functionality you need. By doing this, you keep your code minimal and simple.

Now, what to do when you need to create a Python launch file for a specific use case? The syntax is really complicated, and there are too many imports for any functionality. It can quickly become a challenge.

What I myself do when I have to create a Python launch file is to try to find an existing launch file on GitHub that does what I want and tweak the code so that it works with my application. I gave up on trying to learn or even memorize the Python launch file syntax. I am not usually a fan of the “copy/paste from the internet” method, but I make an exception for Python launch files.

In the end, it’s a matter of choice for you. A correctly written XML, YAML, or Python launch file will do the exact same thing. As for YAML, it’s just another markup language, and I find XML easier to use for launch files. My recommendation is to use XML whenever possible. Use Python only if you have to and only for the functionalities that require Python. Then, include the Python launch file inside your XML one.

Following this process will make your life easier when developing ROS 2 applications.

### Including a launch file inside another launch file

Since I talked about including a Python launch file inside an XML launch file, let’s see how to do that. The syntax won’t be that complicated.

Make sure you add everything inside **<launch></launch>** tags. To include another launch file, use an **<include>** tag. Here is an example:

```
&lt;launch&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;include file="$(find-pkg-share &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;my_robot_bringup)/launch/number_app.launch.py" /&gt;
&lt;/launch&gt;
```

This line, with **find-pkg-share**, will find the path to the **number\_app.launch.py** launch file inside the **my\_robot\_bringup** package. Then, the content of the launch file will be included. Even if you include a Python launch file inside an XML one, this will work.

You can reuse this line in any other XML launch file; just replace the package name and launch filename.

Now, if you wanted to do the opposite (which means including an XML launch file inside a Python launch file), here is what you would need to write:

```
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
def generate_launch_description():
&nbsp;&nbsp;&nbsp;&nbsp;ld = LaunchDescription()
&nbsp;&nbsp;&nbsp;&nbsp;other_launch_file = IncludeLaunchDescription(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;XMLLaunchDescriptionSource(os.path.join(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;get_package_share_directory('my_robot_bringup'),
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'launch/number_app.launch.xml')))
&nbsp;&nbsp;&nbsp;&nbsp;ld.add_action(other_launch_file)
&nbsp;&nbsp;&nbsp;&nbsp;return ld
```

This code example illustrates what I was saying about the extra complexity brought by Python launch files. This complexity is not justified here, as it adds nothing compared to the XML file.

With those two code examples, you can now combine any XML and Python launch files.

Now that you have seen the process of creating a launch file in both XML and Python, let’s go a bit further and add some extra configuration for the nodes.

## Configuring nodes inside a launch file

So far, we have just started two nodes, with zero extra configuration. When you start a node with **ros2 run**, as we have seen in the previous chapters in _Part 2_, you can rename it, rename topics/services/actions, add parameters, and so on.

In this section, you will learn how to do that inside a launch file. We will also introduce the concept of namespaces. All code examples will be in XML and Python.

## Renaming nodes and communications

In an XML launch file, to rename a node, simply add a **name** argument in a **<****node>** tag:

```
&lt;node pkg="your_package" exec="your_exec" name="new_name" /&gt;
```

Changing the name for a topic/service/action is actually named _remapping_. To remap a communication, you have to use a **<remap>** tag, inside the **<****node>** tag:

```
&lt;node pkg="your_package" exec="your_exec"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/topic1" to="/topic2" /&gt;
&lt;/node&gt;
```

You can add as many **<remap>** tags as you want, each one in a new line.

Note

This is a quick XML reminder, but it can be useful if you’re not used to XML and can prevent lots of errors in the future. For one-line tags, you open the tag and end it with **/>** (for example, **<node />**). If you need to add a tag inside a tag, you then have to open the tag and close it later, like we did for **<launch>...</launch>** or **<node>...</node>**.

From this, let’s say we want to start two **number\_publisher** nodes and one **number\_counter** node. On top of that, we also want to remap the topic from **number** to **my\_number**. Here is the full XML launch file:

```
&lt;launch&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;node pkg="my_py_pkg" exec="number_publisher" name="num_pub1"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/number" to="/my_number" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;/node&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;node pkg="my_py_pkg" exec="number_publisher" name="num_pub2"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/number" to="/my_number" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;/node&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;node pkg="my_cpp_pkg" exec="number_counter"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/number" to="/my_number" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;/node&gt;
&lt;/launch&gt;
```

We rename the two **number\_publisher** nodes to avoid name conflicts. Then, we make sure to add the same **<remap>** tag for all nodes in which we use a publisher or subscriber on the **number** topic.

Additional tip

When you rename nodes and remap communications, use **rqt\_graph** to verify that everything is working fine. With the graphical view, you can easily spot if a topic name is not the same on both sides of the communication.

Here is the code to do the same thing with a Python launch file:

```
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
&nbsp;&nbsp;&nbsp;&nbsp;ld = LaunchDescription()
&nbsp;&nbsp;&nbsp;&nbsp;number_publisher1 = Node(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;package="my_py_pkg",
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;executable="number_publisher",
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;name="num_pub1",
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;remappings=[("/number", "/my_number")]
&nbsp;&nbsp;&nbsp;&nbsp;)
&nbsp;&nbsp;&nbsp;&nbsp;number_publisher2 = Node(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;package="my_py_pkg",
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;executable="number_publisher",
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;name="num_pub2",
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;remappings=[("/number", "/my_number")]
&nbsp;&nbsp;&nbsp;&nbsp;)
&nbsp;&nbsp;&nbsp;&nbsp;number_counter = Node(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;package="my_cpp_pkg",
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;executable="number_counter",
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;remappings=[("/number", "/my_number")]
&nbsp;&nbsp;&nbsp;&nbsp;)
&nbsp;&nbsp;&nbsp;&nbsp;ld.add_action(number_publisher1)
&nbsp;&nbsp;&nbsp;&nbsp;ld.add_action(number_publisher2)
&nbsp;&nbsp;&nbsp;&nbsp;ld.add_action(number_counter)
&nbsp;&nbsp;&nbsp;&nbsp;return ld
```

After renaming and remapping, let’s see how to add parameters to your nodes inside a launch file.

## Parameters in a launch file

Setting parameters’ values for a node in a launch file is pretty straightforward. We will first see how to provide the values directly, and then how to load a YAML file.

### Setting parameters’ values directly

To add a parameter’s value for a node in an XML launch file, you first need to open and close the **<node></node>** tag. Inside this tag, you will add one **<param>** tag per parameter, with two arguments: **name** and **value**.

Here is an example, where we set the **number** and **publish\_period** parameters for the **number\_publisher** node:

```
&lt;node pkg="my_py_pkg" exec="number_publisher"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;param name="number" value="3" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;param name="publish_period" value="1.5" /&gt;
&lt;/node&gt;
```

It will work the same as adding **\-p <parameter>:=<value>** after the **ros2** **run** command.

Now, you can combine renaming, remapping, and setting parameters. Let’s add parameters to the previous example:

```
&lt;node pkg="my_py_pkg" exec="number_publisher" name="num_pub1"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/number" to="/my_number" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;param name="number" value="3" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;param name="publish_period" value="1.5" /&gt;
&lt;/node&gt;
```

In a Python launch file, you need to add a list of dictionaries in the **Node** object:

```
number_publisher1 = Node(
&nbsp;&nbsp;&nbsp;&nbsp;package="my_py_pkg",
&nbsp;&nbsp;&nbsp;&nbsp;executable="number_publisher",
&nbsp;&nbsp;&nbsp;&nbsp;name="num_pub1",
&nbsp;&nbsp;&nbsp;&nbsp;remappings=[("/number", "/my_number")],
&nbsp;&nbsp;&nbsp;&nbsp;parameters=[
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;{"number": 3},
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;{"publish_period": 1.5}
&nbsp;&nbsp;&nbsp;&nbsp;]
)
```

Setting each parameter’s value like this will work fine if you only have a handful of parameters. For bigger numbers, it’s more suitable to use a YAML file.

Note

Do not confuse YAML param files with YAML launch files. Launch files can be written in Python, XML, and YAML (though we didn’t use YAML in this book). Any of those launch files can include YAML param files, to add parameters’ values for the nodes in the launch file.

### Installing and loading a YAML param file in a launch file

To provide parameters’ values using a YAML file, you will need to follow this process:

1.  Create a YAML file with the values.
2.  Install this file inside the **\_bringup** package.
3.  Load the YAML file in your launch file (we will do that with XML and then Python).

For this example, we are going to reuse the **number\_params.yaml** file that we created in [_Chapter 8_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_08.xhtml#_idTextAnchor397). In this file, you can find the following code:

```
/num_pub1:
&nbsp;&nbsp;ros__parameters:
&nbsp;&nbsp;&nbsp;&nbsp;number: 3
&nbsp;&nbsp;&nbsp;&nbsp;publish_period: 0.5
/num_pub2:
&nbsp;&nbsp;ros__parameters:
&nbsp;&nbsp;&nbsp;&nbsp;number: 4
&nbsp;&nbsp;&nbsp;&nbsp;publish_period: 1.0
```

This will perfectly match the nodes that we launched in the previous example, as the names are exactly the same.

Now, what we have done so far is just provide the path to the file when starting a node with **ros2 run**. To use the YAML param file inside a launch file, we will need to install it in the package.

To do that, create a new directory inside the **my\_robot\_bringup** package. You could choose any name for that directory, but we will follow a common convention and name it **config**:

```
$ cd ~/ros2_ws/src/my_robot_bringup/
$ mkdir config
```

Put the **number\_params.yaml** file inside this **config** directory. This is where you will also put all other YAML param files for this application.

Now, to write instructions to install this directory (and all the YAML files inside), open the **CMakeLists.txt** file of the **my\_robot\_bringup** package and add one line:

```
install(DIRECTORY
&nbsp;&nbsp;launch
&nbsp;&nbsp;<strong>config</strong>
&nbsp;&nbsp;DESTINATION share/${PROJECT_NAME}/
)
```

You only need to do this once. Any other file inside the **config** directory will be installed when running **colcon build** for that package.

Before we build the package, let’s modify the launch file so that we can use this YAML param file. The way to do this in XML is easy. You will add a **<param>** tag, but instead of **name** and **value**, you need to specify a **from** argument:

```
&lt;node pkg="my_py_pkg" exec="number_publisher" name="num_pub2"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/number" to="/my_number" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;param from="$(find-pkg-share &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;my_robot_bringup)/config/number_params.yaml" /&gt;
&lt;/node&gt;
```

As we’ve seen previously in this chapter, **$(find-pkg-share <package\_name>)** will locate the installation folder for that package. Then, you only need to finish with the relative path to the file you want to retrieve.

To test this, first build your package. This will install the YAML param files and the launch files. Then, source your environment and start the XML launch file.

That’s it for parameters. Let’s now see the Python version. In your launch file, add the following imports:

```
from ament_index_python.packages import get_package_share_directory
import os
```

Then, retrieve the YAML file:

```
param_config = os.path.join(
&nbsp;&nbsp;&nbsp;&nbsp;get_package_share_directory("my_robot_bringup"),
&nbsp;&nbsp;&nbsp;&nbsp;"config", "number_params.yaml")
```

Finally, load the configuration into the node:

```
number_publisher2 = Node(
&nbsp;&nbsp;&nbsp;&nbsp;...
&nbsp;&nbsp;&nbsp;&nbsp;parameters=[param_config]
)
```

With this, you should be able to start any node you want with any number of parameters without having any scaling issues.

Let’s now finish this section with namespaces. I have briefly mentioned them a few times during this book. As you now have a better understanding of how names work in ROS 2, and as namespaces are especially useful in launch files, this is a good time to start with them.

## Namespaces

Namespaces are quite common in programming, and you are probably already familiar with them. With a namespace, you can group some functionalities (variables, functions, and so on) inside one _container_ that has a name. This can help you better organize your code and avoid name conflicts.

In ROS, namespaces are also quite practical. Let’s say you want to start an application that contains two identical robots, but you want to be able to control each robot independently. Instead of renaming the nodes, topics, services, and actions for each robot, you could just add a namespace.

If you have a node named **robot\_controller** and a topic named **cmd\_vel**, then those can become **/robot1/robot\_controller** and **/robot1/cmd\_vel** for the first robot. For the second robot, this would be **/robot2/robot\_controller** and **/robot2/cmd\_vel**. This way, the two robots are still running on the same application, but you make sure that the velocity command for each robot is independent.

As you make progress with ROS 2 and learn new stacks and plugins, you will encounter namespaces everywhere. Let’s now see how to work with namespaces. As we have not done this previously, we will first use namespaces with the **ros2 run** command line, and then add them in our launch file.

### Starting a node inside a namespace

Adding a namespace to a node is quite straightforward.

First of all, after the **ros2 run <package> <executable>** command, you add **\--ros-args** once. Then, to specify a namespace, you will write **\-r \_\_ns:=<namespace>**. The **\-r** option (or **\--remap**) is the same as the one for renaming a node, only instead of **\_\_node**, you use **\_\_ns** here.

Let’s start our **number\_publisher** node inside a **/****abc** namespace:

```
$ ros2 run my_py_pkg number_publisher --ros-args -r __ns:=/abc [INFO] [1716981935.646395625] [abc.number_publisher]: Number publisher has been started.
```

After this, you can check what the node and topic names are:

```
$ ros2 node list
/abc/number_publisher
$ ros2 topic list
/abc/number
/parameter_events
/rosout
```

As you can see, **/abc** was added to the node name but also to the topic name—if you have services and actions, the namespace will be equally applied.

Important note

The namespace was successfully applied because the topic name defined in the code is **number** without any leading slash. If you had written **/number** in the code, then the topic would have been considered to be in the _global_ scope or namespace. Adding a namespace to the node will change the node name but not the topic name. Thus, pay attention to this when defining communication (topic, service, action) names in your code.

Now, as the topic name is **/abc/number**, if we want to start the **number\_counter** node and receive some data, we need to either rename the topic or also add a namespace to the node:

```
$ ros2 run my_cpp_pkg number_counter --ros-args -r __ns:=/abc
[abc.number_counter]: Number Counter has been started.
[abc.number_counter]: Counter: 2
[abc.number_counter]: Counter: 4
```

When adding namespaces, name mismatches can become a frequent issue. One of the best ways to verify that things are working is to run **rqt\_graph**:

![[attachments/B22403_09_3.jpg]]

Figure 9.3 – Double-checking namespaces with rqt\_graph

With this, you can see that both nodes are publishing or subscribing to the **/****abc/number** topic.

Note

You can combine any type of renaming. For example, you could both add a namespace and rename the node: **$ ros2 run my\_py\_pkg number\_publisher --ros-args -r \_\_ns:=/abc -****r \_\_node:=num\_pub**.

Now that you know how to provide a namespace for a node at runtime, let’s see how to do this inside a launch file.

### Specifying a namespace in a launch file

To add a namespace to a node in an XML launch file, you just have to add a **namespace** argument inside the **<node>** tag. Let’s continue with our previous example:

```
&lt;node pkg="my_py_pkg" exec="number_publisher" name="num_pub1" <strong>namespace="/abc"</strong>&gt;
```

For Python, the syntax is also quite easy; here too, you just need to add a **namespace** argument inside the **Node** object:

```
number_publisher1 = Node(
&nbsp;&nbsp;&nbsp;&nbsp;package="my_py_pkg",
&nbsp;&nbsp;&nbsp;&nbsp;executable="number_publisher",
&nbsp;&nbsp;&nbsp;&nbsp;<strong>namespace="/abc",</strong>
&nbsp;&nbsp;&nbsp;&nbsp;name="num_pub1",
&nbsp;&nbsp;&nbsp;&nbsp;...
```

If you add a namespace to this node, you will also add the same namespace to nodes that are directly communicating with it:

```
&lt;node pkg="my_py_pkg" exec="number_publisher" name="num_pub1" <strong>namespace="/abc"</strong>&gt;
...
&lt;node pkg="my_cpp_pkg" exec="number_counter" <strong>namespace="/abc"</strong>&gt;
```

Adding namespaces to nodes in a launch file is quite straightforward. However, there is one important thing you need to pay attention to. If you are using YAML param files, you also need to specify the namespace in the YAML file. Open the **number\_params.yaml** file and add the namespace to the node name:

```
<strong>/abc</strong>/num_pub2:
 ros__parameters:
&nbsp;&nbsp;&nbsp;number: 4
&nbsp;&nbsp;&nbsp;publish_period: 1.0
```

If you don’t do this, the parameters will be applied to the **/num\_pub2** node, which doesn’t exist, since it’s named **/abc/num\_pub2**. This can be a common source of errors, so make sure you double-check param files when adding namespaces.

After all those modifications, make sure to build the **my\_robot\_bringup** package again and source the environment before you start any launch file.

You have now seen a few ways to configure your nodes inside a launch file. With this base knowledge, you can already scale your application a lot. Let’s finish this chapter with a new challenge so that you can practice more on your own.

## Launch file challenge

In this challenge, you will practice more with launch files, YAML param files, remappings, and namespaces. This will be the conclusion of _Part 2_. To complete this challenge, you can decide to write the launch file in XML, Python, or both.

## Challenge

What we want to do here is to start two **turtlesim** windows, each one with one turtle. Then, for each turtle, we run a **turtle\_controller** node (the one we have been developing throughout the previous chapters).

The goal is to have each **turtle\_controller** node controlling only one turtle. This is what the result should look like:

![[attachments/B22403_09_4.jpg]]

Figure 9.4 – Two different turtles with two independent controllers

For each turtle, we will apply different settings (parameters):

-   First **turtlesim** window:
    -   **Background color**: Red (you can pick **128** for the RGB value)
-   First controller:
    -   **Color** **1**: Black
    -   **Color** **2**: White
    -   **Velocity**: **1.5**
-   Second **turtlesim** window:
    -   **Background color**: Green (you can pick **128** for the RGB value)
-   Second controller:
    -   **Color** **1**: White
    -   **Color** **2**: Black
    -   **Velocity**: **0.5**

Here are the steps you can take:

1.  Create a **turtle\_params.yaml** file with the parameters for each node. Install this in the **my\_robot\_bringup** package.
2.  Create a new launch file and start the four nodes. Load the parameters from the YAML param file. Put the different nodes into appropriate namespaces (to keep it simple, use **t1** and **t2** for **turtle1** and **turtle2**, respectively).
3.  Build, source, and start the launch file. You will see that some topics and services are not matching, and thus you will know what remappings you need to add.

To make it easier, start with just one pair of nodes (**turtlesim** and **turtle\_controller**), and then add another pair when it’s working.

Here is an important point for this challenge: we will not modify any of the existing code—even if it would make things easier. The goal is to take the nodes exactly as they are (use the code from the **ch8** folder in the repo) and make things work using appropriate namespaces and remappings in the launch file and YAML param file.

## Solution

Create a new file named **turtle\_params.yaml**, inside the **config** directory of the **my\_robot\_bringup** package. As a base, you can take the one that we did in the parameter challenge for [_Chapter 8_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_08.xhtml#_idTextAnchor397).

In this file, we will add parameters for all four nodes. Before we do this, we need to know exactly what will be the name for each node, including the namespaces.

With the **t1** and **t2** namespaces, if we just add a namespace and we don’t rename the nodes, we will then have these names:

-   **/****t1/turtlesim**
-   **/****t2/turtlesim**
-   **/****t1/turtle\_controller**
-   **/****t2/turtle\_controller**

After making this choice, we can write the YAML param file:

```
/t1/turtlesim:
&nbsp;&nbsp;ros__parameters:
&nbsp;&nbsp;&nbsp;&nbsp;background_r: 128
&nbsp;&nbsp;&nbsp;&nbsp;background_g: 0
&nbsp;&nbsp;&nbsp;&nbsp;background_b: 0
/t2/turtlesim:
&nbsp;&nbsp;ros__parameters:
&nbsp;&nbsp;&nbsp;&nbsp;background_r: 0
&nbsp;&nbsp;&nbsp;&nbsp;background_g: 128
&nbsp;&nbsp;&nbsp;&nbsp;background_b: 0
/t1/turtle_controller:
&nbsp;&nbsp;ros__parameters:
&nbsp;&nbsp;&nbsp;&nbsp;color_1: [0, 0, 0]
&nbsp;&nbsp;&nbsp;&nbsp;color_2: [255, 255, 255]
&nbsp;&nbsp;&nbsp;&nbsp;turtle_velocity: 1.5
/t2/turtle_controller:
&nbsp;&nbsp;ros__parameters:
&nbsp;&nbsp;&nbsp;&nbsp;color_1: [255, 255, 255]
&nbsp;&nbsp;&nbsp;&nbsp;color_2: [0, 0, 0]
&nbsp;&nbsp;&nbsp;&nbsp;turtle_velocity: 0.5
```

This contains all the configurations given in the challenge. Now, create a new launch file (for example, **turtlesim\_control.launch.xml**) inside the **launch** directory.

In this launch file, let’s start with something simple. We want to try to run one **turtlesim** node and one **turtle\_controller** node, using the **t1** namespace:

```
&lt;launch&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;node pkg="turtlesim" exec="turtlesim_node" namespace="t1"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;param from="$(find-pkg-share &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;my_robot_bringup)/config/turtle_params.yaml" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;/node&gt;
```

```
&nbsp;&nbsp;&nbsp;&nbsp;&lt;node pkg="turtle_controller" exec="turtle_controller" namespace="t1"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&lt;param from="$(find-pkg-share &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;my_robot_bringup)/config/turtle_params.yaml" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;/node&gt;
&lt;/launch&gt;
```

As we are starting nodes from the **turtlesim** and **turtle\_controller** packages, we also add two new **<exec\_depend>** tags in the **package.xml** file:

```
&lt;exec_depend&gt;turtlesim&lt;/exec_depend&gt;
&lt;exec_depend&gt;turtle_controller&lt;/exec_depend&gt;
```

Now, if you launch this (make sure to build and source first), you will see the **turtlesim** node, but the turtle won’t move. Why is that?

If you look at the topic list, you will find these two topics:

```
$ ros2 topic list
/t1/turtle1/cmd_vel
/turtle1/cmd_vel
```

With **rqt\_graph**, you can also see that the **turtlesim** node is subscribing to **/t1/turtle1/cmd\_vel**, but the **turtle\_controller** node is publishing on **/turtle1/cmd\_vel**. Why did the namespace work for the node name but not for the topic name?

This is because we wrote **/turtle1/cmd\_vel** in the code, and not **turtle1/cmd\_vel**. The fact that we added a leading slash makes the namespace the _global_ namespace. Thus, if you try to add a namespace to that, it will not be taken into account.

We have two options here: either we modify the code (we simply need to remove this leading slash) or we adapt the launch file to make this work. As specified in the challenge instructions, we are not going to modify the code. The reason why I’m adding this constraint is because, in real life, you won’t necessarily be able to modify the code of the nodes you run. Thus, knowing how to solve a name mismatch without touching the code is a great skill to have.

So, if you look at the topic and service names (we don’t use actions here), you will see that we have two topics and one service to modify. Let’s add some **<remap>** tags inside the node:

```
&lt;node pkg="turtle_controller" exec="turtle_controller" namespace="t1"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;param from="$(find-pkg-share &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;my_robot_bringup)/config/turtle_params.yaml" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/turtle1/pose" to="/t1/turtle1/pose" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/turtle1/cmd_vel" to="/t1/turtle1/cmd_vel" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/turtle1/set_pen" to="/t1/turtle1/set_pen" /&gt;
&lt;/node&gt;
```

You can now start the launch file, and you will see the turtle moving. Now that we have this working, adding a second pair of nodes is easy. We basically need to copy/paste the two nodes and replace **t1** with **t2**:

```
&lt;node pkg="turtlesim" exec="turtlesim_node" namespace="t2"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;param from="$(find-pkg-share &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;my_robot_bringup)/config/turtle_params.yaml" /&gt;
&lt;/node&gt;
&lt;node pkg="turtle_controller" exec="turtle_controller" namespace="t2"&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;param from="$(find-pkg-share &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;my_robot_bringup)/config/turtle_params.yaml" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/turtle1/pose" to="/t2/turtle1/pose" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/turtle1/cmd_vel" to="/t2/turtle1/cmd_vel" /&gt;
&nbsp;&nbsp;&nbsp;&nbsp;&lt;remap from="/turtle1/set_pen" to="/t2/turtle1/set_pen" /&gt;
&lt;/node&gt;
```

The challenge is now complete. If you start this launch file, you will see two **turtlesim** windows, each one containing a turtle that moves at a different speed and using different pen colors.

You can find the complete code and package organization in the book’s GitHub repository (including the Python launch file).

## Summary

In this chapter, you worked on ROS 2 launch files. Launch files allow you to properly scale your application with multiple nodes, parameters, and sets of configuration.

You can write a launch file in Python, XML, or YAML. Here, you discovered the Python and XML syntax and saw that XML is probably the best choice by default. The syntax is much easier, and the code is much shorter. If you ever need to combine XML and Python launch files, you can do so by including a launch file in another one.

The best practice is to set up a dedicated package for launch files and YAML files. You can name the package using the **\_bringup** suffix. Launch files will be installed in a **launch** folder, and YAML param files in a **config** folder.

If you correctly understand how to start nodes with the **ros2 run command**, then doing so in a launch file is pretty straightforward: you just need to provide the package and executable name for each node. The only thing to learn is the XML or Python syntax.

In a launch file, you can also configure your nodes in multiple ways:

-   Renaming the node and/or adding a namespace
-   Remapping topics, services, and actions
-   Adding parameters, individually or from a YAML param file

This is what we have seen so far, but there are many other ways to configure your nodes that you will discover throughout your ROS 2 learning journey.

_Part 2_ of this book is now finished. You have discovered all the core concepts that will allow you to write complete ROS 2 applications and join existing ROS 2 projects. You should now be able to interact with any ROS 2 node, write code to communicate with it, and scale your application with parameters and launch files.

Now, this part was heavily focused on programming (Python and C++), which is incredibly important, but ROS 2 is more than just that. In _Part 3_, we will dive into some additional concepts and tools (**TransForms** (**TFs**), **Unified Robot Description Format** (**URDF**), **Gazebo**) so that you can design a custom application for a robot, including a 3D simulation. This, combined with the programming we did in _Part 2_, will be the backbone of any ROS 2 application you work on.

<table id="table001-4"><colgroup></colgroup><colgroup><col></colgroup><colgroup><col></colgroup><tbody><tr><td><h4>Unlock this book’s exclusive benefits now</h4><p>This book comes with additional benefits designed to elevate your learning experience.</p></td><td rowspan="2"><p><img alt="" width="246" height="238" src="attachments/9781835881408.png"></p><p lang="en-US" xml:lang="en-US"><a href="https://www.packtpub.com/unlock/9781835881408" target="_blank" rel="noopener noreferrer">https://www.packtpub.com/unlock/9781835881408</a></p></td></tr><tr><td><p><em>Note: Have your purchase invoice ready before </em><span><em>you begin.</em></span></p></td></tr></tbody></table>
