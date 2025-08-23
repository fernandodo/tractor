---
created: 2025-08-14T13:00:06 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_08.xhtml#_idParaDest-192
author: 
---

# Chapter 8: Parameters – Making Nodes More Dynamic | ROS 2 from Scratch

> ## Excerpt
> 8
Parameters – Making Nodes More Dynamic
 We are now done with the basics of ROS 2 communications. In this chapter, we will continue to work on nodes, but this time by making them...

---
## Parameters – Making Nodes More Dynamic

We are now done with the basics of ROS 2 communications. In this chapter, we will continue to work on nodes, but this time by making them more dynamic with **parameters**.

To understand parameters, I will start with why we need them in the first place. Then, you will learn how to add parameters to your nodes so that you can customize them at runtime. You will also see how to load multiple parameters at once with **YAML** files and how to allow parameters to be modified in your code with **parameter callbacks**.

As a starting point, we will use the code inside the **ch7** folder of the book’s GitHub repository ([https://github.com/PacktPublishing/ROS-2-from-Scratch](https://github.com/PacktPublishing/ROS-2-from-Scratch)). If you skipped _actions_ ([_Chapter 7_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_07.xhtml#_idTextAnchor341)), you can also start from the **ch6** folder, which will work the same. The final code for this chapter will be in the **ch8** folder.

By the end of this chapter, you will be able to add parameters to any of your nodes and handle parameters for other nodes that you start.

The concept of parameters is not too difficult, and there won’t be too much to do in the code. However, it’s an important concept and the first step toward making your application more dynamic and scalable.

In this chapter, we will cover the following topics:

-   What is a ROS 2 parameter?
-   Using parameters in your nodes
-   Storing parameters in YAML files
-   Additional tools to handle parameters
-   Updating parameters with parameter callbacks
-   Parameter challenge

## What is a ROS 2 parameter?

You have already experimented a bit with parameters in [_Chapter 3_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_03.xhtml#_idTextAnchor095), where you ran a node with different settings.

I am now going to start from scratch again and explain parameters with a real-life example.

## Why parameters?

Let’s start with a problem to understand the need for parameters. I will use a camera driver as an example—we won’t write the node; it’s just for the explanation.

This camera driver connects to a USB camera, reads images, and publishes them on a ROS 2 topic. This is classic behavior for any ROS 2 hardware driver.

Inside this node, you will have some variables for different kinds of settings. Here are some examples:

-   USB device name
-   Frames per second (FPS)
-   Simulation mode

Let’s say the camera you’re working on is connected to the **/dev/ttyUSB0** port (typical USB port name on Linux). You want to set **60** FPS and not use the simulation mode (**false**). Those are the values you will write for the variables inside your node.

Later on, if the USB device name is different (for example, **/dev/ttyUSB1**), you will have to change that setting in your code and maybe build again—you'll do the same thing if you want to start your camera with **30** FPS instead of **60** FPS, or if you want to run it in simulation mode.

Also, what if you have two cameras, and you want to use them both at the same time? Will you duplicate your code for each camera? How can you handle the different settings for both cameras?

As you can see, hardcoding those settings in your code is not a great option for reusability. This is why we have ROS 2 parameters.

## Example of a node with parameters

A ROS 2 parameter is basically a setting for a node that you can modify when you start the node.

So, if we keep the camera driver example, we could add three parameters—USB device name (string), FPS value (integer), and simulation mode (boolean):

![[attachments/B22403_08_1.jpg]]

Figure 8.1 – A node class with three parameters

When you start this camera driver with **ros2 run** (we will see how to do that later in this chapter), you will be able to provide the values you want for those three parameters.

Let’s say you want to start two nodes for two different cameras, given the following settings:

1.  Port: **/dev/ttyUSB0**; FPS: **30**; simulation mode: off
2.  Port: **/dev/ttyUSB1**; FPS: **60**; simulation mode: off

With the parameters we’ve added in the code, we can start the same node multiple times with different values:

![[attachments/B22403_08_2.jpg]]

Figure 8.2 – Starting two nodes with different settings

From the same code, we start two different nodes. At runtime, we rename the nodes (because we can’t have two nodes with the same name), and we provide the parameters’ values.

Our two camera nodes are now running, each with a different configuration. You could stop one camera node and start it again with a different set of values.

## ROS 2 parameters – wrapping things up

With parameters, you can reuse the same code and start several nodes with different settings. There’s no need to compile or build anything again; you just have to provide the parameters’ values at runtime.

Making your nodes customizable allows for greater flexibility and reusability. Your application will become much more dynamic.

Parameters are also very convenient for collaborating with other ROS developers. If you develop a node that could be reused by others, then with parameters, you allow other developers to fully customize the node without even having to look at the code. This also applies when using existing nodes. Lots of them can be configured at runtime.

Here are a few important points about parameters:

-   Just as with a variable, a parameter has a name and a data type. Among the most common types, you can use booleans, integer numbers, float numbers, strings, and lists of those types.
-   A parameter’s value is specific to a node. If you kill the node, the value is gone with it.
-   You can set the value for each parameter when you start a node with **ros2 run** (or from a launch file, which we will see in the next chapter).

Now, how to add parameters to your code? As for nodes, topics, and services, you will get everything you need from the **rclpy** and **rclcpp** libraries. You will be able to declare the parameters in your code and get the value for each parameter.

## Using parameters in your nodes

We will continue with the code we have written in the previous chapters. Here, we will improve the **number\_publisher** node. As a quick recap, this node publishes a number on a topic, at a given rate. The number and publishing rate are directly written in the code.

Now, instead of hardcoding the number and publishing rate values, we will use parameters. This way, we will be able to specify what number to publish, and the publishing frequency or period, when we start the node.

You need to follow two steps to be able to use a parameter in your code:

1.  Declare the parameter in the node. This will make the parameter exist within the node so that you can set a value to it when starting the node with **ros2 run**.
2.  Retrieve the parameter’s value so that you can use it in the code.

Let’s start with Python, and later on, we will also see the C++ code.

## Declaring, getting, and using parameters with Python

Before using a parameter, we need to declare it. Where should we declare parameters? We will do that in the node’s constructor, before everything else. To declare a parameter, use the **declare\_parameter()** method from the **Node** class.

You will provide two arguments:

-   **Parameter name**: This is the name that you will use to set the parameter’s value at runtime
-   **Default value**: If the parameter’s value is not provided at runtime, this value will be used

There are, in fact, different ways to declare a parameter. You don’t necessarily need to provide a default value if you provide the parameter type instead. However, we will keep things like that, as it will probably make your life easier. Adding a default value for each parameter is a best practice to follow.

Open the **number\_publisher.py** file, and let’s declare two parameters in the constructor:

```
self.declare_parameter("number", 2)
self.declare_parameter("publish_period", 1.0)
```

![[attachments/3.png]] **Quick tip**: Enhance your coding experience with the **AI Code Explainer** and **Quick Copy** features. Open this book in the next-gen Packt Reader. Click the **Copy** button (**1**) to quickly copy code into your coding environment, or click the **Explain** button (**2**) to get the AI assistant to explain a block of code to you.

![[attachments/image_(2).png]]

![[attachments/4.png]] **The next-gen Packt Reader** is included for free with the purchase of this book. Unlock it by scanning the QR code below or visiting [https://www.packtpub.com/unlock/9781835881408](https://www.packtpub.com/unlock/9781835881408).

![[attachments/9781835881408.png]]

A parameter is defined by a name and a data type. Here, you choose the name, and the data type will be automatically set depending on the default value you have provided. In this example, the default value for **number** is **2**, which means that the parameter’s data type is integer. For the **publish\_period** parameter, the default value is **1.0**, which is a float number.

Here are a few more examples of different data types:

-   **Booleans**: **self.declare\_parameter("simulation\_mode", False)**
-   **String**: **self.declare\_parameter("device\_name", "/dev/ttyUSB0")**
-   **Integer array**: **self.declare\_parameter("numbers", \[4,** **5, 6\])**

Now, declaring a parameter means that it exists within the node, and you can set a value from the outside. However, in your code, to be able to use the parameter, it’s not enough to declare it. After doing that, you need to get the value.

For this, you will use the **get\_parameter()** method, and provide the parameter’s name as an argument. Then, you can access the value with the **value** attribute:

```
self.number_ = self.get_parameter("number").value
self.timer_period_ = self.get_parameter(
&nbsp;&nbsp;&nbsp;&nbsp;"publish_period"
).value
```

At this point in the code, the **number\_** variable (which is a class attribute) contains the value that was set for the **number** parameter at runtime with **ros2 run**.

Note

You always need to declare a parameter before getting its value. If you fail to do so, when starting the node, you will get an exception (**ParameterNotDeclaredException**) as soon as you try to get the value.

After you get the values for all parameters and store them inside variables or class attributes, you can use them in your code. Here, we modify the timer callback:

```
self.number_timer_ = self.create_timer(
&nbsp;&nbsp;&nbsp;&nbsp;self.timer_period_, self.publish_number
)
```

With this, we set the publishing period from the parameter’s value.

That’s pretty much it for the code. As you can see, there is nothing too complicated. For one parameter, you will just add two instructions: one to declare the parameter (give it a name and a default value), and another to get its value.

Now, I’ve been talking about setting a parameter’s value at runtime with **ros2 run**. How do we do that?

## Providing parameters at runtime

Before going further, make sure to save the **number\_publisher.py** file and build the **my\_py\_pkg** package (if you haven’t used **\--****symlink-install** before).

To provide a parameter’s value with the **ros2 run** command, follow the next steps:

1.  You will first start your node with **ros2 run <****package\_name> <exec\_name>**.
2.  Then, to add any argument after this command, you have to write **\--ros-args** (only once).
3.  To specify a parameter’s value, write **\-p <param\_name>:=<param\_value>**. You can add as many parameters as you want.

Let’s say we want to start the node and publish the number **3** every **0.5** seconds. In that case, we’d run the following command:

```
$ ros2 run my_py_pkg number_publisher --ros-args -p number:=3 -p publish_period:=0.5
```

To verify it’s working, we can subscribe to the **/****number** topic:

```
$ ros2 topic echo /number
data: 3
---
data: 3
---
```

We can also verify the publish rate:

```
$ ros2 topic hz /number
average rate: 2.000
&nbsp;&nbsp;&nbsp;&nbsp;min: 0.500s max: 0.500s std dev: 0.00004s window: 3
```

So, what happened? You provided some values for different parameters at runtime. The node will start and recognize those parameters because they match the names that have been declared in the code. Then, the node can get the value for each parameter.

If you provide the wrong data type for a parameter, you will get an error. As seen previously, the data type is set in the code from the default value. In this example, the **number** parameter should be an integer number. Look at what happens if we try to set a double value:

```
$ ros2 run my_py_pkg number_publisher --ros-args -p number:=3.14
…
&nbsp;&nbsp;&nbsp;&nbsp;raise InvalidParameterTypeException(
rclpy.exceptions.InvalidParameterTypeException: Trying to set parameter 'number' to '3.14' of type 'DOUBLE', expecting type 'INTEGER': number
[ros2run]: Process exited with failure 1
```

As you can see, once a parameter type is set in the code, you have to use that exact same type whenever you provide a value at runtime.

As each parameter has a default value, you could also omit one or more parameters:

```
$ ros2 run my_py_pkg number_publisher --ros-args -p number:=3
```

In this case, the **publish\_period** parameter will be set to its default value (**1.0**), defined in the code.

To finish here, let’s just see an example where renaming the node and setting parameters’ values can allow you to run several different nodes from the same code without having to modify anything in the code.

In Terminal 1, run the following:

```
$ ros2 run my_py_pkg number_publisher --ros-args -r __node:=num_pub1 -p number:=3 -p publish_period:=0.5
```

In Terminal 2, run the following:

```
$ ros2 run my_py_pkg number_publisher --ros-args -r __node:=num_pub2 -p number:=4 -p publish_period:=1.0
```

With this, you have two nodes (**num\_pub1** and **num\_pub2**), both publishing to the **/number** topic but with different data and publishing rates. With this example, you can see that parameters are a great way to make your nodes more dynamic.

Let’s now finish this section with the C++ code for parameters.

## Parameters with C++

Parameters work the same for Python and C++; only the syntax differs. Here, we will modify the **number\_publisher.cpp** file.

In the constructor, you can declare some parameters:

```
this-&gt;declare_parameter("number", 2);
this-&gt;declare_parameter("publish_period", 1.0);
```

We use the **declare\_parameter()** method from the **rclcpp::Node** class. The arguments are the same as for Python: name and default value. From this value, the parameter type will be set.

Then, to get a parameter’s value in the code, write the following:

```
number_ = this-&gt;get_parameter("number").as_int();
double timer_period = this-&gt;get_parameter("publish_period")
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;.as_double();
```

We use the **get\_parameter()** method and provide the name for the parameter. Then, we get the value with the method that corresponds to the data type: **as\_int()**, **as\_double()**, **as\_string()**, **as\_string\_array()**, and so on. If you have an IDE with auto-completion, you should be able to see all possible types.

The rest is the same as for Python. Please refer to the GitHub files for any other minor changes and additions.

To start a node with parameters, run the following:

```
$ ros2 run my_cpp_pkg number_publisher --ros-args -p number:=4 -p publish_period:=1.2
```

Working with parameters is not that tough. For each parameter you want to create, you have to declare it in the code and get its value. When starting the node from the terminal, you can specify a value for each parameter you want.

Now, that works well only if you have a small number of parameters. In a real application, it’s not uncommon to have a few dozen or even hundreds of parameters for a node. How can you manage so many parameters?

## Storing parameters in YAML files

As your ROS 2 application grows, so will the number of parameters. Adding 10 or more parameters from the command line is not really an option anymore.

Fortunately, you can use YAML files to store your parameters, and you can load these files at runtime. If you don’t know YAML, it’s basically a markup language, similar to XML and JSON, but supposedly more readable by humans.

In this section, you will learn how to add your parameters to a YAML file and how to load this file at runtime.

## Loading parameters from a YAML file

Let’s start by saving parameters into a file so we can use them when we start a node.

First, create a YAML file with the **.yaml** extension. The filename doesn’t matter that much, but it’s better to give it a meaningful name. As our application deals with numbers, we can name it **number\_params.yaml**.

For now, let’s just create a new file in our home directory (in the next chapter, we will see how to properly install a YAML file in a ROS 2 application):

```
$ cd ~
$ touch number_params.yaml
```

Edit this file and add parameters for the **/****number\_publisher** node:

```
/number_publisher:
&nbsp;&nbsp;ros__parameters:
&nbsp;&nbsp;&nbsp;&nbsp;number: 7
&nbsp;&nbsp;&nbsp;&nbsp;publish_period: 0.8
```

First, you write the name of the node. On the next line, and with an indentation (usually, it’s recommended to use two spaces), we add **ros\_\_parameters** (make sure you use two underscores). This will be the same for every node you add in a YAML file. On the following lines, and with yet another indentation, you can add all the parameters’ values for the node.

Note

It’s important that the node name matches; otherwise, the parameters won’t be loaded into the node. If you omit the leading slash, it would still work for loading parameters with **ros2 run**, but you could have issues with other commands.

Once you’ve written this file, you can load the parameters with the **\--****params-file** argument:

```
$ ros2 run my_py_pkg number_publisher --ros-args --params-file ~/number_params.yaml
```

This will start the node and specify the values for the **number** and **publish\_period** parameters.

If you have two or fifty parameters, the **ros2 run** command stays the same. All you have to do is add more parameters in the YAML file. If you want to modify a parameter, you can modify the corresponding line in the file or even create several files for different sets of configurations.

## Parameters for multiple nodes

What should you do if you want to save parameters for several nodes?

Good news: inside one param YAML file, you can add the configuration for as many nodes as you want. Here’s an example:

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

This corresponds to the example we ran before, with two nodes and different parameters.

Now, to start the same nodes and parameters, we only need to run the commands shown next.

In Terminal 1, run the following:

```
$ ros2 run my_py_pkg number_publisher --ros-args -r __node:=num_pub1 --params-file ~/number_params.yaml
```

In Terminal 2, run the following:

```
$ ros2 run my_py_pkg number_publisher --ros-args -r __node:=num_pub2 --params-file ~/number_params.yaml
```

We give the same YAML file to both nodes. Each node will only load the parameters’ values that are defined under the node name.

## Recapping all parameters’ data types

Let’s say you have all those parameters declared in your code (Python example only, but you can easily translate to C++):

```
self.declare_parameter("bool_value", False)
self.declare_parameter("int_number", 1)
self.declare_parameter("float_number", 0.0)
self.declare_parameter("str_text", "Hola")
self.declare_parameter("int_array", [1, 2, 3])
self.declare_parameter("float_array", [3.14, 1.2])
self.declare_parameter("str_array", ["default", "values"])
self.declare_parameter("bytes_array", [0x03, 0xA1])
```

Those are basically all the available data types for parameters.

To specify the value for each parameter, you can create a YAML file or add some configuration to an existing YAML file. Here is what you would write for this node (named **your\_node**):

```
/your_node:
&nbsp;&nbsp;ros__parameters:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;bool_value: True
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;int_number: 5
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;float_number: 3.14
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;str_text: "Hello"
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;bool_array: [True, False, True]
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;int_array: [10, 11, 12, 13]
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;float_array: [7.5, 400.4]
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;str_array: ['Nice', 'more', 'params']
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;bytes_array: [0x01, 0xF1, 0xA2]
```

With YAML files, you will be able to customize your nodes in a quick and efficient way. I recommend using them as soon as you get more than a few parameters.

Also, as you continue your journey with ROS 2, you will start to use nodes and complete stacks developed by other developers. Those nodes often come with a bunch of YAML files that allow you to configure the stack without having to change anything in the nodes directly.

Let’s now continue with the command-line tools. You have set the parameters’ values with **ros2 run**, but there are actually more tools to handle parameters.

## Additional tools for handling parameters

You start to get used to it: for each ROS 2 core concept, we get a dedicated **ros2** command-line tool. For parameters, we have **ros2 param**.

You can see all the commands with **ros2 param -h**. Let’s focus on the most important ones so that we can get parameters’ values from the terminal and set some values after the node has been started. At the end of this section, we will also explore the different parameter services available for all nodes.

## Getting parameters’ values from the terminal

After you’ve started one or several nodes, you can list all available parameters with **ros2** **param list**.

Stop all nodes and start two nodes, **num\_pub1** and **num\_pub2**, either by using the YAML file or by providing the parameters’ values manually.

In Terminal 1, run the following:

```
$ ros2 run my_py_pkg number_publisher --ros-args -r __node:=num_pub1 -p number:=3 -p publish_period:=0.5
```

In Terminal 2, run the following:

```
$ ros2 run my_py_pkg number_publisher --ros-args -r __node:=num_pub2 -p number:=4 -p publish_period:=1.0
```

Now, list all available parameters:

```
$ ros2 param list
/num_pub1:
&nbsp;&nbsp;number
&nbsp;&nbsp;publish_period
&nbsp;&nbsp;start_type_description_service
&nbsp;&nbsp;use_sim_time
/num_pub2:
&nbsp;&nbsp;number
```

```
&nbsp;&nbsp;publish_period
&nbsp;&nbsp;start_type_description_service
&nbsp;&nbsp;use_sim_time
```

Here, I started two nodes to show you that each node gets its own set of parameters. The **number** parameter inside **/num\_pub1** is not the same as the **number** parameter inside **/num\_pub2**.

Note

For each parameter, we also always get the **use\_sim\_time** parameter with a default value of **false**. This means that we use the system clock. We would set it to **true** if we were simulating the robot so that we could use the simulation engine clock instead. This is not important for now, and you can ignore this parameter. You can also ignore the **start\_type\_description\_service** parameter.

From this, you can get the value for one specific parameter, using **ros2 param get <****node\_name> <param\_name>**:

```
$ ros2 param get /num_pub1 number
Integer value is: 3
$ ros2 param get /num_pub2 number
Integer value is: 4
```

This corresponds to the values we have set when starting the node. Using **ros2 param get** allows you to introspect the parameters inside any running node.

## Exporting parameters into YAML

If you’d like to get the complete set of parameters for a node, you can do so with **ros2 param** **dump <node\_name>**.

Let’s dump all parameters for the nodes we are running.

For the first node, run the following:

```
$ ros2 param dump /num_pub1
/num_pub1:
&nbsp;&nbsp;ros__parameters:
&nbsp;&nbsp;&nbsp;&nbsp;number: 3
&nbsp;&nbsp;&nbsp;&nbsp;publish_period: 0.5
&nbsp;&nbsp;&nbsp;&nbsp;start_type_description_service: true
&nbsp;&nbsp;&nbsp;&nbsp;use_sim_time: false
```

For the second node, run the following:

```
$ ros2 param dump /num_pub2
/num_pub2:
&nbsp;&nbsp;ros__parameters:
&nbsp;&nbsp;&nbsp;&nbsp;number: 4
&nbsp;&nbsp;&nbsp;&nbsp;publish_period: 1.0
&nbsp;&nbsp;&nbsp;&nbsp;start_type_description_service: true
&nbsp;&nbsp;&nbsp;&nbsp;use_sim_time: false
```

As you can see, the output is exactly what you need to write inside a YAML file. You can then just copy and paste what you get in the terminal and create your own YAML file to load later (there’s no need to set **use\_sim\_time** and **start\_type\_description\_service**).

This **ros2 param dump** command can be useful for getting all parameters’ values at once and for building a param YAML file quickly.

## Setting a parameter’s value from the terminal

Parameters are actually not set in stone for the entire life of a node. After you initialize a parameter’s value with **ros2 run**, you can modify it from the terminal.

With our camera driver example, let’s say you disconnect and reconnect the camera. The device name might change on Linux. If it were **/dev/ttyUSB0**, now it could be **/dev/ttyUSB1**. You could stop and start the node again with a different value for the device name parameter, but with the **ros2 param set** command, you could also just change the value directly while the node is still running.

To show you how it works, let’s come back to our number application.

Stop all nodes and start one **number\_publisher** node (here, I don’t provide any parameter; we will use the default values):

```
$ ros2 run my_py_pkg number_publisher
```

Let’s just verify the value for the **number** parameter:

```
$ ros2 param get /number_publisher number
Integer value is: 2
```

To modify a parameter from the terminal, you have to run **ros2 param set <node\_name> <param\_name> <new\_value>**, as in the following example:

```
$ ros2 param set /number_publisher number 3
Set parameter successful
```

Of course, make sure to provide the correct data type for the parameter; otherwise, you will get an error. You can also load a YAML file directly with **ros2 param load <node\_name> <yaml\_file>** so that you can set several parameters at the same time:

```
$ ros2 param load /number_publisher ~/number_params.yaml
Set parameter number successful
Set parameter publish_period successful
```

After modifying a parameter, we check the parameter’s value again:

```
$ ros2 param get /number_publisher number
Integer value is: 7
```

As you can see, the value was successfully changed. However, did this really work? Is the new parameter’s value used in the code?

Let’s verify that we are publishing the correct number:

```
$ ros2 topic echo /number
data: 2
---
```

Even if we changed the parameter’s value, the new value was not updated inside the code. To do that, we will need to add a parameter callback. That’s what we will see in a minute, but for now, let’s just finish this section with the extra existing services that allow you to manage parameters.

## Parameter services

If you remember, when we worked on services, you saw that for each node, we got an additional set of seven services, most of them related to parameters.

List all services for the **number\_publisher** node:

```
$ ros2 service list
/number_publisher/describe_parameters
/number_publisher/get_parameter_types
/number_publisher/get_parameters
/number_publisher/get_type_description
/number_publisher/list_parameters
/number_publisher/set_parameters
/number_publisher/set_parameters_atomically
```

With those services, you can list parameters, get their value, and even set new values. Those services basically give you the same functionalities as the **ros2 param** command-line tool.

This is good news because getting and setting parameters from the terminal is not really practical and scalable in a real application. By using those services, you can create a service client in node A, which will get or modify parameters in node B.

I will not dive too far into this; you can experiment on your own with what you saw in [_Chapter 6_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_06.xhtml#_idTextAnchor285). Let’s just do a quick example here by modifying the **number** parameter. Let’s first check which interface you need to use:

```
$ ros2 service type /number_publisher/set_parameters
rcl_interfaces/srv/SetParameters
```

Then, you can get more details with **ros2 interface show**. Finally, you can create a service client (inside a node) to modify a parameter. Let’s do so from the terminal:

```
$ ros2 service call /number_publisher/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: 'number', value: {type: 2, integer_value: 3}}]}"
```

This is the same as running **ros2 param set /number\_publisher number 3**. The benefit of the service is that you can use it inside any of your other nodes, with a service client from **rclpy** or **rclcpp**.

If you’re wondering what **type: 2** means in the service request, here are all the types you can get or set with the parameter services:

```
$ ros2 interface show rcl_interfaces/msg/ParameterType
uint8 PARAMETER_NOT_SET=0
uint8 PARAMETER_BOOL=1
uint8 PARAMETER_INTEGER=2
uint8 PARAMETER_DOUBLE=3
uint8 PARAMETER_STRING=4
uint8 PARAMETER_BYTE_ARRAY=5
uint8 PARAMETER_BOOL_ARRAY=6
uint8 PARAMETER_INTEGER_ARRAY=7
uint8 PARAMETER_DOUBLE_ARRAY=8
uint8 PARAMETER_STRING_ARRAY=9
```

So, the number **2** corresponds to the **PARAMETER\_INTEGER** type.

Now that you’ve seen how to set a parameter’s value while the node is already running, let’s continue with parameter callbacks. The problem so far is that if we modify a parameter, the value doesn’t _reach_ the code.

## Updating parameters with parameter callbacks

After a parameter’s value has been set when the node starts, you can modify it from the terminal or with a service client. To be able to receive the new value in your code, however, you will need to add what is called a parameter callback.

In this section, you will learn how to implement a parameter callback for Python and C++. This callback will be triggered whenever a parameter’s value has been changed, and we will be able to get the new value in the code.

Note

You don’t necessarily need to add parameter callbacks in your nodes. For some parameters, you will want to have a fixed value when you start the node and not modify this value anymore. Use parameter callbacks only if it makes sense to modify some parameters during the execution of a node.

Parameter callbacks are a great way to change a setting in your node without having to create yet another service. Let me explain that with the camera driver example. If you want to be able to change the device name while the node is running, the default way would be services. You would create a service server in your node that accepts requests to change the device name. However, doing this for each small setting in your node can be a hassle. With parameters, not only can you provide a different device name at runtime, but you can also modify it later by using the parameter services that each ROS 2 node already has. There’s no need to make it more complicated than that.

Now, let’s see how to solve the issue we had when setting a new value for the **number** parameter, and let’s start with Python.

There are actually several parameter callbacks you could implement, but to keep things simple, I will just use one of them. Parameter callbacks are a nice and useful functionality, but it’s not necessarily the most important when you begin with ROS 2. Thus, here you will get an overview of the functionality, and feel free to do more research on your own after finishing the book (you will find additional resources in [_Chapter 14_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_14.xhtml#_idTextAnchor669)).

## Python parameter callback

Let’s write our first Python parameter callback.

Open the **number\_publisher.py** file and register a parameter callback in the node’s constructor:

```
self.add_post_set_parameters_callback(self.parameters_callback)
```

We also add a new import line:

```
from rclpy.parameter import Parameter
```

Then, we implement the callback method:

```
def parameters_callback(self, params: list[Parameter]):
&nbsp;&nbsp;&nbsp;&nbsp;for param in params:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;if param.name == "number":
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.number_ = param.value
```

In this callback, you receive a list of **Parameter** objects. For each parameter, you can access its name, value, and type. With a **for** loop, we go through each parameter we get and set the corresponding values in the code. You could also decide to validate the values (for example, only accept positive numbers), but I will not do that here to keep the code minimal.

To make a quick test, run the **number\_publisher** node again (no specified params; default values will be used). In another terminal, subscribe to the **/****number** topic:

```
$ ros2 topic echo /number
data: 2
---
```

Now, change the parameter’s value:

```
$ ros2 param set /number_publisher number 3
Set parameter successful
```

Let’s now go back to the other terminal to observe the change:

```
$ ros2 topic echo /number
data: 3
---
```

The parameter’s value has been changed, and we have received this value in the code, thanks to the parameter callback.

## C++ parameter callback

The behavior for parameter callbacks in C++ is exactly the same as for Python. Let’s have a look at the syntax. Open the **number\_publisher.cpp** file and register the parameter callback in the constructor:

```
param_callback_handle_ = this-&gt;add_post_set_parameters_callback(
&nbsp;&nbsp;&nbsp;&nbsp;std::bind(&amp;NumberPublisherNode::parametersCallback, this, _1));
```

Here is the implementation for the callback:

```
void parametersCallback(
&nbsp;&nbsp;&nbsp;&nbsp;const std::vector&lt;rclcpp::Parameter&gt; &amp; parameters)
{
```

```
&nbsp;&nbsp;&nbsp;&nbsp;for (const auto &amp;param: parameters) {
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;if (param.get_name() == "number") {
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;number_ = param.as_int();
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;}
&nbsp;&nbsp;&nbsp;&nbsp;}
}
```

We get a list of **rclcpp::Parameter** objects. From this, we can check each parameter’s name with the **get\_name()** method. If the parameter’s name matches, we get the value. Since we are receiving an integer here, we use the **as\_int()** method. For a string, you would use the **as\_string()** method, and so on. Please refer to the GitHub files for the complete code.

You have now seen the basics of parameter callbacks. You will not necessarily add them to all your nodes. They are great if you need to be able to modify a parameter’s value after the node has been started.

Let’s end this chapter with an additional challenge to make you practice more with parameters.

## Parameter challenge

With this challenge, you will practice everything you’ve seen in this chapter: declaring and getting parameters in your code, providing parameters’ values at runtime, and saving the values inside a YAML file. We will just skip parameter callbacks, but feel free to add them if you want to practice those too.

As usual for challenges, I will first explain what the challenge is and then provide the Python solution. You can find the complete code for both Python and C++ in the book’s GitHub repository.

## Challenge

We will continue to improve the **turtle\_controller** node. For this challenge, we want to be able to choose different settings at runtime:

-   Pen color on the right side
-   Pen color on the left side
-   Velocity to publish on the **cmd\_vel** topic

To do that, you will add these parameters:

-   **color\_1**: Instead of just arbitrarily choosing a color for when the turtle is on the right side, we rename the color as **color\_1**, and we get the value from a parameter. This parameter will be an integer list containing three values (**red**, **green**, **blue**).
-   **color\_2**: Same as for **color\_1**, this one is the color used when the turtle is on the left side of the screen.
-   **turtle\_velocity**: By default, we used **1.0** and **2.0** for velocities sent on the **cmd\_vel** topic. We make this a parameter so that we can provide the velocity at runtime. Instead of **1.0** and **2.0**, we will use **turtle\_velocity** and **turtle\_velocity \*** **2.0**.

To test this node, you will start the **turtle\_controller** node with **ros2 run** and provide different values for the parameters. You should see if it works by watching how fast the turtle is moving and what the colors are for the pen. If needed, add some logs in the code to see what’s happening.

As a last step for this challenge, you can put all the parameters inside a YAML file and load this YAML file at runtime.

## Solution

Let’s start by declaring the parameters that we will need for this challenge.

Open the **turtle\_controller.py** file. Let’s declare a few parameters at the beginning of the node’s constructor:

```
self.declare_parameter("color_1", [255, 0, 0])
self.declare_parameter("color_2", [0, 255, 0])
self.declare_parameter("turtle_velocity", 1.0)
```

We provide default values that correspond to the same values we previously hardcoded. Thus, if we start the node without providing any parameters, the behavior will be the same as before.

After declaring the parameters, we can get their values:

```
self.color_1_ = self.get_parameter("color_1").value
self.color_2_ = self.get_parameter("color_2").value
self.turtle_velocity_ = self.get_parameter("turtle_velocity").value
```

We store the values inside class attributes so that we can reuse them later in the code.

Note

As a reminder, in Python, don’t forget to add **.value** (without any parentheses) after **get\_parameter()**. This is a common error that will lead to an exception when you start the node.

Then, we modify a few lines in the **callback\_pose()** method:

```
if pose.x &lt; 5.5:
&nbsp;&nbsp;&nbsp;&nbsp;cmd.linear.x = self.turtle_velocity_
&nbsp;&nbsp;&nbsp;&nbsp;cmd.angular.z = self.turtle_velocity_
else:
&nbsp;&nbsp;&nbsp;&nbsp;cmd.linear.x = self.turtle_velocity_ * 2.0
&nbsp;&nbsp;&nbsp;&nbsp;cmd.angular.z = self.turtle_velocity_ * 2.0
self.cmd_vel_pub_.publish(cmd)
```

Instead of hardcoding the velocity value, we use the one we got from the parameter.

Then, we set the pen color:

```
if pose.x &gt; 5.5 and self.previous_x_ &lt;= 5.5:
&nbsp;&nbsp;&nbsp;&nbsp;self.previous_x_ = pose.x
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Set color 1.")
&nbsp;&nbsp;&nbsp;&nbsp;self.call_set_pen(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.color_1_[0],
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.color_1_[1],
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.color_1_[2]
&nbsp;&nbsp;&nbsp;&nbsp;)
elif pose.x &lt;= 5.5 and self.previous_x_ &gt; 5.5:
&nbsp;&nbsp;&nbsp;&nbsp;self.previous_x_ = pose.x
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Set color 2.")
&nbsp;&nbsp;&nbsp;&nbsp;self.call_set_pen(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.color_2_[0],
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.color_2_[1],
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.color_2_[2]
&nbsp;&nbsp;&nbsp;&nbsp;)
```

Here, we modify the logs so that they make more sense, as the color could be anything.

Finally, there are different ways you could pass the integer array to the **call\_set\_pen()** method. You could modify **call\_set\_pen()** so that it receives an array of three integers and extracts each number from it. Or, like I did here, you don’t modify the method and you just make sure to pass the correct arguments.

The code is now finished. To test it, start the **turtlesim** node in one terminal and the **turtle\_controller** node in another one. You can provide different values for the parameters. For example, if we want the velocity to be **1.5** and the colors to be black and white, we run the following:

```
$ ros2 run turtle_controller turtle_controller --ros-args -p color_1:=[0,0,0] -p color_2:=[255,255,255] -p turtle_velocity:=1.5
```

You can also save those parameters inside a YAML file. Create a new YAML file (for example, in your home directory) named **turtle\_params.yaml**. In this file, write this:

```
/turtle_controller:
&nbsp;&nbsp;ros__parameters:
&nbsp;&nbsp;&nbsp;&nbsp;color_1: [0, 0, 0]
&nbsp;&nbsp;&nbsp;&nbsp;color_2: [255, 255, 255]
&nbsp;&nbsp;&nbsp;&nbsp;turtle_velocity: 1.5
```

Then, you can start the turtle controller node with the YAML file directly:

```
$ ros2 run turtle_controller turtle_controller --ros-args --params-file ~/turtle_params.yaml
```

That’s it for this challenge. In the end, for each parameter, we did three things: we declared it, got its value, and used it in the code. This is not too complicated, and if you just know how to do that, you will be able to successfully handle parameters in your future ROS 2 applications.

## Summary

In this chapter, you worked on parameters. Parameters allow you to provide settings for your nodes at runtime. Thus, with the same code, you could start several different nodes with different configurations. This increases the code reusability a lot.

To handle parameters in your nodes, follow these guidelines:

1.  Declare the parameter so that it exists within the node. The best practice is to set a default value. This value will also set the type for the parameter.
2.  Get the parameter’s value and store it in your node—for example, in a private attribute.
3.  Use this value in your code.

Then, when you start a node with **ros2 run**, you can specify any parameter’s value you want.

You can also organize your parameters inside a YAML file, which makes it much more convenient when you start to have more than a handful of parameters. You will load the YAML file when you start a node.

Finally, you can also decide to allow parameters to be modified even after you’ve started a node. To do that, you will need to implement parameter callbacks.

Parameters make your nodes much more dynamic. In almost every node you run, you will have parameters. Using them makes it easier to scale your application by allowing different sets of configurations to be loaded.

Speaking of scaling, in the following chapter, we will dive into launch files. With launch files, you can start multiple nodes and parameters at once. This will be of great help when your application starts to grow.

<table id="table001-3"><colgroup></colgroup><colgroup><col></colgroup><colgroup><col></colgroup><tbody><tr><td><h4>Unlock this book’s exclusive benefits now</h4><p>This book comes with additional benefits designed to elevate your learning experience.</p></td><td rowspan="2"><p><img alt="" width="246" height="238" src="attachments/9781835881408.png"></p><p lang="en-US" xml:lang="en-US"><a href="https://www.packtpub.com/unlock/9781835881408" target="_blank" rel="noopener noreferrer">https://www.packtpub.com/unlock/9781835881408</a></p></td></tr><tr><td><p><em>Note: Have your purchase invoice ready before </em><span><em>you begin.</em></span></p></td></tr></tbody></table>
