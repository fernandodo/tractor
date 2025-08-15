---
created: 2025-08-14T12:58:23 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_06.xhtml
author: 
---

# Chapter 6: Services – Client/Server Interaction between Nodes | ROS 2 from Scratch

> ## Excerpt
> 6
Services – Client/Server Interaction between Nodes
 Nodes can communicate with each other using one of three communication types. You discovered topics in the previous chapter....

---
## Services – Client/Server Interaction between Nodes

Nodes can communicate with each other using one of three communication types. You discovered topics in the previous chapter. Now is the time to switch to the second most used communication: ROS 2 services.

As we did for topics, we will first understand services with the help of a real-life analogy. I will also share more thoughts on when to use topics versus services. After that, you will dive into the code and write a service server and client inside nodes using custom service interfaces. You will also explore additional tools to handle services from the Terminal.

All the code we’ll write in this chapter starts from the final code of the previous chapter. We will improve the number application to learn how to use services, and then work on the turtle controller application with an additional challenge. If you want to have the same starting point as me, you can download the code from GitHub ([https://github.com/PacktPublishing/ROS-2-from-Scratch](https://github.com/PacktPublishing/ROS-2-from-Scratch)), in the **ch5** folder, and use it as a starting point. The final code can be found in the **ch6** folder.

By the end of this chapter, you will understand how services work, and you will be able to create your own service interfaces, service servers, and service clients.

Becoming confident with topics and services is one of the most important things when starting with ROS 2. With this, you will be able to write custom code for your projects and interact with most of the existing ROS 2 applications.

In this chapter, we will cover the following topics:

-   What is a ROS 2 service?
-   Creating a custom service interface
-   Writing a service server
-   Writing a service client
-   Additional tools to handle services
-   Service challenge – client and server

## What is a ROS 2 service?

You discovered the concept of ROS 2 services in [_Chapter 3_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_03.xhtml#_idTextAnchor095), in the _Services_ section, where you ran your first service server and client to get an intuition of how they work. You also became familiar with the **ros2** command-line tool for handling services from the Terminal.

From here, I will start from scratch again and explain what services are, using a real-life analogy. We will build an example, step by step, and then recap the most important points.

## A server and a client

To start, I will use an online weather service as an analogy.

This online weather service can tell us the local weather after we send our location. To get the weather report for your city, you will need to interact with this service. You can use your computer to send a web request with the URL provided by the service.

What’s going to happen? First, your computer will send a request to the weather service. The request contains your location. The service will receive the request, process it, and if the location is valid, it will return the weather for that location. Your computer then receives a response containing the weather information. That’s the end of the communication. Here’s an illustration of this process:

![[attachments/B22403_06_1.jpg]]

Figure 6.1 – Client/server interaction

This is basically how a ROS 2 service works. On one side, you have a **service server** inside a node, and on the other side, you have a **service client** inside another node.

To start the communication, the service **Client** needs to send a **request** to the **Server**. The **Server** will then process the request, do any appropriate actions or computations, and return a **response** to the **Client**.

As you can see, a service, just like for topics, has a name and an interface. The interface is not just one message, it’s a pair of messages: a request and a response. Both the client and server must use the same name and interface to successfully communicate with each other.

With this example, the HTTP URL is the service name, and the pair (location, weather) is the service interface (request, response).

## Multiple clients for one service

In real life, many people will try to get the weather from this online service (at different times or at the same time). That’s not a problem: each client will send a request with a location to the server through the HTTP URL. The server will process each request individually and return the appropriate weather information to each client.

Now, this is very important: there can be only one server. One URL only goes to one server, just like one physical mail address is unique. Imagine if you send a package to someone and there are two places with the same address. How can the mail delivery person know where to deliver the package?

This will be the same for ROS 2 services. You can have several clients send a request to the same service. However, for one service, only one server can exist. See the following figure:

![[attachments/B22403_06_2.jpg]]

Figure 6.2 – Service server with multiple clients

Here, you can see some boxes, each box representing a node. Thus, we have four nodes. Three nodes contain a service **Client** and talk to the **Weather service** node, which contains a service **Server**.

One thing to note here is that the clients don’t know exactly which node to communicate with. They must go through the URL (service name). In this example, the clients aren’t aware of the IP address of the server—they just know they have to use the URL to connect to the server.

Also, no client is aware of the other clients. When you try to get the weather information from this service, you don’t know who is also trying to access the service, or even how many people are sending a request.

## Another service example with robotics

Let’s use another example that could be part of a ROS application.

Imagine that you have a node responsible for controlling an LED panel (three LEDs). This node could contain a service server that allows other nodes to request turning an LED on or off.

You also have a node monitoring a battery. In your application, what you want to do is turn on one LED when the battery is low, and then turn it off when the battery is high again.

You can do that using a ROS 2 service. The LED panel node would contain a service server named **set\_led**. To send a request to this server, you must provide the LED number and the state of that LED (on or off). Then, you receive a response containing a boolean value to see if the request was successfully processed by the server.

So, the battery is now running low. Here’s what’s going to happen:

![[attachments/B22403_06_3.jpg]]

Figure 6.3 – Client asking to turn on LED number 3

Here, **Battery node** will send a **Request** to the **set\_led** service. The **Request** contains the **LED number 3** and **state on** details so that it can turn on LED **3** of the panel.

The **Service** server, in the **LED panel node**, receives the **Request**. The server may decide to validate the **Request** (for example, if the LED number is 4, this is not valid) and process it. Processing the **Request** here means turning on the third LED. After that, the server sends a **Response** back to the **Client**, with a boolean flag. The **Client** receives this **Response**, and the communication ends.

Then, when the battery is fully charged, the **Battery node** sends another **Request** this time to turn off **LED 3**:

![[attachments/B22403_06_4.jpg]]

Figure 6.4 – Client asking to turn off LED number 3

The process is the same. The **Client** sends a **Request**, this time with **state off** for **LED 3**. The **Server**, inside the **LED panel node**, receives that **Request** and turns off the third LED. Then, the **Server** sends a **Response** back to the **Client**.

## Wrapping things up

On top of topics, ROS 2 nodes can use services to communicate with each other.

When should you use topics versus services? You should use topics to publish unidirectional data streams and services when you want to have a client/server type of communication.

For example, if you want to continuously send a velocity command to a robot 10 times per second, or send the data you read from a sensor, you will use topics. If you want to have a node perform quick computations or do some actions on demand (enabling/disabling a motor, starting/stopping a robot), then you would use services.

It can be hard to give a definitive answer to that question. Each application is different. Most of the time, the choice will be obvious, but sometimes, you have to go one way only to realize that that was the wrong way. The more experience you get with ROS 2, the more you will be able to make the best design decisions.

Here are some important points about how services work:

-   A service is defined by a name and an interface.
-   The name of a service follows the same rules as for topics. It must start with a letter and can be followed by other letters, numbers, underscores, tildes, and slashes.
-   The interface contains two things: a request and a response. Both the client and server must use the same interface to be able to communicate with each other.
-   A service server can only exist once but can have multiple clients.
-   Service clients are not aware of each other and are not aware of the server node. To reach the server, they just know that they must use the service name and provide the correct interface.
-   One node can contain multiple service servers and clients, each with a different service name.

Now, how can you write a service client and server?

Just as for nodes and topics, you will find everything you need in the **rclpy** and **rclcpp** libraries. With those libraries, you can write a service server and client inside nodes. That’s what we are going to do now.

As we can’t test a client without a server, let’s start with the server side. But before we even start writing the server, what interface will we need to use for the service?

## Creating a custom service interface

In [_Chapter 5_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_05.xhtml#_idTextAnchor214), when we created the ROS 2 application with the **number\_publisher** and **number\_counter** nodes, we used an existing interface for the **number** topic. Since we wanted to publish an integer number, the **example\_interfaces/msg/Int64** interface seemed to be exactly what we needed. At this point, you know that you must avoid using the **example\_interfaces** package for real applications, but for a first test, that wasn’t a problem.

We’re going to continue working on this application and add more functionalities so that we can practice with services. Here, we will focus on the **number\_counter** node. For now now, in this node, every time we receive a message from the **number** topic, we’ll add this number to a counter and print the counter.

What we want to do is allow the **number\_counter** node to reset the counter to a given number when we ask it to. For that, we will add a service server inside the node. Then, any other node can send a request, specifying the reset value for the counter. For example, let’s say the counter is currently at 76, and you send a request to reset it to 20. If the request is accepted by the service server, the counter will now become 20 and continue to increment from that value.

Great—we know what we must do. Now, which interface should we use? Can we find an existing interface for what we need, or will we have to create a custom one? As per the title of this section, you can already guess the answer to that question. Nonetheless, let’s see what we could find if we were looking at existing interfaces.

## Finding an existing interface for our service

When it comes to a service interface, we need to think about two things: the request and the response.

In our application, the request, which is sent from the client to the server, should contain an integer number. This is the reset value for the counter.

For the response, which is sent from the server to the client, we can decide to use a boolean flag, to specify whether we were able to perform the request, and a message to explain what went wrong if something went wrong.

The question is, will we find an existing interface that matches our needs? Unfortunately, this time, it seems that there is no matching interface. We can check the **example\_interfaces** package:

```
$ ros2 interface list | grep example_interfaces/srv
example_interfaces/srv/AddTwoInts
example_interfaces/srv/SetBool
example_interfaces/srv/Trigger
```

We can even check the **std\_srvs** package:

```
$ ros2 interface list | grep std_srvs/srv
std_srvs/srv/Empty
std_srvs/srv/SetBool
std_srvs/srv/Trigger
```

Note

As you can see, service interfaces are placed inside a **srv** folder, inside the package. For topics, we had a **msg** folder. This is a good way to differentiate both types of interfaces easily.

If you look more closely at those interfaces, especially **SetBool** and **Trigger**, you’ll see that there is no way to send an integer number in the request. Here’s an example where we’re trying to use **SetBool**:

```
$ ros2 interface show example_interfaces/srv/SetBool
# some comments
bool data # e.g. for hardware enabling / disabling
---
bool success&nbsp;&nbsp;&nbsp;# indicate successful run of triggered service
string message # informational, e.g. for error messages
```

When looking at the interface definition, you can see that the request and response are separated by three dashes (**\---**). In the response, we can find a boolean and a string, which is what we want. However, the request only contains a boolean, not an integer.

You could have a look at other interfaces in the common interfaces GitHub repository ([https://github.com/ros2/common\_interfaces](https://github.com/ros2/common_interfaces)) but you won’t find exactly what we are looking for.

Thus, we will create our own service interface before writing the code for the service. For the **number** topic, we were lucky enough to find an interface that we could directly use in the code (even though for real applications, the best practice is to avoid using **example\_interfaces** and **std\_srvs** anyway). Here, we need to create the interface first.

## Creating a new service interface

To create a service interface, just like for a topic interface, you need to create and configure a package dedicated to interfaces.

Good news: we did that in [_Chapter 5_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_05.xhtml#_idTextAnchor214) in the _Creating a custom interface for a topic_ section. Since we’re working on the same application, we will put all the topic and service interfaces in the same package: **my\_robot\_interfaces** (if you don’t already have this package, go back to the previous chapter and set it up).

We can create a new service interface inside that package directly; there’s nothing else to do. So, the process will be quite quick.

First, navigate inside the **my\_robot\_interfaces** package (where you already have a **msg** folder) and create a new **srv** folder:

```
$ cd ~/ros2_ws/src/my_robot_interfaces/
$ mkdir srv
```

In this new folder, you will put all the service interfaces that are specific to your application (or robot).

Now, create a new file for a service. Here are the rules to follow regarding the filename:

-   Use UpperCamelCase (PascalCase)—for example, **ActivateMotor**.
-   Don’t write **Srv** or **Interface** in the name as this would add unnecessary redundancy.
-   Use **.srv** for the file extension.
-   As a best practice, use a verb in the interface name—for example, **TriggerSomething**, **ActivateMotor**, or **ComputeDistance**. Services are about doing an action or computation, so by using a verb, you make it very clear what the service is doing.

Since we want to reset the counter, let’s simply call the interface **ResetCounter**:

```
$ cd ~/ros2_ws/src/my_robot_interfaces/srv/
$ touch ResetCounter.srv
```

Open this file and write the definition for the service interface. One very important thing to do here is add three dashes (**\---**) and put the request definition on top, and then the response definition below the dashes.

For the request and response, you can use the following:

-   Built-in types (**bool**, **byte**, **int64**, and so on).
-   Existing message interfaces. For example, the request of the service could contain **geometry\_msgs/Twist**.

Note

You can’t include a service definition inside another service definition. You can only include a message (topic definition) inside the request or the response of the service. The request and response can be seen as two independent messages.

Let’s write our service interface. As it’s not too complex, we can use simple built-in types:

```
int64 reset_value
---
bool success
string message
```

With this, the client will send a request with one integer value, and the response will contain a boolean flag as well as a string. All the fields inside the definition must follow the snake\_case convention (use underscores between words, all letters lowercase, and no space).

Note

Make sure you always have three dashes in all your service definitions, even if the request or the response is empty.

Now that we’ve written our interface, we need to build it so that we can use it in our code.

Go back to the **CMakeLists.txt** of **my\_robot\_interfaces** package. Since the package has already been configured, we just need to add one line. Add the relative path to the interface on a new line inside the **rosidl\_generate\_interfaces()** function. Don’t use any commas between the lines:

```
rosidl_generate_interfaces(${PROJECT_NAME}
&nbsp;&nbsp;"msg/HardwareStatus.msg"
<strong>&nbsp;&nbsp;"srv/ResetCounter.srv"</strong>
)
```

After this, save all files and build the **my\_robot\_interfaces** package:

```
$ colcon build --packages-select my_robot_interfaces
```

Once built, source the environment. You should be able to find your new interface:

```
$ ros2 interface show my_robot_interfaces/srv/ResetCounter
int64 reset_value
---
bool success
string message
```

If you see that, you know that the service interface has been built successfully, and you can use it in your code. So, let’s do that and write a service server.

## Writing a service server

You will now write your first service server. As mentioned previously, we will continue with the number application we started in the previous chapter. What we want to do here is allow **number\_counter** to reset the counter to a given number when we ask it to do so. This is a perfect example of when to use a service.

The first thing to think about when creating a new service is what service interface you need. We’ve just done that, so we can now focus on the code.

To write a service server, you will need to import the interface and then create a new service in the node’s constructor. You will also need to add a callback to be able to process the request, do the required action or computation, and return a response to the client.

As always, let’s start with a fully detailed explanation with Python, after which we will see how to do the same with C++.

## Writing a Python service server

To write a Python service server, we first need to have a Python node. We won’t create a new node here since we’re adding functionality to an existing one (**number\_counter**).

Note

You can have any number of publishers, subscribers, and services inside a node. So long as you keep things clean, that will not be a problem.

Let’s get started. As always, you can find the complete code in this book’s GitHub repository. I will not display the full code for the node here, just the new lines that are required for the service we are adding.

### Importing a service interface

The first big part of creating a service is to find an existing interface or create a new one. That’s what we just did, so let’s use the **ResetCounter** interface from the **my\_robot\_interfaces** package.

First, we need to add the dependency to this interface package inside the package where we write the node with the service. Open the **package.xml** file from **my\_py\_pkg** and add the new dependency:

```
&lt;depend&gt;rclpy&lt;/depend&gt;
&lt;depend&gt;example_interfaces&lt;/depend&gt;
<strong>&lt;depend&gt;my_robot_interfaces&lt;/depend&gt;</strong>
```

This will ensure that the interfaces package is installed when you build the **my\_py\_pkg** package with **colcon**. Now, import the dependency into your code (**number\_counter.py**):

```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
<strong>from my_robot_interfaces.srv import ResetCounter</strong>
```

To import a service, we must specify the package name (**my\_robot\_interfaces**), followed by the folder name for services (**srv**), and finally the class for the interface (**ResetCounter**).

Note

I’ve already mentioned this, but if you’re using VS Code and auto-completion doesn’t work, or the service isn’t recognized (import error), follow the process below.

Close VS code. Then, open a new Terminal, make sure the environment is correctly sourced, and find the interface (**ros2 interface show <interface\_name>**). After, navigate to the **src** directory of the ROS 2 workspace and open VS Code with:

**$** **code .**

### Adding a service server to the node

Now that you’ve correctly imported the service interface, you can create the service server.

As you did for publishers and subscribers, you will add your service servers to the node’s constructor.

Here’s the constructor of the **NumberCounterNode** class, which contains the previously created subscriber and the new service server:

```
def __init__(self):
&nbsp;&nbsp;&nbsp;&nbsp;super().__init__("number_counter")
&nbsp;&nbsp;&nbsp;&nbsp;self.counter_ = 0
&nbsp;&nbsp;&nbsp;&nbsp;self.number_subscriber_ = self.create_subscription(Int64, "number", self.callback_number, 10)
<strong>&nbsp;&nbsp;&nbsp;&nbsp;self.reset_counter_service_ = self.create_service(ResetCounter, "reset_counter", self.callback_reset_counter)</strong>
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Number Counter has been started.")
```

We add the service server at the same time as the number subscriber and just before the ending log.

To create the service server, we use the **create\_service()** method from the **Node** class. Once again, you can see that by inheriting from this class, we get access to all ROS 2 functionalities easily. In this method, you must provide three arguments:

-   **Service interface**: This is the **ResetCounter** class we have imported.
-   **Service name**: Whenever you create a service server, you’re creating the service itself, so you decide what its name will be. As best practice, start with a verb. Since we want to reset the counter, we’ll simply name it **reset\_counter**.
-   **Service callback**: The service server, as its name suggests, is a server. This means that it won’t do anything by itself. You will need to have a client send a request so that the server does something. So, while the node is spinning, the server will be in “waiting mode.” Upon reception of a request, the service callback will be triggered, and the request will be passed to this callback.

Now, we need to implement this callback. First, let’s write a minimal code example:

```
def callback_reset_counter(self, request: ResetCounter.Request, response: ResetCounter.Response):
&nbsp;&nbsp;&nbsp;&nbsp;self.counter_ = request.reset_value
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Reset counter to " + str(self.counter_))
&nbsp;&nbsp;&nbsp;&nbsp;response.success = True
&nbsp;&nbsp;&nbsp;&nbsp;response.message = "Success"
&nbsp;&nbsp;&nbsp;&nbsp;return response
```

In a service callback, we receive two things: an object for the request and an object for the response. The request object contains all the data sent by the client. The response object is empty, and we will need to fill it, as well as return it.

To name the callback, I usually write **callback\_** followed by the service name. This makes it easier to recognize in the code and will prevent future mistakes as you want to make sure you don’t call this method directly. It should only be called while the node is spinning and when a client sends a request from another node.

Note

In the method’s arguments, I have also specified the type for the two arguments. This way, we make the code more robust, and we can use auto-completion features from IDEs such as VS Code.

When you create an interface for a topic, you only get one class for that interface (for example, **Int64**). As you can see, in a service, we get two classes: one for the request (**Interface.Request**) and one for the response (**Interface.Response**).

In this callback, we get **reset\_value** from the request and modify the **counter\_** variable accordingly. Then, we fill the success and message fields from the response and return the response.

This is a very minimal piece of code for a service server. In real life, you’ll probably want to check if the request is valid before you use the values from it. For example, if you have a service that will modify the maximum velocity of a mobile robot, you might want to be sure the value you receive is not too high, to prevent the robot from becoming uncontrolled and damaging itself or the environment.

Let’s improve the callback so that we can validate **reset\_value** before we modify the **counter\_** variable.

### Validating the request

Let’s say we want to add those two validation rules: the reset value must be a positive number, and it cannot be higher than the current counter value.

Modify the code in the **callback\_reset\_counter** method, like so:

```
def callback_reset_counter(self, request: ResetCounter.Request, response: ResetCounter.Response):
&nbsp;&nbsp;&nbsp;&nbsp;if request.reset_value &lt; 0:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response.success = False
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response.message = "Cannot reset counter to a negative value"
&nbsp;&nbsp;&nbsp;&nbsp;elif request.reset_value &gt; self.counter_:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response.success = False
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response.message = "Reset value must be lower than current counter value"
&nbsp;&nbsp;&nbsp;&nbsp;else:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.counter_ = request.reset_value
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Reset counter to " + str(self.counter_))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response.success = True
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response.message = "Success"
&nbsp;&nbsp;&nbsp;&nbsp;return response
```

First, we check if the value is negative. If so, we don’t do anything with the **counter\_** variable. We set the boolean flag to **False** and provide an appropriate error message.

Then, we check if the value is greater than the current **counter\_** value. If that’s the case, we do the same thing as before, with a different error message.

Finally, if none of those conditions are true (which means we’ve validated the request), then we process the request and modify the **counter\_** variable.

Here’s a recap of the steps for a service server callback:

1.  (Optional but recommended) Validate the request, or validate that external conditions are met for the callback to be processed. For example, if the service is about activating a motor, but the communication with the motor hasn’t been started yet, then you can’t activate the motor.
2.  Process the action or computation using the data from the request if needed.
3.  Fill in the appropriate field for the response. It’s not mandatory to fill in all the fields. If you omit some of them, default values will be used (**0** for numbers and **""** for strings).
4.  Return the response. This is quite an important step that many people forget at the beginning. If you don’t return the response, you will get an error at runtime.

All you must do now is build your package where the node is, source, and run the node.

When you run the **number\_counter** node, you’ll see the following:

```
$ ros2 run my_py_pkg number_counter
[INFO] [1712647809.789229368] [number_counter]: Number Counter has been started.
```

The service server has been started within the node, but of course, nothing will happen as you need to send a request from a client to try the server.

That’s what we will do in a minute, but before that, let’s learn how to write the service server in C++. If you don’t want to learn ROS 2 with C++ for now, you can skip this and go to the next section in this chapter.

## Writing a C++ service server

Let’s add a service server inside our C++ **number\_counter** node using the same name and interface that we used for the one we created with Python. The process is the same: import the interface, create a service server, and add a callback function.

As mentioned previously in this book, make sure you follow all C++ explanations while keeping the GitHub code open on the side.

### Importing a service interface

First, since we’ll have a dependency on **my\_robot\_interfaces**, open the **package.xml** file of the **my****\_cpp\_pkg** package and add the following one line:

```
&lt;depend&gt;rclcpp&lt;/depend&gt;
&lt;depend&gt;example_interfaces&lt;/depend&gt;
<strong>&lt;depend&gt;my_robot_interfaces&lt;/depend&gt;</strong>
```

Then, open the **number\_counter.cpp** file and include the **ResetCounter** interface:

```
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
<strong>#include "my_robot_interfaces/srv/reset_counter.hpp"</strong>
```

To import a service interface in C++, you must use **#****include "<package\_name>/srv/<service\_name>.hpp"**.

Note

As a reminder, for this **include** to be recognized by VS Code, make sure you add the following to the **c\_cpp\_properties.json** file, in the **.vscode** folder that was generated when you opened VS Code: **`"/home/<user>/ros2\_ws/install/my\_robot\_interfaces/include/\*\*"`**.

After this, I added an extra line with the **using** keyword so that we can just write **ResetCounter** in the code, instead of **my\_robot\_interfaces::srv::ResetCounter**:

```
using ResetCounter = my_robot_interfaces::srv::ResetCounter;
```

This will help us make the code more readable. With C++, you can quickly end up with very long types that almost take more than one line to write. Since we will need to use the service interface quite often, adding this **using** line is a best practice to keep things simple.

I didn’t do it previously with **example\_interfaces::msg::Int64** when we worked on topics, but if you want, you can also write **using Int64 = example\_interfaces::msg::Int64;** and then reduce the code for the subscriber.

### Adding a service server to the node

Now that we’ve included the interface, let’s create the service server. We will store it as a private attribute in the class:

```
rclcpp::Service&lt;ResetCounter&gt;::SharedPtr reset_counter_service_;
```

As you can see, we use the **rclcpp::Service** class, and then, as always, we make it a shared pointer with **::SharedPtr**.

Now, we can initialize the service in the constructor:

```
reset_counter_service_ = this-&gt;create_service&lt;ResetCounter&gt;("reset_counter",&nbsp;&nbsp;std::bind(&amp;NumberCounterNode::callbackResetCounter, this, _1, _2));
```

To create the service, we must use the **create\_service()** method from the **rclcpp::Node** class. As for Python, we need to provide the service interface, the service name, and a callback to process the incoming requests. For **\_1** and **\_2** to work, don’t forget to add **using namespace** **std::placeholders;** beforehand.

Here’s the callback method, including the code to validate the request:

```
void callbackResetCounter(const ResetCounter::Request::SharedPtr request, const ResetCounter::Response::SharedPtr response)
{
&nbsp;&nbsp;&nbsp;&nbsp;if (request-&gt;reset_value &lt; 0) {
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response-&gt;success = false;
```

```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response-&gt;message = "Cannot reset counter to a negative value";
&nbsp;&nbsp;&nbsp;&nbsp;}
&nbsp;&nbsp;&nbsp;&nbsp;else if (request-&gt;reset_value &gt; counter_) {
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response-&gt;success = false;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response-&gt;message = "Reset value must be lower than current counter value";
&nbsp;&nbsp;&nbsp;&nbsp;}
&nbsp;&nbsp;&nbsp;&nbsp;else {
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;counter_ = request-&gt;reset_value;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;RCLCPP_INFO(this-&gt;get_logger(), "Reset counter to %d", counter_);
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response-&gt;success = true;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response-&gt;message = "Success";
&nbsp;&nbsp;&nbsp;&nbsp;}
}
```

In the callback, we receive two arguments—the request and the response. Both are **const** shared pointers.

What we do in this callback is the same as for Python. The biggest difference here is that we don’t return anything (in Python, we had to return the response) as the return type for the callback is **void**.

Now, we can build the package to compile and install the node. However, before we run **colcon build**, we have to modify the **CMakeLists.txt** file of the **my\_cpp\_pkg** package. Since we have a new dependency on **my\_robot\_interfaces**, we need to link the **number\_counter** executable with that dependency.

First, add a line under all the **find\_package()** lines:

```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
<strong>find_package(my_robot_interfaces REQUIRED)</strong>
```

Then, add **my\_robot\_interfaces** to the **ament\_target\_dependencies()** function, for the **number\_counter** executable:

```
add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces <strong>my_robot_interfaces</strong>)
```

For every new dependency you’re using in this executable, you will have to link to it before you build.

If you forget about this, then you will get this kind of error when you run **colcon build**:

```
fatal error: my_robot_interfaces/srv/reset_counter.hpp: No such file or directory
Failed&nbsp;&nbsp;&nbsp;&lt;&lt;&lt; my_cpp_pkg [1.49s, exited with code 2]
```

Now you can build the C++ package, source, and run the **number\_counter** node.

```
$ ros2 run my_cpp_pkg number_counter
[INFO] [1712726520.316615636] [number_counter]: Number Counter has been started.
```

We are now at the same point as when we finished the Python service server. The next step is to try the service server. To do that, we need a service client.

## Writing a service client

For service communication to work, you need a service server and a service client. As a reminder, you can only have one service server but multiple clients.

So far, we’ve finished our service server inside the **number\_counter** node. Now, let’s create a service client inside another node so that you can try the service.

Where will you write the code for the client? In a real application, you will create a service client in a node that needs to call the service. In terms of the battery and LED example from the beginning of this chapter, the LED panel node contains a service server. The battery node, which is responsible for monitoring the battery state, contains a service client that can send some requests to the server.

Then, when to send a request depends on the application. With the previous example, we decided that when the battery gets full or empty, we use the service client inside the node to send a request to the server so that we can turn an LED on/off.

To keep things simple for now, we will create a new node named **reset\_counter\_client**. This node will only do one thing: send a request to the service server and get the response. With this, we will be able to focus only on writing the service client. As usual, we’ll start with Python and then see the C++ code as well.

## Writing a Python service client

Create a new file, named **reset\_counter\_client.py**, inside the **my\_py\_pkg** package. Make this file executable. The file should be placed with all the other Python files you created previously.

Open the file and start by importing the interface:

```
from my_robot_interfaces.srv import ResetCounter
```

In the node’s constructor, create a service client:

```
def __init__(self):
&nbsp;&nbsp;&nbsp;&nbsp;super().__init__("reset_counter_client")
&nbsp;&nbsp;&nbsp;&nbsp;self.client_ = self.create_client(ResetCounter, "reset_counter")
```

To create a service client, we use the **create\_client()** method from the **Node** class. We need to provide the service interface and service name. Make sure you use the same name and interface you defined in the server.

Then, to call the service, we create a new method:

```
def call_reset_counter(self, value):
&nbsp;&nbsp;&nbsp;&nbsp;while not self.client_.wait_for_service(1.0):
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().warn("Waiting for service...")
&nbsp;&nbsp;&nbsp;&nbsp;request = ResetCounter.Request()
&nbsp;&nbsp;&nbsp;&nbsp;request.reset_value = value
&nbsp;&nbsp;&nbsp;&nbsp;future = self.client_.call_async(request)
&nbsp;&nbsp;&nbsp;&nbsp;future.add_done_callback(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.callback_reset_counter_response)
```

Here are the steps to make a service call:

1.  Make sure the service is up and running with **wait\_for\_service()**. This function will return **True** as soon as the service has been found, or return **False** after the provided timeout, which is **1.0** seconds here.
2.  Create a request object from the service interface.
3.  Fill in the request fields.
4.  Send the request with **call\_async()**. This will give you a Python **Future** object.
5.  Register a callback for when the node receives the response from the server.

To process the response from the service, add a callback method:

```
def callback_reset_counter_response(self, future):
&nbsp;&nbsp;&nbsp;&nbsp;response = future.result()
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Success flag: " + str(response.success))
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Message: " + str(response.message))
```

In the callback, we get the response with **future.result()**, and we can access each field of the response. In this example, we simply print the response with a log.

So, what’s going to happen? After you send the request with **call\_async()**, the server will receive and process the request. Upon completion of the task, the server will return a response to the node where the client is. When the client node receives the response, it will process it in the callback that you’ve written.

Note

You might be wondering, why do we need a callback? Why can’t we just wait for the response in the same method where we send the request? That’s because if you block this method (or in other words, this thread), then the node won’t be able to spin. If the spin is blocked, then any response you get for this node won’t be processed, and you have what is called a deadlock.

The only thing left to do is call the **call\_reset\_counter()** method. If we don’t call it, nothing will happen. In a real application, you would call this method whenever you need it (it could be from a timer callback, a subscriber callback, and so on). Here, to make a test, we just call the method after creating the node, and before spinning, in the **main()** function:

```
node = ResetCounterClientNode()
node.call_reset_counter(20)
rclpy.spin(node)
```

The service client will send a request and register a callback for the response. After that, the **call\_reset\_counter()** method exits, and the node starts to spin.

That’s it for the code. You can use this structure for the client (one method to send the request and one callback to process the response) in any other node.

Now, let’s test the client/server communication.

## Running the client and server nodes together

Create an executable in the **setup.py** file named **reset\_counter\_client**, for example.

Then, build the workspace and open three Terminals. In Terminals 1 and 2, start **number\_publisher** and **number\_counter**. The latter will start the **reset\_counter** service server.

In Terminal 3, start the **reset\_counter\_client** node. Since we want to reset the counter to 20, if the counter inside the **number\_counter** node is less than 20 at the moment of sending the request, you will get the following response:

```
$ ros2 run my_py_pkg reset_counter_client
[INFO] [1713082991.940407360] [reset_counter_client]: Success flag: False
[INFO] [1713082991.940899261] [reset_counter_client]: Message: Reset value must be lower than current counter value
```

If the counter is 20 or more, you will get the following response instead:

```
$ ros2 run my_py_pkg reset_counter_client
[INFO] [1713082968.101789868] [reset_counter_client]: Success flag: True
[INFO] [1713082968.102277613] [reset_counter_client]: Message: Success
```

Also, just after you start the node, the client sometimes needs a bit of time to find the service. In this case, you might see this log as well:

```
[WARN] [1713082991.437932627] [reset_counter_client]: Waiting for service...
```

On the server side (the **number\_counter** node), if the counter is being reset, you will see this log:

```
[INFO] [1713083108.125753986] [number_counter]: Reset counter to 20
```

With that, we have tested two cases: when the counter is less than the requested reset value and when the counter is more than the requested reset value. If you want, you can also test the third case: when the requested reset value is lower than 0.

Now that we’ve finalized the client/server communication between the two nodes, let’s switch to C++.

## Writing a C++ service client

The C++ code follows the same logic as the Python code.

First, we include the interface:

```
#include "my_robot_interfaces/srv/reset_counter.hpp"
```

Then, we add a few **using** lines to reduce the code later:

```
using ResetCounter = my_robot_interfaces::srv::ResetCounter;
using namespace std::chrono_literals;
using namespace std::placeholders;
```

Next, we declare the service client as a private attribute in the class:

```
rclcpp::Client&lt;ResetCounter&gt;::SharedPtr client_;
```

After, we initialize the client in the constructor:

```
ResetCounterClientNode() : Node("reset_counter_client")
{
&nbsp;&nbsp;&nbsp;&nbsp;client_ = this-&gt;create_client&lt;ResetCounter&gt;("reset_counter");
}
```

Then, as we did for Python, we add a method to call the service:

```
void callResetCounter(int value)
{
&nbsp;&nbsp;&nbsp;&nbsp;while (!client_-&gt;wait_for_service(1s)) {
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;RCLCPP_WARN(this-&gt;get_logger(), "Waiting for the server...");
&nbsp;&nbsp;&nbsp;&nbsp;}
```

```
&nbsp;&nbsp;&nbsp;&nbsp;auto request = std::make_shared&lt;ResetCounter::Request&gt;();
&nbsp;&nbsp;&nbsp;&nbsp;request-&gt;reset_value = value;
&nbsp;&nbsp;&nbsp;&nbsp;client_-&gt;async_send_request(request, std::bind(&amp;ResetCounterClientNode::callbackResetCounterResponse, this, _1));
}
```

In this method, we wait for the service (don’t forget the exclamation mark in front of **client->wait\_for\_service(1s)**), create a request, fill in the request, and send it with **async\_send\_request()**. We pass the callback method as an argument, which will register the callback when the node is spinning.

Here’s the callback method for the response:

```
void callbackResetCounterResponse(
&nbsp;&nbsp;&nbsp;&nbsp;rclcpp::Client&lt;ResetCounter&gt;::SharedFuture future)
{
&nbsp;&nbsp;&nbsp;&nbsp;auto response = future.get();
&nbsp;&nbsp;&nbsp;&nbsp;RCLCPP_INFO(this-&gt;get_logger(), "Success flag: %d, Message: %s", (int)response-&gt;success, response-&gt;message.c_str());
}
```

Finally, to be able to send a request, we call the **callResetCounter()** method just after creating the node, and before spinning:

```
auto node = std::make_shared&lt;ResetCounterClientNode&gt;();
node-&gt;callResetCounter(20);
rclcpp::spin(node);
```

Now, create a new executable in **CMakeLists.txt**. Build the package, open a few Terminals, and start the **number\_publisher** and **number\_counter** nodes. Then, start the **reset\_counter\_client** node to try the service communication.

Now that you’ve written the code for both the service server and client, let’s explore what you can do with the ROS 2 tools. For services with a simple interface, you will be able to test them directly from the Terminal, even before writing the code for a client.

## Additional tools to handle services

We’ve already used the **ros2** command-line tool a lot in this book. With this tool, each core ROS 2 concept gets additional functionalities in the Terminal. This is no exception for services.

We’re now going to explore **ros2 service** a bit more so that we can introspect services and send a request from the Terminal. We will also learn how to change a service name at runtime (**ros2 run**).

To see all commands for ROS 2 services, type **ros2** **service -h**.

## Listing and introspecting services

First, **rqt\_graph** does not support services (yet—there are plans to maybe add this in a future ROS 2 distribution), so we won’t use it here. We will only use the **ros2** command-line tool.

Stop all nodes and start the **number\_counter** node. Then, to list all services, run the following command:

```
$ ros2 service list
/number_counter/describe_parameters
/number_counter/get_parameter_types
/number_counter/get_parameters
/number_publisher/get_type_description
/number_counter/list_parameters
/number_counter/set_parameters
/number_counter/set_parameters_atomically
/reset_counter
```

For each node you start, you will get seven additional services, mostly related to parameters. You can ignore those. If you look at the list, apart from those seven services, we can retrieve our **/****reset\_counter** service.

Note

Note that there is an additional leading slash in front of the service name. Service names follow the same rules as nodes and topics. If you don’t provide any namespace (for example, **/abc/reset\_counter**), you’re in the “global” namespace, and a slash is added at the beginning.

Once you have the service name you want, you can get the service interface with **ros2 service** **type <service\_name>**:

```
$ ros2 service type /reset_counter
my_robot_interfaces/srv/ResetCounter
```

From this, you can see the details inside the interface:

```
$ ros2 interface show my_robot_interfaces/srv/ResetCounter
int64 reset_value
---
bool success
string message
```

This process is extremely useful when you need to create a service client for an existing server. There’s no need to even read the server code—you can get all the information you need from the Terminal.

## Sending a service request

To test a service server, you usually have to write a service client.

Good news: instead of writing a client, you can call the service from the Terminal directly. This can save you some development time.

First, you must know the service name and interface. Then, use the **ros2 call <service\_name> <interface\_name> "<request\_in\_json>"** command. Let’s try this with our **reset\_counter** service:

```
$ ros2 service call /reset_counter \ my_robot_interfaces/srv/ResetCounter {reset_value: 7}"
waiting for service to become available...
requester: making request: my_robot_interfaces.srv.ResetCounter_Request(reset_value=7)
response:
my_robot_interfaces.srv.ResetCounter_Response(success=True, message='Success')
```

You can see the request being sent, followed by the response. Then, the command exits. This is practical and in this case, we save a lot of time.

We can also easily test the different cases. For example, let’s send a negative value for the reset number:

```
$ ros2 service call /reset_counter \my_robot_interfaces/srv/ResetCounter "{reset_value: -7}"
waiting for service to become available...
requester: making request: my_robot_interfaces.srv.ResetCounter_Request(reset_value=-7)
response:
my_robot_interfaces.srv.ResetCounter_Response(success=False, message='Cannot reset counter to a negative value')
```

With this example, it’s quite easy as the request is very simple (only one integer number). For more complex service requests that contain lots of nested fields and arrays, writing the full request in the Terminal can become quite cumbersome, and you will spend a lot of time trying to get it right.

So, for simple interfaces, use **ros2 service call** to try the service first. For more complex interfaces, you’ll have to write a client code first. This isn’t really a problem: you can use the code we used for **ResetCounterClientNode** as a template for any other client. In the end, both methods allow you to test a service server quite quickly.

## Changing a service name at runtime

When you start a node with **ros2 run**, you can change the node name and any topic name inside the node. You can do the same for services.

As a reminder, for any additional argument you pass after **ros2 run**, add **\--ros-args**, but only once.

Then, to rename a service, add **\-r** followed by **<service\_name>:=<new\_service\_name>**.

For example, let’s rename the **reset\_counter** service to **reset\_counter1** when we start the **number\_counter** node:

```
$ ros2 run my_py_pkg number_counter --ros-args -r \ reset_counter:=reset_counter1
```

Now, let’s verify this with **ros2** **service list**:

```
$ ros2 service list
# Some other services
/reset_counter1
```

The service name is now **/reset\_counter1**. If we start a node with a service client, we need to modify the name as well; otherwise, the nodes won’t be able to communicate with each other:

```
$ ros2 run my_py_pkg reset_counter_client --ros-args -r \ reset_counter:=reset_counter1
```

Doing this is quite useful, especially when you want to run several nodes (written by yourself or others) that use a slightly different service name, or that are in different namespaces.

You are now able to write a service server/client and introspect/test them from the Terminal. Before moving on to the next chapter, let’s practice more with an additional challenge.

## Service challenge – client and server

With this new challenge, you will practice everything that was covered in this chapter: custom service interfaces, service servers, and service clients.

We will use the **turtle\_controller** node we wrote in the previous chapter’s challenge as a starting point. We won’t create a new node here; instead, we will improve the existing code. You can either start from the code you wrote or from the code I provided in the **ch5** folder of this book’s GitHub repository.

As always, I will explain what you need to do to complete the challenge, and then detail the most important points for the Python solution. You can find the complete solution code on GitHub for both Python and C++.

## Challenge

This challenge is divided into two parts. I suggest following them in order.

### Challenge 1 – service client

So far, our **turtle\_controller** node is subscribing to the **/turtle1/pose** topic. In the subscriber callback, we send a velocity command to the **/****turtle1/cmd\_vel** topic.

The result of this is the turtle drawing a circle on the screen, with a different velocity depending on if it is on the right or left of the screen.

What we want to do now is change the color of the pen, depending on where the turtle is. If the turtle is on the right of the screen, we want the pen color to be red. On the left, the color should be green.

To do that, we will need to add a service client in the node so that we can call the service to change the pen’s color in the **turtlesim** node (I won’t give you the service name—that’s part of the challenge).

Here are the steps you can take to get started:

1.  Start the **turtlesim** node and use the **ros2 service** command line to find which service to call, as well as what interface to import (optional: at that point, you can also test the service with **ros2 service call**, directly from the Terminal).
2.  In the **turtle\_controller** node, add a service client for that service.
3.  Create a method that will call the service.
4.  Call this method from the existing subscriber callback. After you publish the new command velocity, check whether the turtle is on the right or left of the screen. When the turtle switches to a different side, call the service with the updated color.

### Challenge 2 – custom interface and service server

Once you’re done with the first challenge, try this one. This time, you’ll practice on the server side of services.

Here, we want to allow the **turtle\_controller** node to activate or deactivate the turtle (meaning to start or stop the turtle), depending on an external request. For that, we will create a service server.

Here are the steps you can take to get started:

1.  Define a service name and interface for that service.
2.  If no existing interface matches your needs, you will need to create and build a new one (hint: that’s what we will do here).
3.  In the **turtle\_controller** node, add a service server and a callback, in which you activate or deactivate the turtle. Tip: you can use a simple boolean attribute in the class to store the activated state for the turtle.
4.  If the turtle is activated, then in the subscriber callback, you can keep sending additional velocity commands. If it is not activated, you don’t send any commands.

With those instructions, you should be able to get started. Taking the time to do this exercise is probably the best investment you can make to learn ROS faster.

## Solution

Let’s start with the first challenge.

### Challenge 1

For this challenge, we are on the client side, which means that we need to find out which service we need to call. I will do a quick recap of the steps for finding the service name and interface.

Start the **turtlesim** node and list all services. You should see a **/turtle1/set\_pen** service with **ros2** **service list**.

Now, get the type for this service and see what’s inside the interface:

```
$ ros2 service type /turtle1/set_pen
turtlesim/srv/SetPen
$ ros2 interface show turtlesim/srv/SetPen
uint8 r
uint8 g
uint8 b
uint8 width
uint8 off
---
```

In the service request, we can send an (**r**,**g**,**b**) value (red, green, blue). There are also **width** and **off** attributes, but we won’t use them.

At this point, before you even write the code for the client, you can try the service from the Terminal:

```
$ ros2 service call /turtle1/set_pen turtlesim/srv/SetPen \ "{r: 255, g: 0, b: 0}"
```

Then, execute **ros2 run turtlesim turtle\_teleop\_key** and move the turtle around. You’ll see that the pen is now using a red color.

Back to the code, inside the **turtle\_controller.py** file, import the interface:

```
from turtlesim.srv import SetPen
```

Since we’ve already added the dependency for **turtlesim** in the **package.xml** file of the **turtle\_controller** package (in the previous chapter), there’s no need to do it again.

Then, create the service client in the constructor:

```
self.set_pen_client_ = self.create_client(SetPen, "/turtle1/set_pen")
```

Write the method that will call the service, as well as the callback for the response:

```
def call_set_pen(self, r, g, b):
&nbsp;&nbsp;&nbsp;&nbsp;while not self.set_pen_client_.wait_for_service(1.0):
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().warn("Waiting for service...")
&nbsp;&nbsp;&nbsp;&nbsp;request = SetPen.Request()
&nbsp;&nbsp;&nbsp;&nbsp;request.r = r
&nbsp;&nbsp;&nbsp;&nbsp;request.g = g
&nbsp;&nbsp;&nbsp;&nbsp;request.b = b
&nbsp;&nbsp;&nbsp;&nbsp;future = self.set_pen_client_.call_async(request)
&nbsp;&nbsp;&nbsp;&nbsp;future.add_done_callback(
self.callback_set_pen_response)
def callback_set_pen_response(self, future):
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Successfully changed pen color")
```

We only send the **r**, **g**, and **b** parts of the request. The other values (**width** and **off**) will be kept as-is.

As you can see, in the callback for the response, we don’t check what’s inside the response since the response is empty (it exists but it doesn’t contain a field).

The only thing we need to do now is call this new **call\_set\_pen()** method. We will do that from within the subscriber callback since this is where we have access to the _X_ position of the turtle.

Inside the **callback\_pose()** method, and after the code to publish on the topic, add the code to handle the pen color:

```
if pose.x &gt; 5.5 and self.previous_x_ &lt;= 5.5:
&nbsp;&nbsp;&nbsp;&nbsp;self.previous_x_ = pose.x
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Set color to red.")
&nbsp;&nbsp;&nbsp;&nbsp;self.call_set_pen(255, 0, 0)
elif pose.x &lt;= 5.5 and self.previous_x_ &gt; 5.5:
&nbsp;&nbsp;&nbsp;&nbsp;self.previous_x_ = pose.x
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Set color to green.")
&nbsp;&nbsp;&nbsp;&nbsp;self.call_set_pen(0, 255, 0)
```

If the turtle is on the right, we set the color to red (**255, 0, 0**), and if it’s on the left, we set the color to green (**0,** **255, 0**).

On top of that, we also define a new attribute in the constructor so that we can keep track of the previous _X_ coordinate:

```
self.previous_x_ = 0.0
```

We use this to only call the service when the turtle switches from one side to the other. Why do we do that? We could send a service request every time, even if the color would be the same as the previous one. Why “optimize” the code?

The reason is that the **callback\_pose()** method will be called a lot. Check the frequency for the **/turtle1/pose** topic in the Terminal:

```
$ ros2 topic hz /turtle1/pose
average rate: 62.515
```

This means that we execute **callback\_pose()** about 62 times per second. This is not really a problem. We also publish on the **/turtle1/cmd\_vel** topic at 62 Hz. Again, that’s not a problem. Publishers and subscribers can sustain a high frequency (with a bigger message size, this could become complicated, but here, the messages are really small).

Now, what if we send a request to a service 62 times per second? This is where the problem is. Services are not made for high-frequency requests, and this could seriously affect the performance of the application. Also, if you find yourself having to call a service at 62 Hz, then you probably have a design problem, and you either need to modify your code to reduce the frequency or use a publish/subscribe mechanism instead.

So, what we do in the code is make sure we only call the service when it’s needed—that is, when the turtle switches from one side to the other.

The code is now complete! At this point, you can build your **turtle\_controller** package again (unless you have already built it with **\--symlink-install**), source the environment, and then start both the **turtlesim** and **turtle\_controller** nodes to see the result.

### Challenge 2

Now, we want to add a service server inside our node so that we can activate or deactivate the turtle. Since we’re defining the server, we need to come up with a name and an interface:

-   **Name**: Let’s use **activate\_turtle**. We’ll start with a verb and try to make the name as explicit as possible.
-   **Interface**: If you look at existing interfaces, we could use the **SetBool** service from **example\_interfaces**. It contains a boolean in the request and a string in the response. However, as stated previously, the best practice is to avoid using the **std\_srvs** and **example\_interfaces** packages if your application is any serious. So, in this case, we’ll create our own interface.

Let’s create a new interface for our service. This will be quite quick and easy as we already have the **my\_robot\_interfaces** package fully configured.

Inside the **srv** folder of the **my\_robot\_interfaces** package, create a new service file named **ActivateTurtle.srv**. In this file, write the service definition:

```
bool activate
---
string message
```

This is all we need in the request: a boolean to activate or deactivate the turtle. We also added a string in the response so that we get to know what happened, but you could also decide to have an empty response.

After this, add the interface to the **CMakeLists.txt** file of the **my\_robot\_interfaces** package, and build the package. Source the environment, and make sure you can see the interface with:

```
ros2 interface show my_robot_interfaces/srv/ActivateTurtle
```

Now, let’s go back to the **turtle\_controller** package.

Since we will have a dependency on **my\_robot\_interfaces**, add a new line to the **package.xml** file of the **turtle\_controller** package:

```
&lt;depend&gt;my_robot_interfaces&lt;/depend&gt;
```

Now, it’s time to write the code inside **turtle\_controller.py**. Import the interface:

```
from my_robot_interfaces.srv import ActivateTurtle
```

In the constructor, add a boolean flag to keep track of the activated status for the turtle, and create a new service server:

```
self.is_active_ = True
self.activate_turtle_service_ = self.create_service(ActivateTurtle, "activate_turtle", self.callback_activate_turtle)
```

Implement the callback method for that service:

```
def callback_activate_turtle(self, request: ActivateTurtle.Request, response: ActivateTurtle.Response):
&nbsp;&nbsp;&nbsp;&nbsp;self.is_active_ = request.activate
&nbsp;&nbsp;&nbsp;&nbsp;if request.activate:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response.message = "Starting the turtle"
&nbsp;&nbsp;&nbsp;&nbsp;else:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;response.message = "Stopping the turtle"
&nbsp;&nbsp;&nbsp;&nbsp;return response
```

What we do is simple—we just set the **is\_active\_** boolean with the value we get from the boolean in the request. Now, whenever you call this service, the **is\_active\_** boolean will be updated with the value you send.

The last step, to make the turtle start or stop when activated or deactivated, is to modify the code inside the **callback\_pose()** method:

```
def callback_pose(self, pose: Pose):
&nbsp;&nbsp;&nbsp;&nbsp;if self.is_active_:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;# Entire code for the callback, inside the "if"
```

This way, we only publish a new command velocity if the turtle is activated. If not, we publish nothing. Also, the service request will only work when the turtle is activated.

To try this new service, start the **turtlesim** node and **turtle\_controller** nodes. In a third Terminal, send a service request with the **ros2** command-line tool. Here’s an example:

```
$ ros2 service call /activate_turtle \
my_robot_interfaces/srv/ActivateTurtle "{activate: false}"
```

This should make the turtle stop. You can send a request again, this time with **"{activate: true}"**, which should make the turtle move again.

That’s the end of this challenge on services. If you managed to finish this challenge by yourself, you have a good understanding of services. No worries if you couldn’t do it without having to look at the solution. Come back to it in a few days and see if you can solve the challenge again.

## Summary

In this chapter, you worked on ROS 2 services, which is another ROS 2 communication you can use alongside topics.

With services, nodes can talk to each other using a client/server type of communication. Only one server can exist for a service, but you can send multiple requests from several clients.

You can implement service servers and clients directly in your nodes using **rclpy** for Python and **rclcpp** for C++.

To write a service server, you must do the following:

1.  As the name and interface are defined by the server, you have to choose them here. As a best practice, use a verb as the first word in the name.
2.  Import the interface in your code and create the service server in the constructor.
3.  Add a callback method to process any received request.

When choosing a service interface, if you can’t find an existing one that perfectly matches what you need, then you have to create and build your own. To do that, you must do the following:

1.  Create and set up a package dedicated to interfaces. If you already have one for your application, use it.
2.  Add the new service interface to the package and build it.
3.  Now, you can use this interface in your service server.

To write a service client, do the following:

1.  If you’re writing a client, it means that there is an existing server on the other side. Find which name and interface you need to use.
2.  Import the interface into your code and create the service client in the constructor.
3.  Create a method to call the service. In this method, you send the request asynchronously, and then register a callback to process the response.
4.  You can call the service from anywhere in your code.

With the **ros2 service** command line, you can introspect the services in your nodes and see what interface they’re using.

To try a service server, you can either write a corresponding service client inside another node or, if the request is simple, call the service directly from the Terminal with **ros2** **service call**.

You have now seen the two most common ROS 2 communication types: topics and services. In the next chapter, we will work with the third and last one: actions.

<table id="table001-1"><colgroup></colgroup><colgroup><col></colgroup><colgroup><col></colgroup><tbody><tr><td><h4>Unlock this book’s exclusive benefits now</h4><p>This book comes with additional benefits designed to elevate your learning experience.</p></td><td rowspan="2"><p><img alt="" width="246" height="238" src="attachments/9781835881408.png"></p><p lang="en-US" xml:lang="en-US"><a href="https://www.packtpub.com/unlock/9781835881408" target="_blank" rel="noopener noreferrer">https://www.packtpub.com/unlock/9781835881408</a></p></td></tr><tr><td><p><em>Note: Have your purchase invoice ready before </em><span><em>you begin.</em></span></p></td></tr></tbody></table>
