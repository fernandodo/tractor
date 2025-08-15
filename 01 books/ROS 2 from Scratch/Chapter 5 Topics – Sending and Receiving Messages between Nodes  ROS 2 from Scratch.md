---
created: 2025-08-14T12:57:38 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_05.xhtml#_idParaDest-116
author: 
---

# Chapter 5: Topics – Sending and Receiving Messages between Nodes | ROS 2 from Scratch

> ## Excerpt
> 5
Topics – Sending and Receiving Messages between Nodes
 Now that you can write nodes, how can you make several nodes communicate with each other, and how can you interact with...

---
## Topics – Sending and Receiving Messages between Nodes

Now that you can write nodes, how can you make several nodes communicate with each other, and how can you interact with existing nodes in an application?

There are three kinds of communication in ROS 2: topics, services, and actions. In this chapter, we will dive into ROS 2 topics.

To understand how topics work, we will start with a real-life analogy. This will allow you to grasp the concept using existing and common knowledge. Then, you will dive into the code and write a publisher and a subscriber inside a node—first with existing interfaces, and then by building custom interfaces. You will also use ROS 2 tools such as the **ros2** command line and **rqt\_graph** to introspect topics and unlock more functionalities.

By the end of this chapter, you will be able to make your nodes communicate with each other using ROS 2 topics. You will learn by writing code and will be provided with an additional challenge at the end of this chapter.

Topics are used everywhere in ROS 2. Whether you wish to create an application from scratch or use existing ROS plugins, you will have to use topics.

We will use the code inside the **ch4** folder in this book’s GitHub repository ([https://github.com/PacktPublishing/ROS-2-from-Scratch](https://github.com/PacktPublishing/ROS-2-from-Scratch)) as a starting point. You can find the final code in the **ch5** folder.

In this chapter, we will cover the following topics:

-   What is a ROS 2 topic?
-   Writing a topic publisher
-   Writing a topic subscriber
-   Additional tools to handle topics
-   Creating a custom interface for a topic
-   Topic challenge – closed-loop control

## What is a ROS 2 topic?

You discovered the concept of topics through hands-on experiments in [_Chapter 3_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_03.xhtml#_idTextAnchor095). With this, you should have a basic intuition of how things work.

I am now going to start from scratch again and explain topics—not by running code, but by using a real-life analogy that makes it easier to understand. We will build an example, step by step, and then recap the most important points.

## A publisher and a subscriber

For this analogy, I will use radio transmitters and receivers. As this is a simplified example, not everything I’ll say about radio will be correct, but the point here is to understand ROS 2 topics.

Let’s start with one radio transmitter. This radio transmitter will send some data at a given frequency. To make it easier for people to remember, this frequency is usually represented by a number, such as _98.7_. We can even think of _98.7_ as a name. If you want to listen to the radio, you know you have to connect your device to _98.7_.

In this case, we can say that _98.7_ is a topic. The radio transmitter is a **publisher** on this topic:

![[attachments/B22403_05_1.jpg]]

Figure 5.1 – Radio transmitter publishing to the 98.7 topic

Now, let’s say you want to listen to that radio from your phone. You will ask your phone to connect to _98.7_ to receive the data.

With this analogy, the phone is then a **subscriber** to the _98.7_ topic.

One important thing to note here is that both the radio transmitter and the phone must use the same type of frequency. For example, if the radio transmitter is using an AM signal, and if the phone is trying to decode an FM signal, it will not work.

Similarly, with ROS 2 topics, both the publisher and subscriber must use the same data type. This data type is called an **interface**.

This is what defines a topic: a **name** and an interface:

![[attachments/B22403_05_2.jpg]]

Figure 5.2 – Publisher and subscriber using the same interface

With that, the communication is complete. The radio transmitter publishes an AM signal on the _98.7_ topic. The phone subscribes to the _98.7_ topic, decoding an AM signal.

## Multiple publishers and subscribers

In real life, there won,t be just one device trying to listen to the radio. Let’s add a few more devices, each one subscribing to the _98.7_ topic and decoding an AM signal:

![[attachments/B22403_05_3.jpg]]

Figure 5.3 – Topic with multiple subscribers

As you can see, one topic can have several subscribers. Each subscriber will get the same data. On the other hand, we could also have several publishers for one topic.

Imagine that there is another radio transmitter, also publishing an AM signal to _98.7_. In this case, both the data from the first transmitter and the second transmitter are received by all listening devices:

![[attachments/B22403_05_4.jpg]]

Figure 5.4 – Multiple publishers and subscribers

The preceding figure shows boxes. Each box represents a node. Thus, we have two radio transmitter nodes, both containing a publisher to the _98.7_ topic. We also have three nodes (phone, radio receiver, and car), each one containing a subscriber to _98.7_.

Note that one subscriber is not aware of the other subscribers. When you listen to the radio on your phone, you have no idea who else is listening to the radio, and on what device.

Also, the phone, the radio receiver and the car are not aware of who is publishing on the radio. They only know they have to subscribe to _98.7_; they don’t know what’s behind it.

On the other side, both radio transmitters are not aware of each other and of who is receiving the data. They just publish on the topic, regardless of who is listening. Thus, we say that topics are **anonymous**. Publishers and subscribers are not aware of other publishers and subscribers. They only publish or subscribe to a topic, using its name and interface.

Any combination of publishers and subscribers is possible. For example, you could have two publishers on the topic and zero subscribers. In this case, the data is still correctly published, but no one receives it. Alternatively, you could have zero publishers and one or more subscribers. The subscribers will listen to the topic but will receive nothing.

## Multiple publishers and subscribers inside one node

A node is not limited to having just one publisher or one subscriber.

Let’s add another radio to our example. We will name it _101.3_, and its data type is FM signal.

The second radio transmitter is now publishing both on the _98.7_ topic and the _101.3_ topic, sending the appropriate type of data for each topic. Let’s also make the car listen to the _101.3_ topic:

![[attachments/B22403_05_5.jpg]]

Figure 5.5 – A node with two publishers

As you can see, the second radio transmitter can publish on several topics, so long as it uses the correct name and interface for each topic.

Now, imagine that the car, while listening to the radio, is also sending its GPS coordinates to a remote server. We could create a topic named **car\_location**, and the interface would contain a latitude and a longitude. The car node now contains one subscriber to the _98.7_ topic, and one publisher to the **car\_location** topic:

![[attachments/B22403_05_6.jpg]]

Figure 5.6 – A node with both a publisher and a subscriber

In the preceding figure, I have also added another node for the server, represented by a computer. The server node will subscribe to the **car\_location** topic so that it can receive the GPS coordinates. Of course, both the publisher and subscriber are using the same interface (latitude and longitude).

Thus, inside one node, you can have any number of publishers and subscribers to different topics with different data types. A node can communicate with several nodes at the same time.

## Wrapping things up

ROS 2 nodes can send messages to other nodes using topics.

Topics are mostly used to send data streams. For example, you could create a hardware driver for a camera sensor, and publish images taken from the camera. Other nodes can then subscribe to the topic and receive the images. You could also publish a stream of commands for a robot to move, and so on.

There are many possibilities for when to use topics, and you will get to know more about them as you progress throughout this book.

Here are some important points about how topics work:

-   A topic is defined by a name and an interface.
-   A topic name must start with a letter and can be followed by other letters, numbers, underscores, tildes, and slashes. For the real-life analogy with radio, I used numbers with dots as topic names. Although it made the examples easier, this is not valid for ROS 2 topics. To make it valid, instead of _98.7_, we would have to create a topic named **radio\_98\_7**.
-   Any publisher or subscriber to a topic must use the same interface.
-   Publishers and subscribers are anonymous. They are not aware of each other; they just know they are publishing or subscribing to a topic.
-   A node can contain several publishers and subscribers to different topics.

Now, how do you create a publisher or a subscriber?

You will do this by adding some code to your nodes. As you saw previously, you can write a Python node using **rclpy** and a C++ node using **rclcpp**. With those two libraries, you can create publishers and subscribers directly in your nodes.

## Writing a topic publisher

In this section, you’ll write your first ROS 2 publisher. To work on the core concepts, we will create a new ROS 2 application and build upon it in the following chapters. This application will be super minimalistic so that we can focus on the concept we want to learn, nothing else.

What we want to do for now is publish a number on a topic. This topic is new and we will _create_ it. You don’t really create a topic—you create a publisher or a subscriber to that topic. This will automatically create the topic name, which will be registered on the graph.

To write a publisher, we need a node. We could use the first node we created in the previous chapter, but the purpose of the node is not the same. Hence, we will create a new node named **number\_publisher**. In this node, we will create a publisher. As to the topic we want to publish to, we will have to choose a name and an interface.

Now, let’s get started with Python.

## Writing a Python publisher

To write a publisher, we need to create a node; to create a node, we need a package. To make things simple, let’s continue using the **my\_py\_pkg** package.

### Creating a node

Navigate inside the **my\_py\_pkg** package, create a Python file, and make it executable:

```
$ cd ~/ros2_ws/src/my_py_pkg/my_py_pkg/
$ touch number_publisher.py
$ chmod +x number_publisher.py
```

![[attachments/3.png]] **Quick tip**: Enhance your coding experience with the **AI Code Explainer** and **Quick Copy** features. Open this book in the next-gen Packt Reader. Click the **Copy** button (**1**) to quickly copy code into your coding environment, or click the **Explain** button (**2**) to get the AI assistant to explain a block of code to you.

![[attachments/image_(2).png]]

![[attachments/4.png]] **The next-gen Packt Reader** is included for free with the purchase of this book. Unlock it by scanning the QR code below or visiting [https://www.packtpub.com/unlock/9781835881408](https://www.packtpub.com/unlock/9781835881408).

![[attachments/9781835881408.png]]

Now, open this file, use the node OOP template (given in [_Chapter 4_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_04.xhtml#_idTextAnchor160)), and modify the required fields to give names that make sense:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
```

Now that you have a **main()** function and a **NumberPublisherNode** class for your node, we can create a publisher.

### Adding a publisher to the node

Where can we create a publisher in this node? We will do that in the constructor.

And before we write the code, we need to ask ourselves a question: what is the name and the interface for this topic?

-   **Case 1**: You’re publishing to a topic that already exists (other publishers or subscribers on that topic), and then you use the same name and interface
-   **Case 2**: You create a publisher for a new topic (what we are doing now), and then you have to choose a name and interface

For the name, let’s keep it simple and use **number**. If we publish a number, we can expect to receive this number on a **number** topic. If you were to publish a temperature, you could name the topic **temperature**.

For the interface, you have two choices: use an existing interface or create a custom one. To get started, we will ==use an existing interface.== To make this easier, I will just tell you what to use; you’ll learn how to find other interfaces yourself later.

Let’s use **example\_interfaces/msg/Int64**. To get more details about what’s in the interface, we can run **ros2 interface show <interface\_name>** in the Terminal:

```
$ ros2 interface show example_interfaces/msg/Int64
# Some comments
int64 data
```

Great—this is exactly what we need: an **int64** number.

Now that we have this information, let’s create the publisher. First, import the interface, and then create the publisher in the constructor:

```python
import rclpy
from rclpy.node import Node
 from example_interfaces.msg import Int64 
class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
         self.number_publisher_ = self.create_publisher(Int64, "number", 10) 
```

To import the interface, we must specify the name of the package (**example\_interfaces**), then the folder name for topic messages (**msg**), and finally the class for the interface (**Int64**).

To create the publisher, we must use the **create\_publisher()** method from the **Node** class. Inheriting from this class gives us access to all ROS 2 functionalities. In this method, you have to provide ==three arguments==:

-   **Topic interface**: We’ll use **Int64** from the **example\_interfaces** package.
-   **Topic name**: As defined previously, this is **number**.
-   **Queue size**: If the messages are published too fast and subscribers can’t keep up, messages will be buffered (up to 10 here) so that they’re not lost. This can be important if you send large messages (images) at a high frequency, on a lossy network. As we get started, there’s no need to worry about this; I recommend that you just set the queue size to **10** every time.

With this, we now have a publisher on the **number** topic. However, if you just run your code like this, nothing will happen. A publisher won’t publish automatically on a topic. You have to write the code for that to happen.

### Publishing with a timer

A common behavior in robotics is to do _X_ action every _Y_ seconds—for example, publish an image from a camera every **0.5** seconds, or in this case, publish a number on a topic every **1.0** second. As seen in [_Chapter 4_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_04.xhtml#_idTextAnchor160), to do this, you must implement a timer and a callback function.

Modify the code inside the node so that you publish on the topic from a timer callback:

```python
def __init__(self):
    super().__init__("number_publisher")
    self.number_ = 2
    self.number_publisher_ = self.create_publisher(Int64, "number", 10)
    self.number_timer_ = self.create_timer(1.0, self.publish_number)
    self.get_logger().info("Number publisher has been started.")
def publish_number(self):
    msg = Int64()
    msg.data = self.number_
    self.number_publisher_.publish(msg)
```

After creating the publisher with **self.create\_publisher()**, we create a timer with **self.create\_timer()**. Here, we say that we want the **publish\_number()** method to be called every **1.0** second. This will happen when the node is spinning.

On top of that, I also added a log at the end of the constructor to say that the node has been started. I usually do this as a best practice so that I can see when the node is fully initialized on the Terminal.

In the **publish\_number()** method, we publish on the topic:

1.  We create an object from the **Int64** class. This is the interface—in other words, the message to send.
2.  This object contains a **data** field. How do we know this? We found this previously when we ran **ros2 interface show example\_interfaces/msg/Int64**. Thus, we provide a number in the **data** field of the message. For simplicity, we specify the same number every time we run the callback function.
3.  We publish the message using the **publish()** method from the publisher.

This code structure is super common in ROS 2. Any time you want to publish data from a sensor, you will write something similar.

### Building the publisher

To try your code, you need to install the node.

Before we do this, since we’re ==using a new dependency (**example\_interfaces** package), we also need to add one line to the **package.xml** file of the **my\_py\_pkg** package:==

```xml
 <depend>rclpy </depend>
  <depend>example_interfaces </depend> 
```

As you add more functionalities inside your package, you will add any other ROS 2 dependency here.

To ==install the node==, open the **setup.py** file from the **my\_py\_pkg** package and add a new line to create another executable:

```
entry_points={
    'console_scripts': [
        "test_node = my_py_pkg.my_first_node:main" , 
         "number_publisher = my_py_pkg.number_publisher:main" 
    ],
},
```

Make sure you add a comma between each line; otherwise, you could encounter some strange errors when building the package.

Here, we’ve created a new executable named **number\_publisher**.

Note

This time, as you can see from this example, the node name, filename, and executable name are the same: **number\_publisher**. This is a common thing to do. Just remember that those names represent three different things.

Now, go to your workspace root directory and build the **my\_py\_pkg** package:

```
$ cd ~/ros2_ws/
$ colcon build --packages-select my_py_pkg
```

You can add **\--symlink-install** if you want to, so that you don’t need to run **colcon build** every time you modify the **number\_publisher** node.

### Running the publisher

After the package has been built successfully, source your workspace and start the node:

```
$ source install/setup.bash # or source ~/.bashrc
$ ros2 run my_py_pkg number_publisher
[INFO] [1711526444.403883187] [number_publisher]: Number publisher has been started.
```

The node is running, but apart from the initial log, nothing is displayed. That’s normal—we didn’t ask the node to print anything else.

How do we know that the publisher is working? We could write a subscriber node right away and see if we receive the messages. But before we do that, we can test the publisher directly from the Terminal.

Open a new Terminal window and list all topics:

```
$ ros2 topic list
/number
/parameter_events
/rosout
```

Here, you can find the **/number** topic.

Note

As you can see, there is an added leading slash in front of the topic name. We only wrote **number** in the code, not **/number**. This is because ROS 2 names (nodes, topics, and so on) are organized inside namespaces. Later, we will see that you can add a namespace to put all your topics or nodes inside the **/abc** namespace, for example. In this case, the topic name would be **/abc/number**. Here, as no namespace is provided, a leading slash is added to the name, even if we don’t provide it in the code. We could call this the _global_ namespace.

With the ==**ros2 topic echo <topic\_name>** command==, you can subscribe to the topic directly from the subscriber and see what’s being published. We will learn more about this command later in this chapter:

```
$ ros2 topic echo /number
data: 2
---
data: 2
---
```

As you can see, we get one new message per second, which contains a **data** field with a value of **2**. This is exactly what we wanted to do in the code.

With that, we’ve finished our first Python publisher. Let’s switch to C++.

## Writing a C++ publisher

Here, the process is the same as for Python. We will create a new node, and in this node, add a publisher and a timer. In the timer callback function, we will create a message and publish it.

I will go a bit more quickly in this section as the explanations are the same. We will just focus on the specificities of the C++ syntax with ROS 2.

Note

For everything related to C++ in this book, make sure you follow the explanations using the GitHub code on the side. I may not provide the full code, only the important snippets that are crucial for comprehension.

### Creating a node with a publisher and a timer

First, let’s create a new file for our **number\_publisher** node in the **my\_cpp\_pkg** package:

```
$ cd ~/ros2_ws/src/my_cpp_pkg/src/
$ touch number_publisher.cpp
```

Open this file and write the code for the node. You can start from the OOP template and add the publisher, timer, and callback function. The complete code for this chapter can be found in this book’s GitHub repository: [https://github.com/PacktPublishing/ROS-2-from-Scratch](https://github.com/PacktPublishing/ROS-2-from-Scratch).

I will now comment on a few important lines:

```
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
```

To include an interface for a topic, use **"<package\_name>/msg/<message\_name>.hpp"**.

Then, in the constructor, add the following:

```
number_publisher_ = this->create_publisher <example_interfaces::msg::Int64>("number", 10);
```

In C++, we also use the **create\_publisher()** method from the **Node** class. The syntax is a bit different since templates are used, but you can still find the topic interface, topic name, and queue size (as a reminder, you can set it to **10** every time).

The publisher is also declared as a private attribute in the class:

```
rclcpp::Publisher <example_interfaces::msg::Int64>::SharedPtr number_publisher_;
```

As you can see, we use the **rclcpp::Publisher** class, and as for many things in ROS 2, we use a shared pointer. For several common classes, ROS 2 provides**::SharedPtr**, which would be the same thing as writing `std::shared\_ptr<the publisher>`.

Let’s go back to the constructor:

```
number_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NumberPublisherNode::publishNumber, this));
RCLCPP_INFO(this->get_logger(), "Number publisher has been started.");
```

After creating the publisher, we create a timer to call the **publishNumber** method every **1.0** second. Finally, we print a log so that we know that the constructor code has been executed:

```
void publishNumber()
{
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_;
    number_publisher_->publish(msg);
}
```

This is the callback method. As for Python, we create an object from the interface class, after which we fill any field from this interface and publish the message.

### Building and running the publisher

Once you’ve written the node with the publisher, timer, and callback function, it’s time to build it.

As we did for Python, open the **package.xml** file of the **my\_cpp\_pkg** package and add one line for the dependency to **example\_interfaces**:

```
 <depend>rclcpp </depend>
  <depend>example_interfaces </depend> 
```

Then, open the **CMakeLists.txt** file from the **my\_cpp\_pkg** package and add the following lines:

```
find_package(rclcpp REQUIRED)
 find_package(example_interfaces REQUIRED) 
add_executable(test_node src/my_first_node.cpp)
ament_target_dependencies(test_node rclcpp)
 add_executable(number_publisher src/number_publisher.cpp) 
 ament_target_dependencies(number_publisher rclcpp example_interfaces) 
install(TARGETS
  test_node
   number_publisher 
  DESTINATION lib/${PROJECT_NAME}/
)
```

For any new dependency, we need to add a new **find\_package()** line.

Then, we create a new executable. Note that we also provide **example\_interfaces** in the arguments of **ament\_target\_dependencies()**. If you omit this, you will get an error during compilation.

Finally, there’s no need to re-create the **install()** block. Just add the executable in a new line, without any commas between the lines.

Now, you can build, source, and run:

```
$ cd ~/ros2_ws/
$ colcon build --packages-select my_cpp_pkg
$ source install/setup.bash
$ ros2 run my_cpp_pkg number_publisher
[INFO] [1711528108.225880935] [number_publisher]: Number publisher has been started.
```

The node containing the publisher is up and running. By using **ros2 topic list** and **ros2 topic echo <topic\_name>**, you can find the topic and see what’s being published.

Now that you’ve created a publisher and you know it’s working, it’s time to learn how to create a subscriber for that topic.

## Writing a topic subscriber

To continue improving our application, let’s create a new node that will subscribe to the **/number** topic. Each number that’s received will be added to a counter. We want to print the counter every time it’s updated.

As we did previously, let’s start the full explanations with Python, and then see the syntax specificities with C++.

## Writing a Python subscriber

You can find the complete code for this Python node on GitHub. Many things we need to do here are identical to what we did previously, so I won’t fully detail every step. Instead, we will focus on the most important things so that we can write the subscriber.

### Creating a Python node with a subscriber

Create a new node named **number\_counter** inside the **my\_py\_pkg** package:

```
$ cd ~/ros2_ws/src/my_py_pkg/my_py_pkg/
$ touch number_counter.py
$ chmod +x number_counter.py
```

In this file, you can write the code for the node and add a subscriber. Here’s the explanation, step by step:

```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 from example_interfaces.msg import Int64 
```

Since we want to create a subscriber to receive what we sent with the publisher, we need to use the same interface. Hence, ==we import **Int64** as well==. Then, we can create the subscriber:

```
class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0
         self.number_subscriber_ = self.create_subscription(Int64, "number", self.callback_number, 10) 
       self.get_logger().info("Number Counter has been started.")
```

As for publishers, we will create subscribers in the node’s constructor. Here, we use the **create\_subscription()** method from the **Node** class. With this method, you need to provide four arguments:

-   **Topic interface**: **Int64**. This needs to be the same for both the publisher and subscriber.
-   **Topic name**: **number**. This is the same name as for the publisher. Note that I don’t provide any additional slash here. This will be added automatically, so the topic name will become **/number**.
-   **Callback function**: Do you remember when I told you that almost everything is a callback in ROS 2? We use a callback method for the subscriber here as well. When the node is spinning, it will stay alive and all registered callbacks will be ready to be called. ==Whenever a message is published on the **/number** topic, it will be received here, and we will be able to use it and process it inside the callback method (that we need to implement).==
-   **Queue size**: As seen previously, you can set it to **10** and forget about it for now.

Now, let’s see the implementation for the callback method, which I named **callback\_number**:

Note

As a best practice, I recommend naming callback methods for topics **`callback\_<topic>`**. By adding the **callback\_** prefix, you make it clear that this method is a callback and shouldn’t be called directly in the code. This can prevent lots of errors in the future.

```
def callback_number(self, msg: Int64):
    self.counter_ += msg.data
    self.get_logger().info("Counter:  " + str(self.counter_))
```

In a subscriber callback, you receive the message directly in the parameters of the function. Since we know that **Int64** contains a **data** field, we can access it with **msg.data**.

Now, we add the received number to a **counter\_** attribute and print the counter every time with a ROS 2 log.

Note

As a best practice, I have specified the **Int64** type for the **msg** argument of the method. This isn’t mandatory for Python code to work, but it adds an extra level of safety (we are sure that we should receive **Int64** and nothing else) and it can sometimes make your IDE work better with auto-completion.

To finish the node, don’t forget to add the **main()** function after the **NumberCounterNode** class.

### Running the Python subscriber

Now, to try the code, add a new executable to the **setup.py** file of your Python package:

```
entry_points={
    'console_scripts': [
        "test_node = my_py_pkg.my_first_node:main",
        "number_publisher = my_py_pkg.number_publisher:main" , 
         "number_counter = my_py_pkg.number_counter:main" 
    ],
},
```

Then, build the package and source the workspace (from now on, I will not write those commands every time since they’re always the same).

Now, run each node (**number\_publisher** and **number\_counter**) in a different Terminal:

```
$ ros2 run my_py_pkg number_publisher
[INFO] [1711529824.816514561] [number_publisher]: Number publisher has been started.
$ ros2 run my_py_pkg number_counter
[INFO] [1711528797.363370081] [number_counter]: Number Counter has been started.
[INFO] [1711528815.739270510] [number_counter]: Counter:  2
[INFO] [1711528816.739186942] [number_counter]: Counter:  4
[INFO] [1711528817.739050485] [number_counter]: Counter:  6
[INFO] [1711528818.738992607] [number_counter]: Counter:  8
```

As you can see, the **number\_counter** node adds **2** to the counter every **1.0** second. If you see this, then the publish/subscribe communication between your two nodes is working.

You can start and stop the **number\_publisher** node and see that every time you start it, **number\_counter** continues to add numbers from the current count.

## Writing a C++ subscriber

Let’s create the **number\_counter** node in C++. The principle is the same, so let’s just focus on the syntax here.

### Creating a C++ node with a subscriber

Create a new file for your node:

```
$ cd ~/ros2_ws/src/my_cpp_pkg/src/
$ touch number_counter.cpp
```

Open this file and write the code for the node (once again, the complete code is on GitHub).

To create a subscriber in your node, run the following code:

```
number_subscriber_ = this->create_subscription <example_interfaces::msg::Int64>(
           "number",
           10,
           std::bind(&NumberCounterNode::callbackNumber, this, _1));
```

We find the same components as for Python (but in a different order): topic interface, topic name, queue size, and callback for received messages. For **\_1** to work, don’t forget to add **using namespace std::placeholders;** before it.

Note

Even if the **rclpy** and **rclcpp** libraries are supposed to be based on the same underlying code, there can still be some differences in the API. Don’t worry if the code sometimes doesn’t look the same between Python and C++.

The subscriber object is declared as a private attribute:

```
rclcpp::Subscription <example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
```

We use the **rclcpp::Subscription** class here, and once again, we create a shared pointer to that object.

We then have the callback method, **callbackNumber**:

```
void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
{
    counter_ += msg->data;
    RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);
}
```

The message we receive in the callback is also a (**const**) shared pointer. Hence, don’t forget to use **\->** when accessing the **data** field.

In this callback, we add the received number to the counter and print it.

### Running the C++ subscriber

Create a new executable for that node. Open **CMakeLists.txt** and add the following code:

```
 add_executable(number_counter src/number_counter.cpp) 
 ament_target_dependencies(number_counter rclcpp example_interfaces) 
```

```
install(TARGETS
  test_node
  number_publisher
   number_counter 
  DESTINATION lib/${PROJECT_NAME}/
)
```

Then, build **my\_cpp\_pkg**, source the workspace, and run both the publisher and the subscriber node in different Terminals. You should see a similar output to what we had with Python.

## Running the Python and C++ nodes together

We’ve just created a publisher and subscriber for both Python and C++. The topic we use has the same name (**number**) and interface (**example\_interfaces/msg/Int64**).

If the topic is the same, it means that you could start the Python **number\_publisher** node with the C++ **number\_counter** node, for example.

Let’s verify that:

```
$ ros2 run  my_py_pkg  number_publisher
[INFO] [1711597703.615546913] [number_publisher]: Number publisher has been started.
$ ros2 run  my_cpp_pkg  number_counter
[INFO] [1711597740.879160448] [number_counter]: Number Counter has been started.
[INFO] [1711597741.607444197] [number_counter]: Counter: 2
[INFO] [1711597742.607408224] [number_counter]: Counter: 4
```

You can also try the opposite by running the C++ **number\_publisher** node with the Python **number\_counter** node.

Why is it working? Simply because ROS 2 is language-agnostic. You could have any node written in any (supported) programming language, and this node could communicate with all the other nodes in the network, using topics and other ROS 2 communications.

ROS 2 communications happen at a lower level, using **Data Distribution Service** (**DDS**). This is the middleware part and is responsible for sending and receiving messages between nodes. When you write a Python or C++ node, you are using the same DDS functionality, with an API implemented in either **rclpy** or **rclcpp**.

I will not go too far with this explanation as it’s quite advanced and not really in the scope of this book. If there is just one thing to remember from this, it’s that Python and C++ nodes can communicate with each other using ROS 2 communication features. You can create some nodes in Python and other nodes in C++; just make sure to use the same communication name and interface on both sides.

## Additional tools to handle topics

You’ve just written a bunch of nodes containing publishers and subscribers. We will now explore how ROS 2 tools can help you do more things with topics.

We will explore the following topics:

-   Introspection with **rqt\_graph**
-   Introspection and debugging with the **ros2 topic** command line
-   Changing a topic name when starting a node
-   Replaying topic data with bags

## Introspecting topics with rqt\_graph

We used **rqt\_graph** to visualize nodes in [_Chapter 3_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_03.xhtml#_idTextAnchor095). Let’s run it again and see how to introspect the publisher and subscriber we have just created.

First, start both the **number\_publisher** and **number\_counter** nodes (from any package: **my\_py\_pkg** or **my\_cpp\_pkg**).

Then, start **rqt\_graph** in another Terminal:

```
$ rqt_graph
```

If needed, refresh the view a few times and select **Nodes/Topics (all)**. You can also uncheck the **Dead sinks** box and the **Leaf topics** box. This will allow you to see topics even if there is just one subscriber and no publisher, or one publisher and no subscriber:

![[attachments/B22403_05_7.jpg]]

Figure 5.7 – The number topic on rqt\_graph

There, we can see the **number\_publisher** node and the **number\_counter** node. In the middle, we have the **/number** topic, and we can see which node is a publisher or a subscriber.

The **rqt\_graph** package can be extremely useful when debugging topics. Imagine that you run some nodes and you’re wondering why topic messages are not received by a subscriber. Maybe those nodes are not using the same topic name. You can easily see this with **rqt\_graph**:

![[attachments/B22403_05_8.jpg]]

Figure 5.8 – Topic name mismatch between publisher and subscriber

In this example, I made an intentional error in the topic name inside the publisher. Instead of **number**, I have written **numberr**. With **rqt\_graph**, I can see where the issue is. The two nodes are not communicating with each other.

## The ros2 topic command line

With **ros2 node**, we get additional command-line tools for nodes. For topics, we will use **ros2 topic**.

If you run **ros2 topic -h**, you’ll see that there are quite a lot of commands. You already know some of them. Here, I will do a quick recap and explore a few more commands that can be useful when debugging topics.

First, to list all topics, use **ros2** **topic list**:

```
$ ros2 topic list
/number
/parameter_events
/rosout
```

As you can see, we get the **/number** topic. You will also always get **/parameter\_events** and **/rosout** (all ROS 2 logs are published on this topic).

With **ros2 topic info <topic\_name>**, you can get the interface for the topic, as well as the number of publishers and subscribers for that topic:

```
$ ros2 topic info /number
Type: example_interfaces/msg/Int64
Publisher count: 1
Subscription count: 1
```

Then, to go further and see the details for the interface, you can run the following command:

```
$ ros2 interface show example_interfaces/msg/Int64
# some comments
int64 data
```

With this, we have all the information we need to create an additional publisher or subscriber to the topic.

On top of that, we can also directly subscribe to the topic from the Terminal with **ros2 topic echo <topic\_name>**. That’s what we did just after writing the publisher so that we can make sure it’s working before we write any subscriber:

```
$ ros2 topic echo /number
data: 2
---
data: 2
---
```

On the other hand, you can publish to a topic directly from the Terminal with **`ros2 topic pub -r <frequency> <topic\_name> <interface> <message\_in\_json>`**. To test this, stop all nodes, and start only the **number\_counter** node in one Terminal. Apart from the first log, nothing will be printed. Then, run the following command in another Terminal:

```
$ ros2 topic pub -r 2.0 /number example_interfaces/msg/Int64 \"{data: 7}"
publisher: beginning loop
publishing #1: example_interfaces.msg.Int64(data=7)
publishing #2: example_interfaces.msg.Int64(data=7)
```

This will publish on the **/number** topic at **2.0** Hertz (every **0.5** seconds). When you run this, you’ll see some logs on the **number\_counter** node, meaning that the messages have been received:

```
[INFO] [1711600360.459298369] [number_counter]: Counter: 7
[INFO] [1711600360.960216275] [number_counter]: Counter: 14
[INFO] [1711600361.459896877] [number_counter]: Counter: 21
```

This way, you can test a subscriber without having to write a publisher first. Note that this only really works for topics with a simple interface. When the interface contains too many fields, it becomes too complicated to write everything on the Terminal.

Note

Both **ros2 topic echo** and **ros2 topic pub** can save you lots of time, and it’s also great for collaborating with other people on a project. You could be responsible for writing a publisher, and someone else would write a subscriber. With those command-line tools, both of you can make sure the topic communication is working. Then, when you run the two nodes together, you know that the data you send or receive is correct.

## Changing a topic name at runtime

In [_Chapter 4_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_04.xhtml#_idTextAnchor160), you learned how to change a node name at runtime—that is, by adding **\--ros-args -r \_\_node:=<new\_name>** after the **ros2** **run** command.

So, for any additional argument you pass after **ros2 run**, add **\--ros-args**, but only once.

Then, you can also change a topic name at runtime. To do that, add another **\-r**, followed by **<topic\_name>:=<new\_topic\_name>**.

For example, let’s rename our topic from **number** to **my\_number**:

```
$ ros2 run my_py_pkg number_publisher --ros-args -r number:=my_number
```

Now, if we start the **number\_counter** node, to be able to receive the messages, we also need to modify the topic name:

```
$ ros2 run my_py_pkg number_counter --ros-args -r number:=my_number
```

With this, the communication will work, but this time using the **my\_number** topic.

To make things a bit more interesting, let’s keep those two nodes running, and let’s run another publisher to this topic, using the same **number\_publisher** node. As you know, we can’t have two nodes running with the same name. Thus, we will have to rename both the node and the topic. In a third Terminal, run the following code:

```
$ ros2 run my_py_pkg number_publisher --ros-args -r \ __node:=number_publisher_2 -r number:=my_number
```

After you run this, you’ll see that the **number\_counter** receives messages twice as fast since there are two nodes publishing one message every **1.0** second.

On top of that, let’s start **rqt\_graph**:

![[attachments/B22403_05_9.jpg]]

Figure 5.9 – Two publishers and a subscriber, with a renamed topic

We’ll see that we have two nodes containing a publisher on the **my\_number** topic, and one node containing a subscriber.

Changing topic names at runtime will be quite useful for you, especially when you want to run several existing nodes that you can’t modify. Even if you can’t rewrite the code, you can modify the names at runtime. Now, let’s continue with the tools and explore ROS 2 bags.

## Replaying topic data with bags

Imagine this scenario: you’re working on a mobile robot that’s supposed to perform in a certain way when navigating outside and while it’s raining.

Now, this means you will need to run the robot in those conditions so that you can develop your application. There are a few problems: maybe you won’t have access to the robot every time, or you can’t take it outside, or it’s simply not raining every day.

A solution to this is to use ==ROS 2 bags. Bags allow you to record a topic and replay it later.== Thus, you can run the experiment once with the required conditions, and then replay the data just like it was recorded. With this data, you can develop your application.

Let’s consider another scenario: you work with a piece of hardware that isn’t stable yet. Most of the time, it doesn’t work properly. You could record a bag while the hardware is working fine, and then replay this bag to develop your application instead of running the hardware again and again and wasting time on this.

To work with ROS 2 bags, you must use the **ros2 bag** command-line tool. Let’s learn how to save and replay a topic with bags.

First, stop all nodes and run the **number\_publisher** node only.

We already know that the topic name is **/number**. You can retrieve that with **ros2 topic list** if needed. Then, in another Terminal, record the bag with **`ros2 bag record <list of topics> -o <bag\_name>`**. To make things more organized, I suggest that you create a **bags** folder and record from within this folder:

```
$ mkdir ~/bags
$ cd ~/bags/
$ ros2 bag record /number -o bag1
...
[INFO] [1711602240.190476880] [rosbag2_recorder]: Subscribed to topic '/number'
[INFO] [1711602240.190542569] [rosbag2_recorder]: Recording...
[INFO] [1711602240.190729185] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...
```

At this point, the bag is recording and saving all incoming messages inside a database. Let it run for a few seconds, then stop it with _Ctrl_ + _C_:

```
[INFO] [1711602269.786924027] [rosbag2_cpp]: Writing remaining messages from cache to the bag. It may take a while
[INFO] [1711602269.787416646] [rosbag2_recorder]: Event publisher thread: Exiting
[INFO] [1711602269.787547010] [rosbag2_recorder]: Recording stopped
```

The **ros2 bag** command will exit, and you’ll end up with a new directory named **bag1**. In this directory, you will find a **.mcap** file containing the recorded messages and a YAML file with more information. If you open this YAML file, you’ll see the recorded duration, number of recorded messages, and topics that were recorded.

Now, you can replay the bag, which means you’ll publish on the topic exactly like it was done when recording.

Stop the **number\_publisher** node and replay the bag with **ros2 bag** **play <path\_to\_bag>**:

```
$ ros2 bag play ~/bags/bag1/
```

This will publish all the recorded messages, with the same duration as the recording. So, if you record for 3 minutes and 14 seconds, the bag will replay the topic for 3 minutes and 14 seconds. Then, the bag will exit, and you can play it again if you want.

While the bag is playing, you can run your subscriber(s). You can do a quick test with **ros2 topic echo /number** and see the data. You can also run your **number\_counter** node, and you will see that the messages are received.

You are now able to save and replay a topic using ROS 2 bags. You can explore more advanced options with **ros2** **bag -h**.

As you’ve seen, there are quite a few available tools for topics. Use these tools as often as possible to introspect, debug, and test your topics. They will save you lots of time when you’re developing your ROS 2 application.

We’re almost done with topics. So far, all we’ve done is use existing interfaces. Now, let’s learn how to create a custom interface.

## Creating a custom interface for a topic

When creating a publisher or subscriber for a topic, you know that you have to use a name and an interface.

It’s quite easy to publish or subscribe to an existing topic: you’ll find the name and interface using the **ros2** command line, and use that in your code.

Now, if you want to start a publisher or subscriber for a new topic, you will need to choose a name and interface by yourself:

-   **Name**: No problem—it’s just a chain of characters
-   **Interface**: You have two choices—using an existing interface that works with your topic or creating a new one

Let’s try to apply the ROS 2 philosophy of not reinventing the wheel. When you create a new topic, check if there is any existing interface that can match your needs. If so, then use it; don’t recreate it.

First, you’ll learn where you can find existing interfaces. Then, you’ll learn how to create a new one.

Note

It’s quite common to use the word _message_ when talking about topic interfaces. I could have named this section _Creating a custom message_. In the following section, when I talk about messages, I’m referring to topic interfaces.

## Using existing interfaces

Before you start a new publisher or subscriber for a topic, take some time to think about what kind of data you want to send or receive. Then, check if an already existing interface contains what you need.

### Where to find interfaces

Just like nodes, interfaces are organized in packages. You can find the most common packages for ROS 2 interfaces here: [https://github.com/ros2/common\_interfaces](https://github.com/ros2/common_interfaces). Not all existing interfaces are listed here, but it’s already quite a lot. For other interfaces, a simple search on the internet should bring you to the corresponding GitHub repository.

In this common interfaces repository, you can find the **Twist** message we used with Turtlesim, inside the **geometry\_msgs** package. As you can see, for topic interfaces, we then have an additional **msg** folder, which contains all the message definitions for that package.

Now, let’s say you want to create a driver node for a camera and publish the images to a topic. If you look inside the **sensor\_msgs** package, and then inside the **msg** folder, you’ll find a file named **Image.msg**. This _Image_ message is probably suitable for your needs. It is also used by a lot of other people, so it will even make your life easier.

### Using an existing interface in your code

To use this message, make sure you’ve installed the package that contains the message—in this case, **sensor\_msgs**. As a quick reminder, to install a ROS 2 package, you can run **`sudo apt install ros-<distro>-<package-name>`**:

```
$ sudo apt install ros-jazzy-sensor-msgs
```

Maybe the package was already installed. If not, source your environment again afterward. Then, you can find the details regarding the interface with **`ros2 interface show <interface>`**:

```
$ ros2 interface show sensor_msgs/msg/Image
```

To use this message in your code, just follow what we did in this chapter (with the **example\_interfaces/msg/Int64** message):

1.  In the **package.xml** file of the package where you write your nodes, add the dependency to the interface package.
2.  In your code, import the message and use it in your publisher or subscriber.
3.  For C++ only: Add the dependency to the interface package in the **CMakeLists.txt** file.

We will see another example of this process very soon, just after we create our interface.

At this point, you know how to find and use existing messages in your code. But should you always do that?

### When not to use existing messages

For common use cases, sensors, and actuators, you will probably find what you need. However, if the interface doesn’t match exactly what you want, you will have to create a new one.

There are a few packages containing basic interfaces, such as **example\_interfaces**, or even **std\_msgs**. You could be tempted to use them in your code. As a best practice, it’s better to avoid it. Just read the comments from the message definitions to be sure of that:

```
$ ros2 interface show example_interfaces/msg/Int64
# This is an example message of using a primitive datatype, int64.
```

```
# If you want to test with this that's fine, but if you are deploying it into a system you should create a semantically meaningful message type.
# If you want to embed it in another message, use the primitive data type instead.
int64 data
$ ros2 interface show std_msgs/msg/Int64
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.
int64 data
```

As you can see, the **std\_msgs** package is deprecated, and **example\_interfaces** is only recommended to make tests—which is what we’ve done in this chapter so far to help us learn various topics.

As a general rule, if you don’t find exactly what you need in the existing interface packages, then create your own interface. It’s not hard to do and will always be the same process.

## Creating a new topic interface

You will now create your first custom interface for a topic. We will see how to set a package up for that, how to create and build the interface, and how to use it in our code.

### Creating and setting up an interfaces package

Before we create any topic interface (message), we need to create a new package and set it up for building interfaces. As a best practice, in your application, you will have one package dedicated to custom interfaces. This means that you create interfaces only in this package, and you keep this package only for interfaces—no nodes or other things, just interfaces. This will make it much easier when you’re scaling the application and will help you avoid creating a dependency mess.

A common practice when naming this interface package is to start with the name of your application or robot and add the **\_interfaces** suffix. So, if your robot is named **abc**, you should use **abc\_interfaces**.

We don’t have a robot for this example, so let’s just name the package **my\_robot\_interfaces**.

Create a new package with the **ament\_cmake** build type and no dependencies. You don’t even need to provide the build type since **ament\_cmake** is the one used by default. Navigate to the **src** directory of your workspace and create this package:

```
$ cd ~/ros2_ws/src/
$ ros2 pkg create my_robot_interfaces
```

At this point, your workspace should contain three packages: **my\_py\_pkg**, **my\_cpp\_pkg**, and **my\_robot\_interfaces**.

We need to set this new package up and modify a few things so it can build messages. Go into the package, remove the **src** and **include** directories, and create a new **msg** folder:

```
$ cd my_robot_interfaces/
$ rm -r src/ include/
$ mkdir msg
```

Now, open the **package.xml** file for this package. After **<buildtool\_depend>ament\_cmake</buildtool\_depend>**, add the following three lines. I recommend that you just copy and paste them so that you don’t make any mistakes:

```
 <build_depend>rosidl_default_generators </build_depend>
 <exec_depend>rosidl_default_runtime </exec_depend>
 <member_of_group>rosidl_interface_packages </member_of_group>
```

With that, the **package.xml** file is complete and you won’t have to do anything else with it for now. Open the **CMakeLists.txt** file. After **find\_package(ament\_cmake REQUIRED)**, and before **ament\_package()**, add the following lines (you can also remove the **if(BUILD\_TESTING)** block):

```
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  # we will add the name of our custom interfaces here
)
ament_export_dependencies(rosidl_default_runtime)
```

There’s not much to understand about these lines you’re adding. They will find some dependencies (**rosidl** packages) and prepare your package so that it can build interfaces.

At this point, your package is ready and you can add new interfaces. You will only need to do this setup phase once. At this point, adding a new interface is very quick.

### Creating and building a new topic interface

Let’s say we want to create a publisher to send some kind of hardware status for our robot, including the robot version, internal temperature, a flag to know if the motors are ready, and a debug message.

We’ve looked at existing interfaces and nothing matches. How can you name this new interface? Here are the rules you have to follow:

-   Use UpperCamelCase—for example, HardwareStatus
-   Don’t write **Msg** or **Interface** in the name as this would add unnecessary redundancy
-   Use **.msg** for the file extension

Following these rules, create a new file named **HardwareStatus.msg** in the **msg** folder:

```
$ cd ~/ros2_ws/src/my_robot_interfaces/msg/
$ touch HardwareStatus.msg
```

Inside this file, we can add the definition for the message. Here’s what you can use:

-   Built-in types, such as **bool**, **byte**, **int64**, **float64**, and **string**, as well as arrays of those types. You can find the complete list here: [https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html#field-types](https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html#field-types).
-   Other existing messages, using the name of the package, followed by the name of the message —for example, **geometry\_msgs/Twist** (don’t add the **msg** folder here).

To make things simple here, we will start with only built-in types. Write the following inside the message file:

```
int64 version
float64 temperature
bool are_motors_ready
string debug_message
```

For each field, we provide the data type, and then the name of the field.

Now, how are we going to build this message? How can we get a Python or C++ class that we can include and use in our code?

To build the message, you simply have to add one line to **CMakelists.txt**, specifying the relative path to the message file:

```
rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/HardwareStatus.msg" 
)
```

For each new interface you build in this package, you will add one line inside the **rosidl\_generate\_interfaces()** function. _Don’t add any commas between_ _the lines._

Now, save all the files and build your new package:

```
$ cd ~/ros2_ws/
$ colcon build --packages-select my_robot_interfaces
Starting >>> my_robot_interfaces
Finished  < < < my_robot_interfaces [4.00s]
Summary: 1 package finished [4.28s]
```

The build system will take the interface definition you’ve written and use it to generate source code for Python and C++:

![[attachments/B22403_05_10.jpg]]

Figure 5.10 – Build system for interfaces

Once you’ve built the package, make sure you source the environment. You should be able to see your interface from the Terminal (don’t forget to use auto-completion to build the command faster and be sure you have the correct name):

```
$ source ~/.bashrc
$ ros2 interface show my_robot_interfaces/msg/HardwareStatus
int64 version
float64 temperature
bool are_motors_ready
string debug_message
```

If you see this, it means that the build process succeeded. If you can’t see the interface in the Terminal, then you need to go back and check that you did all the steps correctly.

### Using your custom message in your code

Let’s say you want to use your new interface in the **number\_publisher** node you created in this chapter, inside the **my\_py\_pkg** package.

First, open the **package.xml** file from the **my\_py\_pkg** package and add a dependency to **my\_robot\_interfaces**:

```
 <depend>rclpy </depend>
 <depend>example_interfaces </depend>
  <depend>my_robot_interfaces </depend> 
```

Then, for Python, do the following:

1.  Import the message by by adding the following import line in your code:
    
    ```
    from my_robot_interfaces.msg import HardwareStatus
    ```
    
2.  Create a publisher and specify the **HardwareStatus** interface.
3.  Create a message in your code, like so:
    
    ```
    msg = HardwareStatus()
    msg.temperature = 34.5
    ```
    

Note

If you’re using VS Code, the message might not be recognized after you import it. Close VS Code and open it again in a sourced environment. So, make sure the interface has been built correctly, then source the environment, and open VS code.

If you want to use this message in your C++ node from the **my\_cpp\_pkg** package, add the dependency to **my\_robot\_interfaces** in the **package.xml** file of **my\_cpp\_package**. Then, do the following:

1.  Import the message by adding the following **include** line in your code:
    
    ```
    #include "my_robot_interfaces/msg/hardware_status.hpp"
    ```
    
2.  Create a publisher and specify the interface with **<my\_robot\_interfaces::msg::** **HardwareStatus>**.
3.  Create a message in your code, like so:
    
    ```
    auto msg = my_robot_interfaces::msg::HardwareStatus();
    msg.temperature = 34.5;
    ```
    

When using VS code, the C++ include will not be recognized. You need to add a new line to the **c\_cpp\_properties.json** file that was auto-generated (inside a **.vscode** folder) when you started VS Code. You can find this file from VS Code using the explorer on the left. Then, in the **includePath** array, add the following line:

```
"includePath": [
        "/opt/ros/jazzy/include/**",
         "/home/ <user>/ros2_ws/install/my_robot_interfaces/include/**" ,
        "/usr/include/**"
    ],
```

You can now create and use your custom interface for topics. As you’ve seen, first, check whether there’s any existing interface that matches your needs. If there is, don’t reinvent the wheel. If nothing matches perfectly, however, don’t hesitate to create your own interface. To do that, you must create a new package dedicated to interfaces. Once you’ve finished the setup process for this package, you can add as many interfaces as you want.

Before we wrap things up, I will give you an additional challenge so that you can practice the concepts that were covered in this chapter.

## Topic challenge – closed-loop control

Here’s a challenge for you so that you can continue practicing creating nodes, publishers, and subscribers. We will start a new ROS 2 project and improve it throughout the following chapters, as we discover more concepts.

I encourage you to read the instructions and take the time to complete this challenge before you check the solution. Practicing is the key to effective learning.

I will not provide a full explanation of all the steps, just a few remarks on the important points. You can find the complete solution code on GitHub, for both Python and C++.

Your challenge is to write a controller for the **turtlesim** node. So far, we’ve just used simple and basic numbers to publish and subscribe to topics. With this, you can practice as if you were working on a real robot.

## Challenge

The goal is simple: we want to make the turtle move in a circle. On top of this, we also want to modify the velocity of the turtle, whether it’s on the right or left of the screen.

To get the _X_ coordinate of a turtle on the screen, you can subscribe to the **pose** topic for that turtle. Then, finding the middle of the screen is easy: the minimum _X_ value on the left is **0**, and the maximum _X_ value on the right is about **11**. We will assume that the _X_ coordinate for the middle of the screen is **5.5**.

You can then send a command velocity by publishing to the **cmd\_vel** topic for the turtle. To make the turtle move in a circle, you just have to publish constant values for the linear _X_ and angular _Z_ velocities. Use **1.0** for both velocities if the turtle is on the left (_X_ < 5.5), and **2.0** for both if the turtle is on the right.

Follow these steps to get started:

1.  Create a new package (let’s name it **turtle\_controller**). You can decide to create either a Python or C++ package. If you do both, make sure you give each a different name.
2.  Inside this package, create a new node named **turtle\_controller**.
3.  In the node’s constructor, add a publisher (command velocity) and a subscriber (pose).
4.  This is where it’s a bit different from before: instead of creating a timer and publishing from the timer callback, you can publish directly from the subscriber callback. The **turtlesim** node is constantly publishing on the **pose** topic. Publishing a command from the subscriber callback allows you to create some kind of closed-loop control. You can get the current _X_ coordinate and send a different velocity command, depending on where the turtle is.

To test your code, create an executable out of your code. Then, run **turtlesim** in one Terminal and your node in another. You should see the turtle drawing a circle, with a different velocity depending on where the turtle is.

## Solution

You can find the complete code (for both Python and C++) and package organization on GitHub.

Here are the most important steps for the Python node. The code starts with all the required import lines:

```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
```

Here, we import **Twist** from **geometry\_msgs** and **Pose** from **turtlesim**. You can find those interfaces by running **turtlesim\_node** and exploring topics with the **ros2 topic** and **ros2 interface** command-line tools.

Then, we create a class for our node, with a constructor:

```
class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)
```

As you can see, we just create a publisher and a subscriber. There’s no timer as we plan to use the publisher directly from the subscriber callback:

```
def callback_pose(self, pose: Pose):
    cmd = Twist()
    if pose.x  < 5.5:
        cmd.linear.x = 1.0
        cmd.angular.z = 1.0
    else:
        cmd.linear.x = 2.0
        cmd.angular.z = 2.0
    self.cmd_vel_pub_.publish(cmd)
```

This is the subscriber callback. Whenever we receive a new **Pose** message, we create a new command (a **Twist** message). Then, depending on the current _X_ coordinate of the turtle, we give different values for the velocity. Finally, we publish the new velocity command.

That’s it for this challenge. It can be a bit challenging to understand how to start, but in the end, you can see that there is not so much code to write. I encourage you to come back to this challenge in a few days and try again without looking at the solution. This way, you can check if you understood the concept of topics correctly.

## Summary

In this chapter, you worked on ROS 2 topics.

Topics allow nodes to communicate with each other using a publish/subscribe mechanism. Topics are made for unidirectional data streams and are anonymous.

You can write topic publishers and subscribers directly in your nodes by using **rclpy** for Python and **rclcpp** for C++.

To write a publisher, you must do the following:

1.  First, check what topic name and interface you must send. Import the interface into the code and create a publisher in the node’s constructor.
2.  To publish, you must create a message, fill in the different fields, and publish the message with your publisher.

You can potentially publish a message from anywhere in the code. A common structure is to add a timer and publish from the timer callback. If it makes sense, you can also publish from a subscriber callback directly.

To write a subscriber, you must do the following:

1.  As for the publisher, you need to know what name and interface to receive. Import the interface and create a subscriber in the node’s constructor.
2.  When creating the subscriber, you will need to specify a callback function. It’s in this callback function that you can receive and process incoming messages.

If you create a publisher or subscriber for a new topic and no interface matches your needs, you might need to create a custom interface. In this case, you must do the following:

1.  Create and configure a new package dedicated to interfaces for your robot or application.
2.  Add your topic interface inside the package and build the package.
3.  Now, you can use this custom interface in your publishers/subscribers, just like any other interface.

To try a publisher or a subscriber, simply build the package where the node is, source the environment, and run the node. You can then use the **ros2** command-line tools, as well as **rqt\_graph**, to introspect your application and solve potential issues.

After topics, the next logical step is to learn about ROS 2 services. This is what we will cover in the following chapter.

<table id="table001-13"><colgroup></colgroup><colgroup><col></colgroup><colgroup><col></colgroup><tbody><tr><td><h4>Unlock this book’s exclusive benefits now</h4><p>This book comes with additional benefits designed to elevate your learning experience.</p></td><td rowspan="2"><p><img alt="" width="246" height="238" src="attachments/9781835881408.png"></p><p lang="en-US" xml:lang="en-US"><a href="https://www.packtpub.com/unlock/9781835881408" target="_blank" rel="noopener noreferrer">https://www.packtpub.com/unlock/9781835881408</a></p></td></tr><tr><td><p><em>Note: Have your purchase invoice ready before </em><span><em>you begin.</em></span></p></td></tr></tbody></table>
