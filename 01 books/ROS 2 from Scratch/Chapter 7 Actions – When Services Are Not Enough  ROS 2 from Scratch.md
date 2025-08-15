---
created: 2025-08-14T12:59:04 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_07.xhtml#_idParaDest-188
author: 
---

# Chapter 7: Actions – When Services Are Not Enough | ROS 2 from Scratch

> ## Excerpt
> 7
Actions – When Services Are Not Enough
 In this chapter, we will explore the third communication type in ROS 2: actions. To understand actions, you need to have read the previous...

---
## Actions – When Services Are Not Enough

In this chapter, we will explore the third communication type in ROS 2: actions. To understand actions, you need to have read the previous chapters on nodes, topics, and services.

Before we begin, I want to alert you that this chapter covers more advanced material compared to what we encountered previously and what’s to come.

If you already have some level of expertise, this chapter will satisfy you as it will give you a full overview of all three ROS 2 communication types. However, if you’re just getting started with ROS with zero experience, it might be a bit too much for you right now. This is OK, and topics/services are more than enough to get started with ROS 2. You can skip this chapter (which is independent of future chapters) for now and continue with parameters and launch files. It might be a good idea to come back to it at a later stage after you’ve built more confidence by working on ROS 2 projects.

Throughout this chapter, you will understand why you need actions and how they work by going through an example that we will build step by step. Then, you will write the code to make two nodes communicate with each other. We will use the code in the **ch6** folder (in this book’s GitHub repository: [https://github.com/PacktPublishing/ROS-2-from-Scratch](https://github.com/PacktPublishing/ROS-2-from-Scratch)) as a starting point. You can find the final code in the **ch7** folder.

By the end of this chapter, you will be able to write an action server and client and take advantage of all action features, such as feedback and cancel mechanisms.

Even though topics and services are more than enough to get started, ROS 2 actions are important as they help you take your code to the next level and implement more complex behaviors in your robotics applications.

In this chapter, we will cover the following topics:

-   What is a ROS 2 action?
-   Creating a custom action interface
-   Writing an action server
-   Writing an action client
-   Taking advantage of all the action mechanisms
-   Additional tools to handle actions

## What is a ROS 2 action?

To understand ROS 2 actions, we need to understand why we need them. That’s what we will focus on first. After that, I will explain how actions work through a real-life example.

You quickly discovered actions in [_Chapter 3_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_03.xhtml#_idTextAnchor095) by running some existing nodes and command-line tools. The intuition you built there will help you better understand the concepts in this chapter.

Let’s dive in and see why and when actions could be needed in a ROS 2 application.

## Why actions?

So far, we’ve looked at two forms of communication in ROS 2: topics and services.

_Topics_ are used by nodes to send and receive messages. Publishers will publish data on a topic, and subscribers will subscribe to the topic to receive the data. Thus, topics are perfect for sending data streams in your application.

_Services_ are used for client/server interactions between nodes. The client sends a request to the server, after which the server executes or computes something and returns a response to the client.

So, that should be all, right? What else could we need?

In its early days, ROS started with only topics and services. However, ROS developers quickly realized that something was missing for some robotics applications. Let’s see that with an example.

Imagine that you have a mobile robot with two wheels. First, you would create a node that’s responsible for controlling the wheels. This node would also be able to receive commands, such as **Move to (x, y) coordinates**. Those commands would be transformed into a velocity to apply to the wheels. However, you would also like to be able to get notified when the robot has finished moving.

With what we know so far, ROS 2 services seem to be a good option. In this server node, you could implement a **/move\_robot** service that will receive coordinates from a client. Once the command is received, the controller starts to move the wheels. Then, when the robot has reached its destination, the server returns a response to the client.

To complete the communication, we must add a service client to another node. The client will send a request to the server with the (x,y) coordinates to reach. When the server returns the response, we know that the robot has finished moving—either successfully by reaching the destination or something prevented it and we get an error:

![[attachments/B22403_07_1.jpg]]

Figure 7.1 – Using a service to control a two-wheeled robot

What’s wrong with that?

Well, moving a physical part of a robot in space can take some time. It could be a fraction of a second in some cases, but also maybe a few seconds, or even a few minutes. The point is that the service execution could take a significant amount of time.

With that said, while the robot is moving, there are a few things you may want to do, and those things are missing when using services:

-   Since the execution is taking some time, it would be nice to get some feedback from the server. With a service, the client has no idea of what’s happening on the server side. So, the client is completely blind and needs to wait for the response to get some information.
-   How can you cancel the current execution? That would seem a reasonable feature to have. After you start the execution on the server side, the client may want to cancel it. For example, let’s say the client node is also monitoring the environment with a camera. If an obstacle is detected, the client could ask the server to stop the execution. With what we have for now, the client can’t do anything but wait for the server to finish the execution.
-   Here’s the last point for now, although we could find more: how could the server correctly handle multiple requests? Let’s say you have two or more clients, each one sending a different request. How can you possibly choose between those requests on the server? How can the server refuse to execute a request, or choose to replace a request with a new one, without finishing the first request? Or, in another scenario, how can the server handle multiple requests at the same time? As an analogy, when you download files on your computer, the computer isn’t stuck with just one file. It can download multiple files at the same time. You can even decide to cancel one download while the others are still running.

Coming back to our example, you can see that a simple service is not enough. For this use case, we need more functionalities. What we could do is implement additional services, such as one to cancel a request. We could also add a new topic to publish some feedback about where the robot is during the execution.

There’s good news—you don’t have to do this. All these problems are solved by ROS 2 actions. The feedback mechanism, cancel mechanism, and other functionalities are also implemented directly in actions.

To conclude, services are perfect for client/server communication, but only if the action/computation is quick to execute. If the execution could take some time, and you want additional features such as feedback or cancellation, then actions are what you need.

Now that you know why we need actions, it’s time to understand how they work.

## How do actions work?

Let’s use the previous example, this time using a ROS 2 action instead of a service. I will show you how actions work at a high level, with the different interactions between the client and the server. Later in this chapter, we will dive into the code and see the implementation details.

We will use two nodes: one containing an **Action** **client**, and the other containing an **Action** **server** (this is the one responsible for controlling the wheels).

To understand how actions work, let’s follow the execution flow for one action:

![[attachments/B22403_07_2.jpg]]

Figure 7.2 – Execution flow for a ROS 2 action

Here are the steps for this flow:

1.  The **Action client** will start the communication by sending a request to the **Action server**. For actions, the _request_ is named _goal_. Hence, we won’t talk about requests here, but about **goals**. Here, the goal can be a (x, y) coordinate to reach.
2.  The **Action server** receives the goal and decides to accept or reject it. The client immediately receives this response from the server. If the goal has been rejected, then the communication ends.
3.  If the goal is accepted, the server can start to process it and execute the corresponding action. With this example, the **Server node** will make the robot move.
4.  As soon as the client knows that the goal has been accepted, it will send a request to get the **Result** and wait for it (asynchronously, by registering a callback). For services, we talk about a _response_. For actions, this will be a _result_.
5.  When the server is done executing the goal (either successfully or not), it will send the **Result** to the client. With this example, the result could be the final reached (x, y) coordinates.
6.  The client receives the **Result**, after which communication ends.

This is how an action works, with a minimal set of functionalities. From the server side, a goal is received, accepted or rejected, then executed, and the result is returned. From the client side, a goal is sent, and if accepted, a request is sent and the result is received from the server.

On top of that, you can add extra functionalities, all of which are optional. Here are the additional mechanisms for actions:

![[attachments/B22403_07_3.jpg]]

Figure 7.3 – Action with all communication mechanisms

Let’s take a closer look:

-   **Feedback**: The server, while executing the goal, can send some feedback to the client. With this example, the feedback could be the current coordinates for the robot or even a completion rate. Thus, the client can know what’s happening during the goal’s execution.
-   **Cancel**: After the goal has been accepted by the server and the goal is being executed on the server side, the client can decide to cancel that goal. To do so, it will send a cancel request that must be approved by the server. If the cancel request is accepted, then the server will try to finish the execution. So, in this example, it could make the robot stop. In the end, the server will still return a result to the client, whether the goal was successful, failed, or canceled.
-   **Goal status**: This is not so important for you as it’s an internal mechanism for actions that you will not use directly in your code (I just added it here for completeness). Each goal will get a state machine, with states such as _accepted_, _executing_, and others. With each change of state for a goal, the server will notify the client.

With this, you have seen all possible communication mechanisms that can be implemented within actions.

Note that in the preceding figure, some communications are represented with a red line, while others are presented with a green line. Behind the scenes, actions just use topics and services. Even if an action is a ROS 2 communication on its own, the underlying code is using the two other communication types. Here, red lines represent services, and green lines represent topics.

Thus, within an action, you have three services (send goal, cancel goal, and receive result) and two topics (feedback and goal status). The good news is that you don’t have to create those topics and services yourself—they are already implemented in the action mechanism. All you have to do is use the action client and server functionalities from the ROS 2 libraries.

To create an action, you will need to give it a **name** (for example, **move\_robot**) so that the client knows where to send the goal. You will also need to use an **interface (goal,** **result, feedback)**.

One additional thing to note is that there can be only one action server. Just as for services, you can’t have two servers using the same name. On the other hand, you can have multiple action clients. Each client can also send multiple goals; that’s not a problem.

## Wrapping things up

On top of topics and services, you can use actions to make your nodes communicate with each other. Now, when should you use topics, services, or actions?

You should use _topics_ when you want to send data streams between nodes. With topics, there’s no response. For example, this can work for publishing sensor data or sending a stream of commands to another node if you don’t need any confirmation.

_Services_ are perfect when you want client/server communication, but also if the action to execute is very quick, such as a computation or a simple action, such as switching on an LED.

Finally, you will use _actions_ for anything that needs client/server communication and may take some time to execute, as well as when you also want to have mechanisms such as feedback and cancel.

Here are some important points about how actions work:

-   An action is defined by a name and an interface.
-   The name of an action follows the same rules as for topics and services. It must start with a letter and can be followed by other letters, numbers, underscores, tildes, and slashes. Also, as the action is _doing_ something, the best practice is to start the name with a verb.
-   The interface contains three things: a goal, a result, and feedback. Both the client and server must use the same interface.
-   An action server can only exist once, but you can send multiple goals from one or multiple action clients.
-   Action clients aren’t aware of the node containing the server. They just know they have to use the action name and interface to reach the server.

To implement an action communication, you will need to do the following at the very least:

-   Send a goal from the client to the server.
-   Accept (or not) the goal and execute it on the server.
-   Once the goal is finished, return a result from the server to the client.

The following are some optional features you can add:

-   Send some execution feedback from the server to the client while the goal is being executed.
-   Allow the client to send a cancel request to the server. If accepted, finish the goal execution on the server side.

To write action servers and clients in your code, you must use the action functionality from the **rclpy.action** and **rclcpp\_action** libraries.

At this point, we can start writing some code. If you’re still a bit confused, don’t worry—actions are quite complex to grasp initially. They contain lots of different mechanisms. Everything will make more sense as we create an action and write the client and server code.

Since we can’t test a client without a server, we will, as we did for services, start with the server side. To create a server, we need an action interface, so that will be our starting point.

## Creating a custom action interface

To create an action interface, we first need to clearly define what we need to achieve with the action. Then, we can add the interface to the **my\_robot\_interfaces** package (in this section, we will continue using the packages we created in the previous chapters).

## Defining the application and the interface we need

In the application that we will write in this chapter, the action server will be responsible for counting until a given number, with a delay between each count, so that we can simulate that the action takes some time and doesn’t return immediately. The client will have to send a number to the server so that the server can start to count. When the server finishes, it will send the result (last reached number) back to the client.

For example, let’s say the client sends the number 5 to the server, and there’s a delay of 0.5 seconds. The server will start to count from 0, up to 5, and wait 0.5 seconds between each iteration. When finishing, the server will return 5 if it could count until the end, or the last reached number if the execution finished sooner (the goal was canceled, or any other reason that could make the server stop the goal). In addition to that, we will add some feedback about the current count while the server is executing the goal.

Before we write any code, we need to know what interface to use for the action. From the previous paragraph, we can see that we need the following:

-   **Goal**: An integer for the target number and a float number for the delay
-   **Result**: An integer for the last reached number
-   **Feedback**: An integer for the current count

For topics and services, you must first check whether you can find an existing interface that matches your needs as there are already a lot of them you can use without having to create a new one.

For actions, you could try to do the same, but there aren’t as many existing action interfaces. Actions are a bit more complex than the other communication types, so you would need to find an interface that matches the goal, result, and feedback for your application exactly. The probability of that is very low as each action will be quite different. Thus, for actions, we won’t try to find existing interfaces and create a custom one directly.

## Creating a new action interface

The process of creating an action interface will be the same as for topic and service interfaces. We will follow a similar approach.

First, you need to create and configure a package dedicated to interfaces. We did that in [_Chapter 5_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_05.xhtml#_idTextAnchor214), in the _Creating a custom interface for a topic_ section, with the **my\_robot\_interfaces** package. You can reuse this package to add your action interfaces. If you don’t have it, go back and configure it first, then continue with the following steps.

In this package, we already have **msg** and **srv** folders for topic and service interfaces, respectively. We will add a third folder, named **action**, for—as you may have guessed—action interfaces:

```
$ cd ros2_ws/src/my_robot_interfaces/
$ mkdir action
```

In this new folder, you will place all the action interfaces specific to your robot or application.

Now, create a new file for your action. Here are the rules you must follow regarding the filename:

-   Use UpperCamelCase—for example, **CountUntil**.
-   Don’t write **Action** or **Interface** in the name as this will add unnecessary redundancy.
-   Use **.action** for the file extension.
-   As a best practice, use a verb in the interface’s name—for example, **NavigateToPosition**, **OpenDoor**, **PickObjectFromTable**, or **FetchDrinkFromFridge**. Actions, just like services, are about performing an action or computation (which can take some time), so by using a verb, you make it very clear what the action is doing.

Since we want to count until a given number, let’s call the interface **CountUntil**:

```
$ cd ~/ros2_ws/src/my_robot_interfaces/action/
$ touch CountUntil.action
```

You can write the definition for the action in this file. Since we have three different parts (goal, result, and feedback), we need to separate them. You must add three dashes (**\---**) between the goal and the result, and another three dashes between the result and the feedback.

Even if you don’t want to send any feedback, or if the result is empty, you still have to add the two separations with three dashes (**\---**). A very simple action definition with nothing in the result and feedback would look like this:

```
int64 goal_number
---
---
```

For the goal, result, and feedback, you can use the following:

-   Built-in types (**bool**, **byte**, **int64**, and so on).
-   Existing message interfaces. For example, the goal of the action could contain **geometry\_msgs/Twist**.

Note

You can’t include an action or service definition inside an action definition. You can only include messages (topic definition) inside the goal, result, or feedback. Those three parts can be seen as three independent messages.

Since we are creating a rather simple application, we will only use built-in types here:

```
# Goal
int64 target_number
float64 delay
---
# Result 
int64 reached_number 
---
# Feedback 
int64 current_number
```

As for topic and service interfaces, all fields inside the definition must follow the **snake\_case** convention (use underscores between words, all letters must be lowercase, and no spaces).

I’ve also added comments to specify which part is the goal, result, and feedback. You don’t need to do this—I only did it for your first action definition so that you don’t get confused. Often, people make mistakes regarding the order and put the feedback before the result, which can lead to hard-to-debug errors later. The order is goal, result, and then feedback.

Now that we’ve written our interface, we need to build it so that we can use it in our code. Go back to the **CMakeLists.txt** file of the **my\_robot\_interfaces** package. Since the package has already been configured, we just need to do one thing: add the relative path to the interface on a new line inside the **rosidl\_generate\_interfaces()** function. Don’t use any commas between the lines:

```
rosidl_generate_interfaces(${PROJECT_NAME}
&nbsp;&nbsp;"msg/HardwareStatus.msg"
&nbsp;&nbsp;"srv/ResetCounter.srv"
&nbsp;&nbsp;"srv/ActivateTurtle.srv"
<strong>&nbsp;&nbsp;"action/CountUntil.action"</strong>
)
```

After this, save all files and build the **my\_robot\_interfaces** package:

```
$ colcon build --packages-select my_robot_interfaces
```

Once built, source the environment. You should be able to find your new interface:

```
$ ros2 interface show my_robot_interfaces/action/CountUntil
# Action interface definition here
```

If you see the action definition, you know that your action interface has been successfully built, and you can now use it in your code. That’s what we will do, starting with the action server for our application.

## Writing an action server

In this section, you’ll write your first action server. In this server, we will be able to receive goals. When a goal is received, we will decide whether to accept or reject it. If it’s accepted, we will execute the goal. For this application, executing the goal means we will start to count from zero to the target number and wait for the provided delay between each iteration. Once the goal has been executed, we will return a result to the client.

That’s what we will implement in the code, starting with Python and then C++. In this section, we start only with the minimum functionalities for the action communication to work correctly. We will add the feedback and cancel mechanisms later. Since actions are a bit more complex than topics and services, let’s start simple and go step by step.

For a better learning experience, make sure you use the GitHub code while following along as I will not necessarily display all lines in this chapter, only the important ones. The code for this section is located in the **count\_until\_server\_minimal** file (with **.py** or **.cpp** appended at the end). We won’t use the **number\_publisher** and **number\_counter** nodes here.

Before we write any code for the server, we need to choose a name and interface for our action. Since we want to count until a given number, we will name the action **count\_until**, and we will use the **CountUntil** interface we’ve just created.

We now have everything we need to start writing the Python code.

## Writing a Python action server

You will need to write your action server inside a node. Create a new file named **count\_until\_server\_minimal.py** inside the **my\_py\_pkg** package (along with the other Python files). Make this file executable.

### Importing the interface and creating the server

Let’s start by setting up the action server.

First, we must import a bunch of libraries and classes that we will need in the code:

```
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
```

Unlike topics and services, the action server is not directly included in the **Node** class. So, we need to import the **ActionServer** class from **rclpy.action**.

After this, you must also import the interface for the action:

```
from my_robot_interfaces.action import CountUntil
```

When you import an interface from another package, make sure to add the dependency to **my\_robot\_interfaces** in the **package.xml** file of **my\_py\_pkg** (you should have already done this if you’ve been following along):

```
&lt;depend&gt;my_robot_interfaces&lt;/depend&gt;
```

Going back to the **count\_until\_server\_minimal.py** file, let’s create the action server in the node’s constructor (as stated in the introduction to this section, I’ll only display the important and relevant snippets; the full constructor code is available on GitHub):

```
self.count_until_server_ = ActionServer(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self,
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;CountUntil,
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;"count_until",
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;goal_callback=self.goal_callback,
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;execute_callback=self.execute_callback)
```

To create an action server with Python, you must use the **ActionServer** class we imported previously. Provide the following arguments:

-   **Action node**: The node to link the action server to. For topics and services, we started with **self.create…()**. Here, it’s a bit different: the object (**self**) is provided as the first argument.
-   **Action interface**: We use the **CountUntil** interface we’ve imported.
-   **Action name**: Since we’re writing the code for the server, we’re creating the action here. This is where you will choose the action name that all clients will have to use to send goals. As seen previously, we will use **count\_until**.
-   **Goal callback**: When a goal is received, it will be processed inside this callback.
-   **Execute callback**: If the goal has been accepted in the goal callback, then it will be processed in the execute callback. This is where you will execute the action.

We specified two callback methods when creating the action server. When the node spins, the action server will be in _waiting mode_. As soon as a goal is received, the node will trigger the goal callback, and then the execute callback if needed. Let’s implement those callbacks.

### Accepting or rejecting a goal

The action server can now receive goals. We need to decide whether to accept or reject them.

Let’s start writing the goal callback, which is the first method to be called whenever a goal is received by the server:

```
def goal_callback(self, goal_request: CountUntil.Goal):
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Received a goal")
&nbsp;&nbsp;&nbsp;&nbsp;if goal_request.target_number &lt;= 0:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().warn("Rejecting the goal, target number must be positive")
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;return GoalResponse.REJECT
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Accepting the goal")
&nbsp;&nbsp;&nbsp;&nbsp;return GoalResponse.ACCEPT
```

In this callback, we receive the goal that was sent by the client (it’s of the **CountUntil.Goal** type).

Note

An action interface contains a goal, a result, and feedback. You get one class for each message: **CountUntil.Goal**, **CountUntil.Result**, and **CountUntil.Feedback**. We will use all three in this chapter.

The best practice is to validate the data you receive whenever you write the code for a server. For this application, let’s say we want to only accept positive target numbers. If the number is negative, we reject the goal.

After validating the data, you need to return either **GoalResponse.ACCEPT** or **GoalResponse.REJECT** to accept or reject the goal, respectively. The client will be notified immediately of that decision. Then, if the goal is rejected, nothing more happens on the server side. If the goal is accepted, the execute callback will be triggered.

### Executing the goal

Let’s implement the execute callback. Here’s the beginning of the code:

```
def execute_callback(self, goal_handle: ServerGoalHandle):
&nbsp;&nbsp;&nbsp;&nbsp;target_number = goal_handle.request.target_number
&nbsp;&nbsp;&nbsp;&nbsp;delay = goal_handle.request.delay
&nbsp;&nbsp;&nbsp;&nbsp;result = CountUntil.Result()
&nbsp;&nbsp;&nbsp;&nbsp;counter = 0
```

In this callback, you get what’s called a goal handle, which is of the **ServerGoalHandle** type. I’ve made the argument type explicit so that we can get auto-completion with VS Code. This goal handle contains the goal information, but you can also use it to set the goal’s final state, which we will see in a minute.

The first thing you must typically do is extract the data from the goal. Here, we get the target number and delay that we will use when executing the action. Then, we initialize a few things: the result from the **CountUntil.Result** class, and a counter starting at **0**.

With this, we can start to execute the goal:

```
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Executing the goal")
&nbsp;&nbsp;&nbsp;&nbsp;for i in range (target_number):
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;counter += 1
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info(str(counter))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;time.sleep(delay)
```

This part of the code will be different every time as it depends entirely on your application. Here, we’re incrementing the counter until the target number is reached, with a delay between each iteration.

The point of using a delay here is just to make this method take more time so that we can simulate the behavior of an action. If we wanted to count as fast as possible, without any delay, we could have used a service since the action would finish almost immediately.

Once the execution is finished, we need to do two things—set a final state for the goal and return a result to the client:

```
&nbsp;&nbsp;&nbsp;&nbsp;goal_handle.succeed()
&nbsp;&nbsp;&nbsp;&nbsp;result.reached_number = counter
&nbsp;&nbsp;&nbsp;&nbsp;return result
```

During the execution of the action, the goal is in the _executing_ state. When finishing the execution, you need to make it transition into a final state.

In this case, since everything went smoothly and we didn’t expect any problems during the execution, we set the goal to _succeeded_ by using the **succeed()** method on the goal handle. If, for example, your action was responsible for moving the wheels of a robot, and if the communication with the wheels is lost during the execution, you would stop the action and set the goal to _aborted_ with the **abort()** method. The last possible state is _canceled_, which we will see a bit later in this chapter.

We’ve now written the minimal code for the action server to work properly. Before we write an action client, let’s switch to C++. If you only want to follow the Python explanations, then go ahead and skip the next section.

## Writing a C++ action server

The code logic for C++ actions is very similar to Python, but there are quite a few specificities about the syntax. We will focus mostly on those differences. Also, as the code starts to become quite large, I will not necessarily display the full code, only the important parts for comprehension. Make sure you take a look at this book’s GitHub repository to see the full code.

### Importing the interface and creating the server

Let’s start by setting up the action server. First, create a new file named **count\_until\_server\_minimal.cpp** in the **src** directory of your **my\_cpp\_pkg** package.

Open the file and start by adding the necessary includes:

```
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"
```

As you can see, the action library is not a sub-library of **rclcpp**—it’s a completely independent one from a different package: **rclcpp\_action**.

For each new package we use, we need to add the dependency to the **package.xml** file of the **my\_cpp\_pkg** package:

```
&lt;depend&gt;my_robot_interfaces&lt;/depend&gt;
&lt;depend&gt;rclcpp_action&lt;/depend&gt;
```

You will also need to specify those dependencies in the **CMakeLists.txt** file:

```
find_package(my_robot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
```

Finally, when you create your executable, don’t forget to add both dependencies to the **ament\_target\_dependencies()** function:

```
add_executable(count_until_server src/count_until_server_minimal.cpp)
ament_target_dependencies(count_until_server rclcpp rclcpp_action my_robot_interfaces)
```

Back to the **count\_until\_server\_minimal.cpp** file, we add a few **using** lines to simplify the code (you can find those lines at the top of the file, under the **#include** lines). After that, you can add an action server to your class as a private attribute:

```
rclcpp_action::Server&lt;CountUntil&gt;::SharedPtr count_until_server_;
```

Once again, we’re going to use a shared pointer to keep the object.

Then, in the constructor, you can create the action server:

```
count_until_server_ = rclcpp_action::create_server&lt;CountUntil&gt;(
&nbsp;&nbsp;&nbsp;&nbsp;this,
&nbsp;&nbsp;&nbsp;&nbsp;"count_until",
&nbsp;&nbsp;&nbsp;&nbsp;std::bind(&amp;CountUntilServerNode::goalCallback, this, _1, _2),
&nbsp;&nbsp;&nbsp;&nbsp;std::bind(&amp;CountUntilServerNode::cancelCallback, this, _1),
&nbsp;&nbsp;&nbsp;&nbsp;std::bind(&amp;CountUntilServerNode::executeCallback, this, _1)
);
```

For actions, the C++ syntax is stricter than Python. On top of the action interface, object to link to, and action name, you have to provide three callbacks (even if you don’t want to use them all):

-   **Goal callback**: To accept or reject incoming goals.
-   **Cancel callback**: To receive cancel requests.
-   **Execute callback**: This is called the _handle accepted callback_ in C++, but I named it _execute callback_ to make the code similar to the Python one. In this callback, we execute goals that have been accepted.

Note

I’ve designed this chapter so that we write minimal code first, and then add the extra optional features. However, the C++ **create\_server()** method will not work if you don’t provide a cancel callback. Thus, what we will do for now is add this callback but not fully implement the cancel mechanism; we’ll do that later.

At this point, we need to implement the three callback methods.

### Implementing the callbacks

The arguments inside the callbacks can be quite long to write. That’s why I suggest simplifying the code with **using** lines at the beginning, as well as double-checking everything as it’s easy to make mistakes.

Here’s the beginning of the goal callback method:

```
rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID &amp;uuid, std::shared_ptr&lt;const CountUntil::Goal&gt; goal)
```

Here, you get a unique identifier for the goal and the goal itself (to be precise, this is a **const** shared pointer to the goal). In the callback, we validate the goal and then accept or reject it. For example, to accept the goal, you would return the following:

```
return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
```

The next callback method is the cancel callback, in which you can decide whether to accept or reject an incoming cancel request. As I will explain the cancel mechanism later in this chapter, I will skip this part now—you just have to write the callback so that the code can compile.

The most important callback here is the execute callback. In this method, we receive a goal handle (**const std::shared\_ptr<CountUntilGoalHandle> goal\_handle**). The first thing we must do is extract the data from the goal and initialize a few things:

```
int target_number = goal_handle-&gt;get_goal()-&gt;target_number;
double delay = goal_handle-&gt;get_goal()-&gt;delay;
auto result = std::make_shared&lt;CountUntil::Result&gt;();
int counter = 0;
rclcpp::Rate loop_rate(1.0/delay);
```

You’ve probably started to get used to seeing shared pointers everywhere, and here is no exception. We don’t create a result object, but a shared pointer to a result object.

Then, to handle the waiting time between each count iteration, we use a **rclcpp::Rate** object. This is a bit different from what we did with Python. In this rate object, we have to pass the rate—that is, the frequency we want for the loop. For example, if the delay is 0.5 seconds, the frequency would be 2.0 Hz. We can now execute the action:

```
RCLCPP_INFO(this-&gt;get_logger(), "Executing the goal");
for (int i = 0; i &lt; target_number; i++) {
&nbsp;&nbsp;&nbsp;&nbsp;counter++;
&nbsp;&nbsp;&nbsp;&nbsp;RCLCPP_INFO(this-&gt;get_logger(), "%d", counter);
&nbsp;&nbsp;&nbsp;&nbsp;loop_rate.sleep();
}
```

Here, we use the **sleep()** function of the rate object to pause the execution.

Finally, once the **for** loop ends, we can finish the execution:

```
result-&gt;reached_number = counter;
goal_handle-&gt;succeed(result);
```

In Python, we would set the goal’s final state first, and then return the result. In C++, we don’t return anything (note the **void** return type). We send the result at the same time as setting the goal state.

Note

Writing C++ code with actions starts to be quite complex, especially if you don’t have much C++ experience. If you feel completely lost, maybe either continue with Python only or, as mentioned previously, skip this chapter for now and come back to it later.

That’s it for the C++ action server. We can now write the client node and try the communication.

## Writing an action client

We now have the minimal code required for the server to receive a goal, accept it, execute it, and return a result. At this point, we can write the client side of the communication.

The action client will send a goal to the server. It will then register a callback to find out whether the goal was accepted or rejected. If the goal is accepted, the client will register yet another callback to get the final result. That’s what we’re going to implement now—first with Python, then with C++.

Where should you write the action client? In your own ROS 2 applications, you could add an action client to any node. As an example, let’s say you have a node that monitors the battery level of a mobile robot. This node could already have some publishers, subscribers, services, and so on. On top of all that, you can add an action client that will send a goal to another node (such as the server node, which controls the wheels of the robot) when the battery runs low.

For this chapter, and to keep things simple, we will create a new node, just for the action client. You can then use this code as a template for adding an action client anywhere you want. You can find the code for this section in **count\_until\_client\_minimal** (**.py** or **.cpp**).

Let’s start with the Python action client.

## Writing a Python action client

Create a new Python file named **count\_until\_client\_minimal.py** in the **my\_py\_pkg** package. Make this file executable.

## Creating an action client

Let’s start by setting up the action client. First, add the dependencies we will need:

```
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import CountUntil
```

As for the action server, we don’t get the action client directly from the **Node** class. Instead, we have to import **ActionClient** from **rclpy.action**.

We must also import the action interface, which should be the same as for the server. If we import this interface, we also need to add a dependency to the **package.xml** file. However, we have already done that, so there’s no need to add anything else.

Then, in the node’s constructor, we create an action client:

```
self.count_until_client_ = ActionClient(
self, CountUntil, "count_until")
```

We use the **ActionClient** class directly, and we pass three arguments: the object to bind to (**self**), the action interface, and the action name. Double-check that the name is the same as on the server side.

Then, to send a goal to the server, we add a new method:

```
def send_goal(self, target_number, delay):
&nbsp;&nbsp;&nbsp;&nbsp;self.count_until_client_.wait_for_server()
&nbsp;&nbsp;&nbsp;&nbsp;goal = CountUntil.Goal()
&nbsp;&nbsp;&nbsp;&nbsp;goal.target_number = target_number
&nbsp;&nbsp;&nbsp;&nbsp;goal.delay = delay
&nbsp;&nbsp;&nbsp;&nbsp;self.count_until_client_.send_goal_async(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;goal).add_done_callback(self.goal_response_callback)
```

Here are the steps for sending a goal from the client to the server:

1.  You can wait for the server with **wait\_for\_server()**. If you send a goal when the server isn’t up and running, you will get an error, so ensure it’s ready before you do anything. I didn’t provide a timeout here, so it will wait indefinitely. You could add a timeout and do something similar to what we did in [_Chapter 6_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_06.xhtml#_idTextAnchor285), in the _Writing a service_ _client_ section.
2.  Create a goal object from the interface: **Interface.Goal()**.
3.  Fill in the goal fields. Any field you omit will get a default value (**0** for numbers, **""** for strings).
4.  Send the goal with **send\_goal\_async()**. This will return a Python **Future** object.
5.  Register a callback for the goal’s response so that you know it’s been accepted or rejected.

Note that just as for services, we make an asynchronous call with **send\_goal\_async()**. This way, the method will return and we won’t block the execution. If we were to block the execution, we would also block the spin, and thus we would never get any response.

### Implementing the callbacks

So far, we’ve sent a goal with the action client and registered a callback, **goal\_response\_callback()**. Let’s implement this method:

```
def goal_response_callback(self, future):
&nbsp;&nbsp;&nbsp;&nbsp;self.goal_handle_: ClientGoalHandle = future.result()
&nbsp;&nbsp;&nbsp;&nbsp;if self.goal_handle_.accepted:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Goal got accepted")
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.goal_handle_.get_result_async().add_done_callback(
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.goal_result_callback)
```

```
&nbsp;&nbsp;&nbsp;&nbsp;else:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Goal got rejected")
```

In this callback, we get a **ClientGoalHandle** object from the result of the Python **Future** object. From this goal handle, we can find out whether the goal was accepted or not.

Please note that you won’t get the final result in this goal response callback. Here, we only get to know whether the server accepted the goal or not. If the goal is accepted, we know that the server will start executing it and return a result at some point.

Then, in the client, we can register another callback for the goal result:

```
def goal_result_callback(self, future):
&nbsp;&nbsp;&nbsp;&nbsp;status = future.result().status
&nbsp;&nbsp;&nbsp;&nbsp;result = future.result().result
&nbsp;&nbsp;&nbsp;&nbsp;if status == GoalStatus.STATUS_SUCCEEDED:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Success")
&nbsp;&nbsp;&nbsp;&nbsp;elif status == GoalStatus.STATUS_ABORTED:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().error("Aborted")
&nbsp;&nbsp;&nbsp;&nbsp;elif status == GoalStatus.STATUS_CANCELED:
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().warn("Canceled")
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Result: " + str(result.reached_number))
```

In this callback, we get the goal’s final state and result after the server has finished executing the goal.

You can do anything you want with this result—here, we simply print it. As you can see, we will receive any of those three final states for the goal: **STATUS\_SUCCEEDED**, **STATUS\_ABORTED**, and **STATUS\_CANCELED**.

Finally, let’s not forget to call the **send\_goal()** method. We will do this in the **main()** function, just after we initialize the node, and before we make the node spin:

```
node = CountUntilClientNode()
node.send_goal(5, 0.5)
rclpy.spin(node)
```

This will ask the server to count until **5** and wait **0.5** seconds between each count.

### Trying the communication

We can now try the communication between the client and server.

Create an executable (in **setup.py**) for both the client and server nodes. Build the package and source the environment.

Then, start the server node and the client node in two different Terminals. You should see some logs in both Terminals as the communication progresses. In the end, you will get something like this for the server:

```
$ ros2 run my_py_pkg count_until_server
[count_until_server]: Action server has been started.
[count_until_server]: Received a goal
[count_until_server]: Accepting the goal
[count_until_server]: Executing the goal
[count_until_server]: 1
...
[count_until_server]: 5
```

For the client:

```
$ ros2 run my_py_pkg count_until_client
[count_until_client]: Goal got accepted
[count_until_client]: Success
[count_until_client]: Result: 5
```

You can see the flow of execution with the timestamp in each log. Here, we tested the case when the target number was positive—and thus, the goal was accepted. If you want, you can also test the case when the target number is negative; you should see the goal being rejected and not executed.

Now, let’s learn how to write an action client with C++.

## Writing a C++ action client

For the C++ code, I will focus on the few important points to notice in the **count\_until\_client\_minimal.cpp** file.

First, we have all the includes and **using** lines. Those are almost the same as for the C++ action server. However, for the goal handle, we get **ClientGoalHandle** (this was **ServerGoalHandle** in the server code):

```
using CountUntilGoalHandle = rclcpp_action::<strong>ClientGoalHandle</strong>&lt;CountUntil&gt;;
```

To create an action client, we declare the client as a private attribute of the class:

```
rclcpp_action::Client&lt;CountUntil&gt;::SharedPtr count_until_client_;
```

Then, we initialize the client in the constructor:

```
count_until_client_ = rclcpp_action::create_client&lt;CountUntil&gt;(this, "count_until");
```

As you can see (but that shouldn’t be a surprise anymore), we store a shared pointer to the action client. When initializing it, we provide the action interface, the object to bind to (**this**), and the action name, which should be the same as the one defined in the server code.

At this point, we can create a **sendGoal()** method to send a goal to the server. This method follows the same steps as for the Python client. We wait for the server, then create a goal, fill in the goal fields, send the goal, and register a callback. However, there is a big difference in how we handle the callbacks:

```
auto options = rclcpp_action::Client&lt;CountUntil&gt;::SendGoalOptions();
options.goal_response_callback = std::bind(
&nbsp;&nbsp;&nbsp;&nbsp;&amp;CountUntilClientNode::goalResponseCallback, this, _1);
options.result_callback = std::bind(
&nbsp;&nbsp;&nbsp;&nbsp;&amp;CountUntilClientNode::goalResultCallback, this, _1);
count_until_client_-&gt;async_send_goal(goal, options);
```

In Python, we would chain the callbacks after sending the goal. In C++, you first need to create a **SendGoalOptions** object. In this object, you can register the different callback methods for your client. Here, we register the response and the result callback. Then, you must pass this object to the **async\_send\_goal()** method. This will register all the callbacks for when the node is spinning.

Now that we’ve registered two callbacks, we need to implement them.

In the goal response callback, to check if the goal was accepted or rejected, we can simply write the following:

```
if (!goal_handle) {
```

If this returns **false**, we know the goal was rejected. If it returns **true**, there’s no need to do anything else in this callback as the result callback was already registered with the **SendGoalOptions** object.

In the result callback, we get the goal’s final state with **result.code**. We can then compare it with the different codes in **rclcpp\_action::ResultCode**, which are **SUCCEEDED**, **ABORTED**, and **CANCELED**. To get access to the actual result, we write **result.result**. This will be a shared pointer to the result object.

Finally, let’s not forget to call the **sendGoal()** method in the **main()** function:

```
auto node = std::make_shared&lt;CountUntilClientNode&gt;();
node-&gt;sendGoal(5, 0.5);
rclcpp::spin(node);
```

That’s about it for the C++ action client. After writing both the client and server, create an executable for both (in **CMakeLists.txt**); then, build, source, and run the two nodes. You can even try running the Python client with the C++ server, or any other combination.

Now that both the client and server are running correctly, we can add the extra functionalities we get with actions: feedback and cancel.

## Taking advantage of all the action mechanisms

The reason I’m talking about feedback and cancel mechanisms now and didn’t previously is to try not to overwhelm you with too much code at once. I know that actions are more complex than everything you’ve seen before with ROS 2. The minimal code alone is already quite long and contains lots of small details you must pay attention to.

Also, as explained in the first part of this chapter, the feedback and cancel mechanisms are optional. You can create a fully working client/server communication without them.

We’re now going to improve the minimal code and add a few more functionalities so that we can take full advantage of ROS 2 actions. Here’s what you can do to prepare the files for this section:

1.  Make a copy of the files containing **\_minimal**.
2.  Rename those new files by removing the **\_minimal**.

For example, you can make a copy of **count\_until\_client\_minimal.py** (we won’t modify this file anymore) and rename the copy **count\_until\_client.py** (this is where we will add more code). You can find the same organization in this book’s GitHub repository.

So, let’s explore the feedback and cancel mechanisms, starting with feedback, which is the easiest one.

## Adding the feedback mechanism

When we wrote the action interface, we had to define three things: goal, result, and feedback. So far, we’ve only used the goal and result. The feedback is optional, and you could choose to leave it empty in the action definition. In this case, there’s nothing else to do.

Since we’ve defined feedback in **CountUntil.action** (**int64 current\_number**), let’s use it in our code so that we can make the server send feedback every time it increases the counter. The action client will be able to receive this feedback inside a callback.

### Feedback with Python

Let’s start with the action server. There are just a few lines to add so that we can publish the feedback.

Open **count\_until\_server.py**. In the **execute\_callback()** method, at the same time as creating a result object, create a feedback object:

```
feedback = CountUntil.Feedback()
```

Now, when you execute the goal, you have to do the following:

```
feedback.current_number = counter
goal_handle.publish_feedback(feedback)
```

We must fill in the different fields of the feedback object and then send the feedback to the client with the **publish\_feedback()** method from the goal handle.

That’s all there is to it for the server side. Now, let’s write the code to receive the feedback.

Open the **count\_until\_client.py** file and modify the line where you send the goal with **send\_goal\_async()**:

```
self.count_until_client_.send_goal_async(
&nbsp;&nbsp;&nbsp;&nbsp;goal, <strong>feedback_callback=self.goal_feedback_callback</strong>). \
&nbsp;&nbsp;&nbsp;&nbsp;add_done_callback(self.goal_response_callback)
```

To get the feedback with a Python action client, you must register a callback function when you send the goal. Here’s the implementation for this callback:

```
def goal_feedback_callback(self, feedback_msg):
&nbsp;&nbsp;&nbsp;number = feedback_msg.feedback.current_number
&nbsp;&nbsp;&nbsp;self.get_logger().info("Got feedback: " + str(number))
```

With this, we get a feedback message and can access each field of that message. You can do anything you want with this feedback. For example, if your action client is asking for a robot to move to certain (x, y) coordinates, you might receive feedback on the current progress of the robot. From this, you could take any appropriate measure: cancel the goal (see the next section), send a new goal, and so on.

That’s it regarding feedback. You can build your package again, source it, and run the two nodes. Here’s what you will see on the client side:

```
$ ros2 run my_py_pkg count_until_client
[count_until_client]: Goal got accepted
[count_until_client]: Got feedback: 1
```

It will continue as follows:

```
[count_until_client]: Got feedback: 5
[count_until_client]: Success
[count_until_client]: Result: 5
```

With this feedback, the client isn’t in the dark anymore. It can get to know what’s happening between sending the goal and receiving the result.

### Feedback with C++

The behavior for adding the feedback for the action server in **count\_until\_server.cpp** is the same as it is for Python.

First, you must create a feedback object in the execute callback:

```
auto result = std::make_shared&lt;CountUntil::Result&gt;();
```

The only difference is that we use a shared pointer here.

Then, you must publish the feedback:

```
feedback-&gt;current_number = counter;
goal_handle-&gt;publish_feedback(feedback);
```

On the client side, the way a callback is registered is a bit different. Open **count\_until\_client.cpp** and add the following line to the **sendGoal()** method:

```
options.feedback_callback = std::bind(
&nbsp;&nbsp;&nbsp;&nbsp;&amp;CountUntilClientNode::goalFeedbackCallback, this, _1, _2);
```

For a C++ action, we register all callbacks in the **SendGoalOptions** object that we pass to the **async\_send\_goal()** method.

Then, you can implement the callback:

```
void goalFeedbackCallback(const CountUntilGoalHandle::SharedPtr &amp;goal_handle, const std::shared_ptr&lt;const CountUntil::Feedback&gt; feedback)
{
&nbsp;&nbsp;&nbsp;(void)goal_handle;
&nbsp;&nbsp;&nbsp;int number = feedback-&gt;current_number;
&nbsp;&nbsp;&nbsp;RCLCPP_INFO(this-&gt;get_logger(), "Got feedback: %d", number);
}
```

Here, we receive both the goal handle and the feedback (as **const** shared pointers).

Note

As you can see, whenever there’s an argument we don’t use in a function, I write **(void)**, followed by the argument. This is a way to prevent getting _unused parameter_ warnings when compiling with **colcon build**. As a best practice, you should address all errors and warnings in your code when developing a ROS 2 application. If you don’t do this, you will end up with lots of ignored warnings, and you could miss the important ones, leading to hard-to-debug issues in the future.

Now that the code is complete, you can compile the package and run the client and server nodes in two different Terminals. You should see a similar output to what we had for Python.

Implementing the feedback mechanism is relatively easy. Now, let’s learn how to cancel a goal. This will be more complex and require the use of more advanced ROS 2 concepts.

## Adding the cancel mechanism

After sending a goal, the client can decide to ask the server to cancel it. The server will receive this request and accept (or not) to cancel the goal. If the cancel request is accepted, the server will take any appropriate action to cancel the execution of the goal. In the end, the server will still send a result to the client.

What do we need to do in the code? In the server node, we will add another callback so that we can receive cancel requests and decide to accept or reject them. Then, in the execute callback, we will be able to check whether the goal should be canceled; if so, we will terminate the execution sooner.

However, if we just do this, it’s not going to work and the cancel requests will never be received. Why is that? Let’s explore this question now.

Note

This section introduces a few concepts that are outside the scope of this (beginner) book. I will talk about them briefly without going into full detail. If you’d like to understand these in more depth, feel free to explore the advanced concepts by yourself (you will find additional resources in [_Chapter 14_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_14.xhtml#_idTextAnchor669)_)._ You can see this section as a going further with actions section.

### Understanding the problem with cancel and spin

We will only focus on the server side here as this is where the issue will occur. I will explain what the issue is so that we can implement the solution later.

So, when you start the action server, three callbacks will be registered: a goal callback, a cancel callback, and an execute callback.

With our current code, when the server receives a goal, here’s what happens:

1.  The goal is received by the goal callback and is accepted or rejected.
2.  If accepted, we execute the goal in the execute callback. Something crucial to note is that while we execute the goal with the **for** loop, the thread is blocked.
3.  Once the goal is executed, we return the result and exit from the execute callback.

The problem is with _Step 2_. Since we’re blocking the execution, we’re blocking the spin mechanism.

When you make a node spin, what’s happening? As mentioned previously, the node will be kept alive and all callbacks can be processed. However, the spin is working in a single thread. This means that if you have one callback taking 5 seconds to execute, it will block the following callbacks for 5 seconds.

We never had any issues before because all the callbacks we wrote were very quick to execute. However, with the execute callback for an action, we’re in a situation where the execution could take quite some time, and thus block all the other callbacks.

That’s quite the problem. How can you ask to cancel a goal if the cancel request is only received after the goal’s execution has finished?

To solve this problem, we have two possible solutions:

-   **The classic programming way**: We could create a new thread in the execute callback. The callback can then exit while the goal is processed in the background. The spin continues, and thus, other callbacks can be called.
-   **The ROS 2 way**: We can use a multi-threaded executor, which means that our spin mechanism will work not in a single thread, but in multiple threads. Thus, if one callback is blocking, you can still execute other callbacks—including the cancel callback.

Since we want to follow ROS 2 principles to stay consistent with other developers, we’re going to follow the ROS 2 way and solve that issue with a multi-threaded executor.

Note

I’m not going to go into more detail about single and multi-threaded executors here. I’m using them now so that we can implement the cancel mechanism correctly. Executors can be a great topic to explore after reading this book.

The process for the cancel mechanism in the server code will be the same for Python and C++:

1.  Register a callback to handle cancel requests.
2.  Cancel the goal in the execute callback.
3.  Make the node spin with a multi-threaded executor.

### Canceling with Python

We will start with the server code, which can be found in **count\_until\_server.py**.

First, let’s register a callback to receive cancel requests:

```
ActionServer(
&nbsp;&nbsp;&nbsp;&nbsp;…
&nbsp;&nbsp;&nbsp;&nbsp;cancel_callback=self.cancel_callback,
&nbsp;&nbsp;&nbsp;&nbsp;…)
```

Here’s the callback’s implementation:

```
def cancel_callback(self, goal_handle: ServerGoalHandle):
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Received a cancel request")
&nbsp;&nbsp;&nbsp;&nbsp;return CancelResponse.ACCEPT
```

In this callback, you receive a goal handle corresponding to the goal the client wants to cancel. You can then create any kind of condition to decide whether the goal should be canceled or not. To accept, you must return **CancelResponse.ACCEPT**; to reject, you must return **CancelResponse.REJECT**. With this example, I kept things simple and we just accepted the cancel request without implementing any other checks.

Now, if the cancel request has been accepted, we need to do something about it. In the execute callback, while we’re executing the goal (inside the **for** loop), add the following code:

```
if goal_handle.is_cancel_requested:
&nbsp;&nbsp;&nbsp;&nbsp;self.get_logger().info("Canceling goal")
&nbsp;&nbsp;&nbsp;&nbsp;goal_handle.canceled()
&nbsp;&nbsp;&nbsp;&nbsp;result.reached_number = counter
&nbsp;&nbsp;&nbsp;&nbsp;return result
```

When we accept a cancel request, an **is\_cancel\_requested** flag in the goal handle will be set to **True**. Now, in the execute callback, we simply need to check this flag.

What we do in the code is stop the current execution. If, for example, your action server controls the wheels of a robot, you could interpret **cancel** as “decelerate and stop moving,” “step on the side so we don’t block the main way,” or even “go back to base.” The way you handle the behavior for the cancellation depends on each application. Here, we just stop counting.

In the execute callback, you need to set the goal’s final state and return a result, even if you cancel the goal. Thus, we use the **canceled()** method to set the state, and we return a result that contains the last reached number. If the client asks the server to count to 10 and then cancels the goal when the counter is at 7, the result will contain 7.

That’s it for the cancel mechanism. However, to make things work, as we’ve seen previously, we need to use a multi-threaded executor.

First, you’ll need to import the following:

```
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
```

When using multi-threaded executors, we also need to use _callback groups_. Here, **ReentrantCallbackGroup** will allow all callbacks to be executed in parallel. This means that you can have several goal, cancel, and execute callbacks running at the same time for one action server.

When you create the action server, add a **callback\_group** argument:

```
ActionServer(
&nbsp;&nbsp;&nbsp;&nbsp;…
&nbsp;&nbsp;&nbsp;&nbsp;callback_group=ReentrantCallbackGroup())
```

Finally, modify the line to make the node spin in the **main()** function:

```
rclpy.spin(node, MultiThreadedExecutor())
```

That’s all there is to it. It’s just a few lines of code, but adding this requires a good understanding of ROS 2 and its underlying mechanisms.

Let’s write the code for the client so that we can send a cancel request for a goal that’s being executed. In **count\_until\_client.py**, add a method to cancel a goal:

```
def cancel_goal(self):
&nbsp;&nbsp;&nbsp;self.get_logger().info("Send a cancel goal request")
&nbsp;&nbsp;&nbsp;self.goal_handle_.cancel_goal_async()
```

Here, we’re using the goal handle that we saved in the goal response callback (**self.goal\_handle\_: ClientGoalHandle = future.result()**). From this goal handle object, we have access to a **cancel\_goal\_async()** method.

So, where do we cancel the goal? This can be done from anywhere: from the feedback callback, an independent subscriber callback, and so on. It will depend on your application.

To make a quick test, let’s arbitrarily decide that we want to cancel the goal if the **current\_number** field from the feedback is greater than or equal to 2. It doesn’t make any sense (why would we ask to count until 5, only to cancel if the number reaches 2?), but it’s a quick way to test the cancel mechanism.

In the goal feedback callback, add the following code:

```
if number &gt;= 2:
&nbsp;&nbsp;&nbsp;&nbsp;self.cancel_goal()
```

Then, build the package, source it, and run both the server and client. Here’s the log for the client:

```
[count_until_client]: Goal got accepted
[count_until_client]: Got feedback: 1
[count_until_client]: Got feedback: 2
[count_until_client]: Send a cancel goal request
[count_until_client]: Canceled
[count_until_client]: Result: 2
```

For the server, you will see this:

```
[count_until_server]: Executing the goal
[count_until_server]: 1
[count_until_server]: 2
[count_until_server]: Received a cancel request
[count_until_server]: Canceling goal
```

With this, we used all the mechanisms available for actions. Now, you can comment the lines to cancel the goal from the feedback callback—this was just for testing purposes.

### Canceling with C++

In the server code (**count\_until\_server.cpp**), we added a cancel callback when we created the action server. This was mandatory so that the code could compile.

In this callback, we just accept the cancel request:

```
return rclcpp_action::CancelResponse::ACCEPT;
```

Then, to handle the cancellation of the goal in the execute callback, add the following code to the **for** loop:

```
if (goal_handle-&gt;is_canceling()) {
&nbsp;&nbsp;&nbsp;&nbsp;RCLCPP_INFO(this-&gt;get_logger(), "Canceling goal");
&nbsp;&nbsp;&nbsp;&nbsp;result-&gt;reached_number = counter;
&nbsp;&nbsp;&nbsp;&nbsp;goal_handle-&gt;canceled(result);
&nbsp;&nbsp;&nbsp;&nbsp;return;
}
```

In C++, we check the **is\_canceling()** method inside the goal handle. If it returns **true**, this means that a cancel request for this goal has been accepted, and we need to do something about it.

We set the goal’s final state and result with **canceled()**, and we exit from the execute callback.

That’s it for the cancel mechanism, but now we need to make the node spin with a multi-threaded executor.

In the **main()** function, we must replace the **rclcpp::spin(node);** line with the following code:

```
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

Here, we create an executor, add the node, and make the executor spin. Then, as we did for Python, inside the node, we need to add a callback group. We can declare one as a private attribute:

```
rclcpp::CallbackGroup::SharedPtr cb_group_;
```

Finally, we modify the code in the node’s constructor to give a reentrant callback group to the action server, so that all callbacks can be executed in parallel:

```
cb_group_ = this-&gt;create_callback_group(
&nbsp;&nbsp;&nbsp;&nbsp;rclcpp::CallbackGroupType::Reentrant);
count_until_server_ = rclcpp_action::create_server&lt;CountUntil&gt;(
&nbsp;&nbsp;&nbsp;&nbsp;…
```

```
&nbsp;&nbsp;&nbsp;&nbsp;rcl_action_server_get_default_options(),
&nbsp;&nbsp;&nbsp;&nbsp;cb_group_
);
```

We also need to add **rcl\_action\_server\_get\_default\_options()** after the callbacks and before the callback group; otherwise, the compiler will complain about not finding an overload for the **create\_server()** function.

Now that we’ve finished writing the server code, let’s send a cancel request from the client. In **count\_until\_client.cpp**, add a **cancelGoal()** method:

```
void cancelGoal()
{
&nbsp;&nbsp;&nbsp;&nbsp;RCLCPP_INFO(this-&gt;get_logger(), "Send a cancel goal request");
&nbsp;&nbsp;&nbsp;&nbsp;count_until_client_-&gt;async_cancel_all_goals();
}
```

In C++, we cancel goals from the action client, not from the goal handle. To make things simpler here, we’re canceling all goals that could have been sent by this client.

To test the cancel mechanism, we add those lines to the feedback callback:

```
if (number &gt;= 2) {
&nbsp;&nbsp;&nbsp;&nbsp;cancelGoal();
}
```

Once you’ve completed the code, run your C++ action client and server nodes. You can also try any combination of Python and C++ nodes; they should behave the same way. Once you’ve tested your code, comment the lines to cancel the goal from the feedback callback.

Let’s finish this chapter with a few more command-line tools that will help you when you’re developing applications with actions.

## Additional tools to handle actions

Since actions are part of the core ROS 2 functionalities, they also get their own command-line tool: **ros2 action**.

In this section, we’ll learn how to introspect actions, send a goal from the Terminal, and change an action name at runtime.

To see all the possible commands, type **ros2** **action -h**.

## Listing and introspecting actions

Actions are based on topics and services. Since **rqt\_graph** doesn’t support services (for now), we could see the topics for an action server and client, but that’s about it. Thus, **rqt\_graph** won’t be very useful for introspecting actions. Because of this, we will use the **ros2** command-line tool here.

Let’s learn how to find existing actions and how to get the interface for one action.

Stop all nodes and start the **count\_until\_server** node (Python or C++ one). Then, list all available actions by running the following command:

```
$ ros2 action list
/count_until
```

Here, we found the **/count\_until** action. As we’ve seen with topics and services, if you don’t provide any namespace for the name (we wrote **count\_until** in the server code), a leading slash will be added automatically.

From this action name, we can get more information, including the action interface.

Run **ros2 action info <****action\_name> -t**:

```
$ ros2 action info /count_until -t
Action: /count_until
Action clients: 0
Action servers: 1
/count_until_server [my_robot_interfaces/action/CountUntil]
```

From this, we can see that the action server is hosted in the **count\_until\_server** node, and we also find the action interface. For **ros2 action info** to show the interface, don’t forget to add **\-t**; otherwise, you’ll just see the node’s name.

Finally, we can get the interface:

```
$ ros2 interface show my_robot_interfaces/action/CountUntil
# Here you should see the action definition
```

This process is the same as what we followed for services. Now that we know the action name and interface, we can try the service directly from the Terminal.

## Sending a goal from the Terminal

If you write a service server and want to try it before writing the action client, you can use the **ros2 action send\_goal** command line.

The complete command is **ros2 action send\_goal <action\_name> <action\_interface> "<goal\_in\_json>"**. You can also add **\--feedback** after the command to receive the (optional) feedback from the server. Let’s try it out:

```
$ ros2 action send_goal /count_until my_robot_interfaces/action/CountUntil "{target_number: 3, delay: 0.4}" --feedback
```

You will get the following result:

```
Waiting for an action server to become available...
Sending goal:
 target_number: 3
delay: 0.4
Goal accepted with ID: cad1aa41829d42c5bb1bf73dd4d66600
Feedback:
current_number: 1
Feedback:
current_number: 2
Feedback:
current_number: 3
Result:
reached_number: 3
Goal finished with status: SUCCEEDED
```

This command is very useful for developing action servers. However, it will only work well for actions for which the goal is simple. Here, we only have an integer number and a double number. If the goal contains an array of 20 3D points, you will spend more time trying to write the command correctly than implementing an action client. In this case, to go faster, use the action client we’ve written in this chapter as a template.

## Topics and services inside actions

By default, with **ros2 topic list** and **ros2 service list**, you won’t see the two topics and three services inside the action. However, they do exist—you just have to add **\--include-hidden-topics** and **\--****include-hidden-services**, respectively:

```
$ ros2 topic list --include-hidden-topics
/count_until/_action/feedback
/count_until/_action/status
...
$ ros2 service list --include-hidden-services
/count_until/_action/cancel_goal
/count_until/_action/get_result
/count_until/_action/send_goal
...
```

With that, we’ve found all the topics and services that are being used. You can explore these a bit more by yourself by using the other **ros2 topic** and **ros2 service** command lines.

Now, there’s one thing we’ve done for nodes, topics, and services: we’ve changed the name at runtime. For some reason, this feature isn’t available for actions yet. As a workaround, you can still do this by renaming the two topics and three services when you start the action server:

```
$ ros2 run my_cpp_pkg count_until_server --ros-args \
&nbsp;&nbsp;&nbsp;&nbsp;-r /count_until/_action/feedback:=/count_until1/_action/feedback \
&nbsp;&nbsp;&nbsp;&nbsp;-r /count_until/_action/status:=/count_until1/_action/status \
&nbsp;&nbsp;&nbsp;&nbsp;-r /count_until/_action/cancel_goal:=/count_until1/_action/cancel_goal \
&nbsp;&nbsp;&nbsp;&nbsp;-r /count_until/_action/get_result:=/count_until1/_action/get_result \
&nbsp;&nbsp;&nbsp;&nbsp;-r /count_until/_action/send_goal:=/count_until1/_action/send_goal
```

With this, the action will be renamed **/count\_until1**. The command is a bit ugly and prone to errors, but when we start nodes using launch files in [_Chapter 9_](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_09.xhtml#_idTextAnchor446), in the _Configuring nodes inside a launch file_ section, this won’t be a problem.

With that, we’ve come to the end of this chapter. I haven’t added any challenges here as I think that this chapter itself is a big enough challenge. I would prefer you to spend your time continuing with the other concepts in this book instead of being stuck too long on actions, especially if you’re just getting started with ROS.

## Summary

In this chapter, you worked on ROS 2 actions. You created various actions to solve a problem that services don’t handle well: when the server may take some time to execute the request.

With actions, you can properly handle this case. While the goal is being executed, you can get some feedback from the server, or even decide to cancel the goal. Also, you could handle several goals at the same time, queue them, replace one with another one, and so on (we haven’t seen this in this chapter as it’s something you can look into if you want to develop your skills further).

You can implement action servers and clients in your code using the **rclpy.action** library for Python and the **rclcpp\_action** library for C++.

Here are the main steps for writing an action server:

1.  Since we’re on the server side, we must choose the action name and interface. Usually, for an action, you will have to create a custom interface (in a dedicated package).
2.  Then, you must import the interface into your code and create an action server in the constructor. Here, you will register three callback methods:
    -   **Goal callback**: When the server receives a goal, choose whether to accept or reject it.
    -   **Execute callback**: After a goal has been accepted, you can execute it. During the execution of the goal, you can also publish optional feedback.
    -   **Cancel callback (optional mechanism)**: If you receive a cancel request, you can accept or reject it. If accepted, you will have to cancel the current goal execution.

To write an action client, you must follow these steps:

1.  Find which name and interface you need to use so that you can communicate with the server.
2.  Import the interface into your code and create an action client in the constructor.
3.  Add a method to send a goal. After you send a goal, you will have to write several callbacks:
    -   **Goal response callback**: You will know whether the goal has been accepted or rejected by the server.
    -   **Goal result callback**: After the goal has been executed by the server, you will get the result and the goal’s final state here.
    -   **Feedback callback (optional)**: If the server publishes any feedback, you can receive it here.
4.  Finally, from anywhere in the code, you can decide to cancel the execution of a currently active goal.

On top of all that, with the **ros2 action** command line, you can introspect your actions and send goals directly from the Terminal. Also, since actions are based on topics and services, you can introspect each underlying communication with **ros2 topic** and **ros2** **service**, respectively.

Now, if you managed to get here while reading this book for the first time, congratulations—this chapter is probably one of the most difficult to follow. If you’re still wondering what I was talking about the whole time, don’t worry—you can come back to actions later once you’ve finished this book and become more experienced with ROS.

We’re now done with the three types of communication in ROS 2. In the next chapter, we will go back to a more beginner level and continue to work on nodes. This time, we will learn how to customize nodes when we start them so that we can make our application more dynamic.

<table id="table001-2"><colgroup></colgroup><colgroup><col></colgroup><colgroup><col></colgroup><tbody><tr><td><h4>Unlock this book’s exclusive benefits now</h4><p>This book comes with additional benefits designed to elevate your learning experience.</p></td><td rowspan="2"><p><img alt="" width="246" height="238" src="attachments/9781835881408.png"></p><p lang="en-US" xml:lang="en-US"><a href="https://www.packtpub.com/unlock/9781835881408" target="_blank" rel="noopener noreferrer">https://www.packtpub.com/unlock/9781835881408</a></p></td></tr><tr><td><p><em>Note: Have your purchase invoice ready before </em><span><em>you begin.</em></span></p></td></tr></tbody></table>
