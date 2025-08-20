---
title: "Chapter 10: Discovering TFs with RViz"
source: "https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_10.xhtml#_idParaDest-252"
author:
  - "[[Edouard Renard]]"
published:
created: 2025-08-18
description: "10			Discovering TFs with RViz			 In Part 3 of this book, you will create a robot simulation with ROS 2. However, before you get started, you first need to understand what TransForms..."
tags:
  - "clippings"
---
## 10

## Discovering TFs with RViz

In *Part 3* of this book, you will create a robot simulation with ROS 2. However, before you get started, you first need to understand what **TransForms** (**TFs**) are.

In ROS, a TF is the transformation between two frames in 3D space. TFs will be used to track the different coordinate frames of a ROS robot (or system with multiple robots) over time. They are used everywhere and will be the backbone of any robot you create.

To understand TFs, we will first look at an existing robot model. As we did back in [*Chapter 3*](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_03.xhtml#_idTextAnchor095), here, we will discover the concepts by experimenting, and you will build an intuition of how things work. During this phase, you will discover a few new ROS tools, including **RViz**, a 3D visualization package.

You will see for yourself how TFs work, how they are related to each other, and how to visualize the TF tree for any robot. By the end of this chapter, you will understand what TFs are, what problems they solve, and how they are used in a ROS 2 application.

Good news: once you understand TFs, well, it’s the same principle for any ROS robot, so you can directly apply what you learn here to your future projects.

This chapter will be quite small and quick to finish. We won’t write any code here, and there is no GitHub repository. All you have to do is follow the experiments. Not everything has to make sense right now; the goal is to get enough context to understand what we will be doing later. Don’t hesitate to come back to this chapter once you’ve finished *Part 3*.

In this chapter, we will cover the following subjects:

- Visualizing a robot model in RViz
- What are TFs?
- Relationship between TFs
- What problem are we trying to solve with TFs?

## Technical requirements

At the beginning of the book, I gave you two options: either installing Ubuntu with a dual boot or in a virtual machine.

If you chose the VM path, you should have been fine for all chapters in *Part 1* and *Part 2*. For this chapter and the next two, we will use a 3D visualization tool (RViz) that might not work if your computer is not powerful enough.

I would suggest first trying to run the commands from this chapter. If it doesn’t work well (it’s too slow, for example), then I strongly recommend you set up a dual boot with Ubuntu and ROS 2 (see the instructions in [*Chapter 2*](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_02.xhtml#_idTextAnchor051)). If RViz works fine, then continue like this for now. The dual boot will be required for [*Chapter 13*](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/ros-2-from/9781835881408/B22403_13.xhtml#_idTextAnchor618).

## Visualizing a robot model in RViz

In this section, you will discover RViz. RViz allows you to visualize a robot model in 3D and contains many plugins and functionalities that will help you develop your robotics applications. With RViz, you will be able to visualize the TFs for a robot, so we can start to understand what they are.

As we haven’t created a robot model yet, we will use one from an existing ROS 2 package named **urdf\_tutorial**. We will load a robot model in RViz and learn how to navigate the software.

Let’s start by setting up everything we need for this chapter.

## Installation and setup

First of all, there is no need to install RViz. It was already included when you installed ROS 2 at the beginning of the book (with the `**sudo apt install** **ros-<distro>-desktop**` command).

To visualize TFs for a robot model on RViz, we will install a new ROS package named **urdf\_tutorial**. This package contains some existing launch files and robot model files (how to create a robot model will be the focus of the next chapter).

If you remember, to install a ROS 2 package with **apt**, you have to start with **ros**, then write the name of the distribution you are using, and finally, add the package name. All words are separated with dashes (not underscores).

Open a terminal and install this package:

```
$ sudo apt install ros-<distro>-urdf-tutorial
```

Then, so that you can use the package, make sure you source the environment or simply open a new terminal.

Let’s now visualize a robot model.

## Starting RViz with a robot model

The **urdf\_tutorial** package contains a launch file, named **display.launch.py**, that will start RViz and load a robot model into it. For now, we will just use it, and in the following chapters, we will understand how this process works so we can replicate it.

So, we need to start this launch file and also load a robot model. Where will we get one? There are some existing models in the **urdf\_tutorial** package. To find them, navigate to the **share** directory where the package was installed, and you will find a **urdf** folder under the package name:

```
$ cd /opt/ros/<distro>/share/urdf_tutorial/urdf/
```

A **Unified Robot Description Format** (**URDF**) file is basically the description of a robot model. We will come back to this in the next chapter. For now, all we want to do is to visualize one. In the **urdf** folder, you can find several robot model files:

```
$ ls
01-myfirst.urdf            04-materials.urdf    07-physics.urdf
02-multipleshapes.urdf     05-visual.urdf        08-macroed.urdf.xacro
03-origins.urdf            06-flexible.urdf
```

You can now start a robot model in RViz by launching the **display.launch.py** file and add the path to the robot model with an additional **model** argument after the launch file:

```
$ ros2 launch urdf_tutorial display.launch.py model:=/opt/ros/<distro>/share/urdf_tutorial/urdf/07-physics.urdf
```

Note

To avoid errors, it’s better to provide the absolute path to the.**urdf** file, even if you run the command from the same folder.

After running the command, you should see something like this:

![Figure 10.1 – A robot model on RViz](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/api/v2/epubs/urn:orm:book:9781835881408/files/image/B22403_10_1.jpg)

Figure 10.1 – A robot model on RViz

![](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/api/v2/epubs/urn:orm:book:9781835881408/files/image/11.png) **Quick tip**: Need to see a high-resolution version of this image? Open this book in the next-gen Packt Reader or view it in the PDF/ePub copy.

![](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/api/v2/epubs/urn:orm:book:9781835881408/files/image/21.png) **The next-gen Packt Reader** and a **free PDF/ePub copy** of this book are included with your purchase. Unlock them by scanning the QR code below or visiting [https://www.packtpub.com/unlock/9781835881408](https://www.packtpub.com/unlock/9781835881408).

![](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/api/v2/epubs/urn:orm:book:9781835881408/files/image/9781835881408.png)

You will get two windows: the main one (**RViz**) with the robot model, and a **Joint State Publisher** window with some cursors. The robot model we have here is a replica of a famous science-fiction movie robot. It has some wheels, a torso, a head, and a gripper.

Let’s focus on the main window (**RViz**) for now. Take some time to learn how to navigate in the 3D space and move around the robot. You can use the left click, right click, and mouse wheel. For this, it’s best to have a mouse, but you could still manage to navigate with the touchpad of a laptop, although it’s less ergonomic.

You can also resize the window and the various sections inside RViz. Pretty much everything you see can be customized. Now that you can load a robot model in RViz, we will start to experiment with TFs.

## What are TFs?

There are two main parts in a robot model: links and TFs. In this section, we will visualize them both and understand how they work together.

Let’s start with links.

## Links

Have a look at the menu on the left side of the **RViz** window. There, you will see, in blue bold letters, **RobotModel** and **TF**. This is what we will focus on in this chapter. As you can see, you can enable or disable both menus.

Disable **TF**, keep **RobotModel**, and expand the menu. There, you can find a submenu called **Links**.

![Figure 10.2 – RobotModel with Links menu on RViz](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/api/v2/epubs/urn:orm:book:9781835881408/files/image/B22403_10_2.jpg)

Figure 10.2 – RobotModel with Links menu on RViz

Check and uncheck some boxes. As you can see from this menu, a *link* is one rigid part (meaning one solid part with no articulation) of the robot. Basically, in ROS, a robot model will consist of a collection of rigid parts put together.

In this example, links are represented by basic shapes: boxes, cylinders, and spheres. Those rigid parts do nothing on their own, so how are they connected, and how do they move between each other?

This is where we introduce TFs.

## TFs

Let’s now check the **TF** box. You can keep **RobotModel** checked or unchecked. Inside the **TF** menu, you have a submenu called **Frames**, and you can also enable or disable each frame for the robot.

![Figure 10.3 – Frames and TFs on RViz](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/api/v2/epubs/urn:orm:book:9781835881408/files/image/B22403_10_3.jpg)

Figure 10.3 – Frames and TFs on RViz

The axes you see here (red, green, and blue coordinate systems) represent the frames, or basically the origin of each link of the robot.

Coordinate systems follow the right-hand rule in ROS. Following *Figure 10**.4*, you have the following:

- X axis (red) pointing forward
- Y axis (green) pointing 90 degrees to the left
- Z axis (blue) pointing up
![Figure 10.4 – Convention for coordinate systems in ROS](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/api/v2/epubs/urn:orm:book:9781835881408/files/image/B22403_10_4.jpg)

Figure 10.4 – Convention for coordinate systems in ROS

The arrows that you see between each frame in *Figure 10**.3* are the relationship between each rigid part (link) of the robot. A TF is represented by an arrow.

Note

There can be some confusion between the names **links**, **frames**, and **TFs**. Let’s make things clear:

\- Link: one rigid part of a robot

\- Frame: the origin of a link (axis in RViz)

\- TF: the relationship between two frames (arrow in RViz)

So, each rigid part will be linked to another rigid part, thanks to a TF. This transformation defines how those two parts are placed relative to each other. In addition to that, the TF also defines whether the two parts are moving, and if so, how—translation, rotation, and so on.

To make some parts of the robot move, you can move some of the cursors in the **Joint State Publisher** window. You will see the frames and TFs moving in RViz. If you also check the **RobotModel** box again, you will see the rigid parts moving as well.

To better understand, here is an analogy with the human arm: we can define the parts of the arm as *arm* (shoulder to elbow) and *forearm* (after the elbow). Those two are rigid parts (here, links) and do not move on their own. Each link has an origin frame, and there is a TF that defines where the arm and forearm are connected (imagine an axis in the elbow), and how they move (in this case, it’s a rotation with a minimum and maximum angle).

As we will see later in this chapter, TFs are tremendously important. If the TFs for a robot are not correctly defined, then nothing will work.

Now you know what TFs are, but how are they all related to each other? As you can see on RViz, it seems that TFs are organized in a certain way. Let’s go one step further and understand what the relationship between TFs is.

## Relationship between TFs

In RViz, we have seen the links (rigid parts) and TFs (connections between the links). Links are mostly used for visual aspects in simulation and will be useful to define inertial and collision properties (when we work with Gazebo). TFs define how links are connected, and how they move between each other.

In addition to that, all the TFs for a robot are organized in a particular way, inside a tree. Let’s explore the relationship between TFs and visualize the TF tree for the robot we started on RViz.

## Parent and child

Each TF will be connected to another TF, with a parent/child relationship. To see one, you can, for example, disable all TFs on RViz, and only check the **base\_link** and **gripper\_pole** frames.

![Figure 10.5 – The relationship between two frames](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/api/v2/epubs/urn:orm:book:9781835881408/files/image/B22403_10_5.jpg)

Figure 10.5 – The relationship between two frames

As you can see in this example, an arrow is going from the **gripper\_pole** frame to the **base\_link** frame. This means that **gripper\_pole** is the child of **base\_link** (or, **base\_link** is the parent of **gripper\_pole**).

If you look back at *Figure 10**.3*, you can see all the frames for the robot, with all the relationships between them (TFs).

The order of those relationships is very important. If you move **gripper\_pole** relative to **base\_link** (the **gripper\_extension** cursor in the **Joint State Publisher** window), then anything that’s attached to **gripper\_pole** (meaning children of **gripper\_pole**) will also move with it.

That makes sense: when you rotate your elbow, your forearm is moving, but also your wrist, hand, and fingers. They don’t move relative to the forearm, but as they are attached to it, they move relative to the arm.

Now, you can visualize all the links and TFs on RViz, see the relationship between TFs, and also how they are related to each other. Let’s go further with the **/** **tf** topic.

## The /tf topic

At this point, you might think that what we did in *Part 2* of this book has nothing to do with what we are doing now. Well, everything we have seen here is still based on nodes, topics, and so on.

Let’s list all nodes:

```
$ ros2 node list
/joint_state_publisher
/robot_state_publisher
/rviz
/transform_listener_impl_5a530d0a8740
```

You can see that RViz is actually started as a node (**rviz**). We also have the **joint\_state\_publisher** and **robot\_state\_publisher** nodes, which we will come back to in the following chapters of this book. Now, let’s list all topics:

```
$ ros2 topic list
/joint_states
/parameter_events
/robot_description
/rosout
/tf
/tf_static
```

You can see that the nodes that were started are using topics to communicate with each other. In this topic list, we find the **/tf** topic. This topic will be there for any robot you create. The TFs you saw on RViz are actually the 3D visualization of this topic—meaning the **rviz** node is a subscriber to the **/** **tf** topic.

You can subscribe to the topic from the terminal with the following:

```
$ ros2 topic echo /tf
```

If you do so, you will receive lots of messages. Here is an extract:

```
transforms:
- header:
    stamp:
      sec: 1719581158
      nanosec: 318170246
    frame_id: base_link
  child_frame_id: gripper_pole
  transform:
    translation:
      x: 0.19
      y: 0.0
      z: 0.2
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

This extract matches what we previously saw on RViz. It represents the transformation between **base\_link** and **gripper\_pole**. Here are the important pieces of information we can get from this message:

- Timestamp for the TF
- Parent and child frame IDs
- The actual transformation, with a translation and a rotation

Note

The rotation is not represented by Euler angles (**x**, **y**, **z**), but by a quaternion (**x**, **y**, **z**, **w**). Quaternions are usually better suited for computers, but it’s difficult for humans to visualize them. Do not worry about this—we won’t really have to deal with quaternions. If you ever have to do so in the future, you will have access to libraries that can convert angles into something you can understand.

One important thing we can get here is that the transformation is for a specific time. It means that with the topic data, you can follow the evolution of all TFs over time. You can know where **gripper\_pole** is relative to **base\_link**, now or in the past.

This **/tf** topic contains all the information we need, but it’s not really human-readable. That’s why we started with RViz, so we could see a 3D view with all the TFs.

Let’s now finish this section by printing the TF tree, so we can see all the relationships in one single image.

## Visualizing the TF tree

For each robot, you can visualize the complete TF tree in a simplified way, so that you can see all the relationships between all the TFs.

To do that, you will need to use the **tf2\_tools** package. Make sure it is installed:

```
$ sudo apt install ros-<distro>-tf2-tools
```

Don’t forget to source the environment after installing the package. Now, keep the robot running on RViz, and execute this command in a second terminal:

```
$ ros2 run tf2_tools view_frames
```

As you will see with the logs, it will listen to the **/tf** topic for five seconds. After this, the command exits with a big log that you can ignore.

You will get two new files, in the same directory as where you ran the command.

Open the PDF file. You will see something like this (I’m just adding the left side of the image, otherwise the text would be too small to read in the book):

![Figure 10.6 – TF tree for a robot](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/api/v2/epubs/urn:orm:book:9781835881408/files/image/B22403_10_6.jpg)

Figure 10.6 – TF tree for a robot

In this file, you get all the links and TFs at once, and it’s quite clear to see which link is a child of which other link. Each arrow on the PDF represents a TF between the links.

As you can see, the root link for that robot is named **base\_link** (the name **base\_link** is used for most robots as the first link). This link has four children: **gripper\_pole**, **head**, **left\_leg**, and **right\_leg**. Then, those links also get more children. Here, we can clearly see all the children for the **gripper\_pole** link.

We can now understand that when we previously moved **gripper\_pole** relative to **base\_link**, then all children of **gripper\_pole** were also moved relative to **base\_link**.

Note

In ROS, one link can have several children, but only one parent. We will come back to this in the next chapter when we define the links and TFs ourselves.

In this example, we have just one robot. If you were to have several robots in your application, then you would have, for example, a **world** frame as the root link. Then, this frame would have several children: **base\_link1**, **base\_link2**, and so on. Each robot base link would be connected to the **world** frame. Thus, you can get a complete TF tree, not just for one robot but also for a complete robotics application with several robots.

Now, you have seen pretty much everything about TFs: what they are, how they are related to each other, and how they are organized. Let’s finish this chapter by understanding what problem we are trying to solve with TFs.

## What problem are we trying to solve with TFs?

You have now seen what TFs are and how you can visualize them for any ROS robot. This is great, but now we come to the final question for this chapter: why do we need to care about this? What problem are we trying to solve?

## What we want to achieve

For a robotics application to work, we want to keep track of each 3D coordinate frame over time. We need a structured tree for all the frames of the robot (or robots).

There are two components here: we need to know where things are and when the transformations happened. If you remember, when we checked the **/tf** topic, you could see that for each parent and child frame, we had a transformation (translation and rotation in 3D space), and we also had a timestamp.

Here are some concrete examples of some questions you could need to answer in a robotics application:

- For one mobile robot, where is the right wheel relative to the left wheel and to the base of the robot? How does the wheel movement evolve over time?
- If you have an application with a robotic arm and a camera, then where is the camera relative to the base of the robot? And to the hand of the robot, so that the arm can correctly pick up and place objects that were detected by the camera?
- In another application with several mobile robots, where is each robot relative to the other ones?
- If you combine the two previous examples, where is the hand of the robotic arm relative to the base of one of the mobile robots?

So, with TFs, we want to know the following:

- How frames are placed relative to each other
- How they move relative to each other and over time

This is required for a robot to correctly work with ROS.

Now, let’s see how you would compute the TFs by yourself, and how ROS automatically does this for you so you don’t need to worry about it.

## How to compute TFs

What is a transformation exactly? A transformation is the combination of a translation and a rotation in space.

As we are working in 3D, we have three components for the translation (**x**, **y**, **z**), and three components for the rotation (**x**, **y**, **z**). To find a transformation between two frames, you will need to compute those six elements, using 3x3 matrices.

I won’t go into any mathematical details here, but you can probably guess that it won’t be an easy task. Also, you need to compute this transformation for each frame, relative to the other frame. This increases the complexity.

For example, let’s say you need to know where **left\_front\_wheel** is relative to **base\_link**. Following the previous TF tree (open the PDF again), you can see that you need to follow this order: **base\_link**, **left\_leg**, **left\_base**, and **left\_front\_wheel**.

Let’s visualize that on RViz:

![Figure 10.7 – Three transformations between four frames](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/api/v2/epubs/urn:orm:book:9781835881408/files/image/B22403_10_7.jpg)

Figure 10.7 – Three transformations between four frames

You will need to compute three transformations in a row so that you can get this **base\_link** to **left\_front\_wheel** transformation. You will have to repeat this for each frame relative to all other frames (the complexity then increases a lot as you add more frames), and track these over time.

This sounds like a lot of work. Fortunately, we don’t have to do any of that, thanks to the ROS TF functionality. There is a library called **tf2**, and it already does that for us.

In the end, the biggest challenge with TFs is to understand how they work. You will mostly not use TFs directly in your application. Several packages will handle that for you. The only thing we need to do is to provide a robot description that specifies all the links and TFs for the robot. Then, using a package named **robot\_state\_publisher**, ROS will automatically publish TFs for us. That’s what we will focus on in the next two chapters.

## Summary

In this chapter, we started something a bit different. ROS is not all about programming; there are many additional things that make it a great tool for robotics.

You first discovered a 3D visualization tool used for ROS, named RViz. You will use this tool in most of your ROS applications. With RViz, you can visualize a robot model, which will be helpful when developing the model by yourself.

Then, you discovered what TFs are, and why they are so important in a ROS application. Here’s a quick recap:

- We need to keep track of each 3D coordinate frame over time, for the entire robotics application (one or several robots).
- Instead of computing the transformations ourselves, we use the ROS TF functionality, with the **tf2** library. TFs are published on the **/** **tf** topic.
- TFs are organized into a structured tree that you can visualize.
- A TF defines how two frames are connected, and how they move relative to each other, over time.

To specify TFs for a robot, we will have to create a robot model, named *URDF*. This robot model will then be used by the **robot\_state\_publisher** node (we will see this a bit later) to publish the TFs. The published TFs will then be used by other packages in your application.

In the end, we won’t really interact with TFs directly. The most important thing in this chapter is to understand what TFs are, and why we need them. This will help you understand what you're doing in the next chapter, when you create a robot model.

If things are still a bit confusing for now, don’t worry too much. Continue with the next few chapters, and then come back to this TF chapter again and everything will make more sense.

Now, let’s jump to the next chapter and create our first robot model.

<table><colgroup><col></colgroup> <colgroup><col></colgroup><tbody><tr><td><h4>Unlock this book’s exclusive benefits now</h4><p>This book comes with additional benefits designed to elevate your learning experience.</p></td><td rowspan="2"><p><img width="246" height="238" src="https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/api/v2/epubs/urn:orm:book:9781835881408/files/image/9781835881408.png"></p><p><a href="https://www.packtpub.com/unlock/9781835881408">https://www.packtpub.com/unlock/9781835881408</a></p></td></tr><tr><td><p><em>Note: Have your purchase invoice ready before </em><span><em>you begin.</em></span></p></td></tr></tbody></table>