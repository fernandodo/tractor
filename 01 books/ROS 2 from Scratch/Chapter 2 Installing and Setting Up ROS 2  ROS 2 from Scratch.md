## Installing and Setting Up ROS 2

Before using ROS 2, we need to install it and set it up. Doing this is not as trivial as just downloading and installing a basic program. There are several ROS 2 versions (called **distributions**), and we need to choose which one is the most appropriate. We also need to pick an Ubuntu version as ROS and Ubuntu distributions are closely linked.

Once you know which ROS/Ubuntu combination you need, you have to install the corresponding Ubuntu **operating system** (**OS**). Although being familiar with Linux is a prerequisite for this book, I will still do a recap on how to install Ubuntu on a **virtual machine** (**VM**), just in case, so you won’t be lost and can continue with this book.

Then, we will install ROS 2 on Ubuntu, set it up in our environment, and install additional tools that will allow you to have a better development experience.

By the end of this chapter, you will have everything ready on your computer so that you can use ROS 2 and write custom programs.

Even if all the installation steps sound a bit daunting, don’t worry—it’s not that hard, and it gets easier to do with every new installation. To give you an idea, with a stable internet connection, it takes me about 1 hour to install a fresh version of Ubuntu and 20 minutes to install ROS (most of that time is spent waiting for the installation to finish).

In this chapter, we are going to cover the following topics:

-   Which ROS 2 distribution to choose
-   Installing the OS (Ubuntu)
-   Installing ROS 2
-   Setting up the environment for ROS 2
-   Extra tools for ROS 2 development

## Which ROS 2 distribution to choose

Before you install ROS 2, it’s important to know which distribution you need to use. To make that decision, you first need to understand a bit more about what ROS 2 distributions are, and what specificities each one has.

## What is a ROS 2 distribution?

ROS 2 is a project in continuous development, constantly receiving new features or improvements to existing ones.

A distribution is simply a _freeze_ in the development at some given point to create a stable release. With this, you can be sure that the core packages for one given distribution will not have any breaking changes. Without distributions, it would be impossible to have a stable system, and you would need to update your code constantly.

Every year, a new ROS 2 distribution is released on May 23. This day corresponds to _World Turtle Day_. As you will be able to observe, all ROS distributions have a turtle as a logo; there’s a mobile robot platform named **TurtleBot** and even a 2D educational tool named **Turtlesim**. This is based on a reference to an educational programming language from 1967 named _Logo_, which included a feature to move some kind of _turtle robot_ on the screen. So, if you were confused about why there are so many turtles everywhere, now you know—and that’s the end of this turtle parenthesis.

You can see all ROS 2 distributions on the ROS 2 documentation releases page: [https://docs.ros.org/en/rolling/Releases.html](https://docs.ros.org/en/rolling/Releases.html).

You will see one new distribution every year in May. As for the order, there is no number; instead, the names are in alphabetical order. The first official release was named _Ardent Apalone_, then _Bouncy Bolson_, and so on. In May 2024, **ROS Jazzy Jalisco** was released. Following this, you can expect to have _ROS K_ in 2025, _ROS L_ in 2026, and so on. The name of a new release is usually announced 1 year in advance.

Note

ROS distributions contain two names, but it’s common practice just to refer to the first one. So, instead of talking about ROS Jazzy Jalisco, we will talk about **ROS Jazzy**. We could also write _ROS 2 Jazzy_ to specify that this distribution is for ROS 2, not ROS 1, but this isn’t needed since Jazzy is a name only used for ROS 2, hence ROS Jazzy.

On top of all the displayed distributions, there is another one that exists in parallel: **ROS Rolling**. This distribution is a bit special and is the distribution where all new developments are made. To make an analogy with Git and versioned systems, it’s just like having a _development_ branch and using this branch to release stable versions once a year. Thus, ROS Rolling is not a stable distribution, and I don’t recommend using it for learning or to release a product. This is a distribution you can use if you want to test brand-new features before they are officially released into the next stable distribution—or if you want to contribute to the ROS code. However, if you’re reading this book, you’re not there yet.

Now that you know what ROS 2 distributions are and how to find them, let’s start to look at the differences between them. This will allow us to choose the right one.

## LTS and non-LTS distributions

If you look a bit closer, you’ll see that some distributions are supported for 5 years, others for 1.5 years (this only applies after 2022). You can see this by comparing the release date with the **end-of-life** (**EOL**) date. Currently supported distributions also have a green background on the screen, so you can easily spot them.

When a distribution reaches its EOL date, it just means that it will not receive official support and package updates anymore. This doesn’t mean you can’t use it (in fact, lots of companies are still using legacy versions from 5 years ago or more), but you won’t get any updates.

The first official ROS 2 distribution was _ROS Ardent_, released in December 2017. After that, the first few distributions were still not quite complete, and the development team preferred to release shorter distributions so that the development could go faster.

_ROS Humble_ was the first **Long-Term Support** (**LTS**) release supported for 5 years (2022-2027).

_ROS Jazzy_ is also an LTS version, with official support from 2024 to 2029. From this, you can expect that every 2 years (even number: 2024, 2026, 2028, and so on), a new LTS distribution will be released in May and supported for 5 years.

A few LTS distributions can coexist. In 2026, for example, with the release of the _ROS L_ distribution, you will be able to use ROS Humble and ROS Jazzy as well.

Then, you have non-LTS distributions. Those are released on odd years (2023, 2025, 2027, and so on) and are supported for 1.5 years only. Those distributions are released just so you can have access to new development in a somehow stable release without having to wait for 2 years. However, due to the short lifespan of non-LTS distributions and the fact that they will probably be less stable (and less supported), it is best not to use them if your goal is learning, teaching, or using ROS for a commercial application.

With this, you can see that we can discard half of the distributions and now focus only on the LTS ones that are currently supported. Let’s finish this section and choose the distribution that we will use for this book.

## How to choose a ROS distribution

What I recommend is to use the latest available LTS distribution. However, I wouldn’t necessarily use an LTS distribution just after it’s been released because it can still contain some bugs and problems. Also, some of the plugins and other community packages you need might not have been ported yet. Generally, if you want to work with a stable system, it’s sometimes best not to stay too close to the new and shiny technology and wait a bit.

For example, ROS Humble was released in May 2022. Right after it was available, I tested it, but to use it in a production environment, I would have had to wait until September or even November, just to be sure everything was working correctly.

So, for this book, we will use ROS Jazzy, which was released in May 2024.

Note

You can learn with one distribution and then start a project with another one. If you have a project or job that requires you to use a different ROS 2 distribution, you can still start to learn with ROS Jazzy. The gap between distributions is very small, especially for the core functionalities. 99% of this book can be applied to any ROS 2 LTS distribution released after 2022.

## Installing the OS (Ubuntu)

ROS 2 works on three OSs: Ubuntu, Windows, and macOS. Although Ubuntu and Windows get Tier 1 support, macOS has only Tier 3 support, meaning “best effort,” not “fully tested.” You can learn more about what Tier 1, 2, and 3 mean on the REP 2000, which describes the timeline and target platforms for ROS 2 releases: [https://www.ros.org/reps/rep-2000.html](https://www.ros.org/reps/rep-2000.html).

This means that using macOS for ROS 2 is not necessarily the best choice for learning (if you’re an Apple user). We’re left with Windows or Ubuntu.

From teaching experience, I saw that even if ROS could work well on Windows, it’s not easy to install and use it correctly. Lots of bugs can occur, especially with the 2D and 3D tools. When you’re learning ROS, you want a smooth experience, and you want to spend time learning the features, not fixing the configuration.

Hence, the best overall option is to use Ubuntu. If you don’t have Ubuntu and are using Windows/macOS, you can either install Ubuntu natively as a dual boot on your computer or use a VM (there are a few other options, but I won’t cover those here).

Now that we have selected ROS Jazzy, and we want to run it on Ubuntu, the question is: on which Ubuntu distribution do we install it?

## The relationship between ROS 2 and Ubuntu

If you go to the Jazzy release page ([https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html)), you’ll see that ROS Jazzy is supported on **Ubuntu 24.04** (and not any other previous or future Ubuntu distributions).

There is a close relationship between ROS and Ubuntu distributions. This relationship is quite simple: for every new Ubuntu LTS distribution (every 2 years on an even number), there is a new ROS 2 LTS distribution:

-   **Ubuntu 22.04**: ROS Humble
-   **Ubuntu 24.04**: ROS Jazzy
-   **Ubuntu 26.04**: ROS L

It’s important to use the correct combination. Thus, before installing ROS Jazzy, the first thing you must do is make sure you have Ubuntu 24.04 installed on your computer. If you happen to have an older version, I strongly encourage you to upgrade or simply install Ubuntu 24.04 from scratch.

Note

If you have to use another Ubuntu distribution because, for example, you’re using a computer from school/work and you can’t change the OS, then use the corresponding ROS distribution. However, I recommend **not** using anything older than ROS Humble and avoiding non-LTS distributions. You could also install Ubuntu 24.04 on a VM (as described a bit later).

You have probably already installed a Linux OS at some point in your life, but, from experience, I know some people reading this could get a bit lost with the installation. Hence, I will provide additional installation instructions—an overview of dual boot and detailed instructions for VMs. Feel free to skip this and go to the ROS 2 installation section if you already have installed Ubuntu.

## Installing Ubuntu 24.04 natively with a dual boot

The best option is to have Ubuntu installed natively on your computer. This will allow you to follow this book and then go further without any problems. I won’t provide a complete tutorial on how to do that here; you can easily find lots of free tutorials on the internet.

Here are the high-level important steps you have to follow:

1.  Free some space on your disk so that you can create a new partition. I recommend a minimum of 70 GB, more if possible.
2.  Download an Ubuntu **.iso** file from the official Ubuntu website (Ubuntu 24.04 LTS).
3.  Flash this image on an SD card or USB key (with a tool such as Balena Etcher).
4.  Reboot your computer and choose to boot from the external device.
5.  Follow the installation instructions. **Important**: When asked how you want to install, select **Alongside Windows**, for example—don’t erase all your disk.
6.  Complete the installation. Now, when you boot your computer, you should get a menu where you can select whether you want to start Ubuntu or Windows.

Those are the main steps to follow; you can find all the information you need on the internet.

## Installing Ubuntu 24.04 on a VM

If you can’t install Ubuntu as a dual boot (due to technical restrictions, lack of admin rights on the computer, or something else), or if you want to get started quickly with not too much effort, just for the sake of learning ROS, then you might wish to use a VM.

A VM is quite easy to set up and is useful for teaching and learning purposes. As an example, when I teach ROS offline in a workshop for beginners, I often provide a VM that contains everything already installed. With that, it’s easier for the participants to get started quickly with ROS. Later, when they have more knowledge, they can take the time to set up a proper OS by themselves.

Note

_Part 3_ of this book (regarding 3D simulation and Gazebo) will probably not work well on a VM. You can still use a VM for most of this book and set up a dual boot at the end.

I will now show you how to install Ubuntu 24.04 on a VM so that you can do it even with no prior knowledge of how to create and run a VM.

### Step 1 – downloading the Ubuntu .iso file

Download the Ubuntu 24.04 **.iso** file [https://releases.ubuntu.com/noble/](https://releases.ubuntu.com/noble/). Note that, just like ROS, Ubuntu distributions also have a name. For Ubuntu 24.04, the name is _Ubuntu Noble Numbat_. We usually only use the first name, so **Ubuntu Noble** in this case.

Click on **64-bit PC (AMD64) desktop image**. The file should be 5 to 6 GB, so make sure you have a good internet connection before downloading it.

### Step 2 – installing VirtualBox

You can start _Step 2_ while the Ubuntu **.iso** file is being downloaded.

Two popular VM managers have a free version: VMware Workstation and VirtualBox. Both would work, but here, I will focus on VirtualBox as it’s slightly easier to use.

Go to the download page of the official VirtualBox website: [https://www.virtualbox.org/wiki/Downloads](https://www.virtualbox.org/wiki/Downloads). Under **VirtualBox Platform Packages**, select the current OS you are running. If you want to install VirtualBox on Windows, for example, choose **Windows hosts**.

Download the installer, then install VirtualBox like any other software.

### Step 3 – creating a new VM

Once you’ve installed VirtualBox and downloaded the Ubuntu **.iso** file, open the VirtualBox software (VirtualBox Manager) and click **New**. This will open a pop-up window where you can start to configure the new VM:

![[attachments/B22403_02_1.jpg]]

Figure 2.1 – Starting the VM setup process

Here, we have the following values:

-   **Name**: Name the machine. This can be anything you want, just so you can recognize the machine—for example, **Ubuntu 24.04 -** **book**.
-   **Folder**: By default, VirtualBox will create a **VirtualBox VMs** folder in your user directory, where it will install all the VMs. You can keep this or change it if you want.
-   **ISO Image**: This is where you select the Ubuntu **.iso** file that you’ve just downloaded.
-   **Type**: This should be **Linux**.
-   **Version**: This should be **Ubuntu (64-bit)**.
-   **Skip Unattended Installation**: Make sure you check this box. Leaving this unchecked could be a cause of issues later.

Click **Next**. Here, you’ll need to choose how much CPU and RAM you want to allocate for the machine:

![[attachments/B22403_02_2.jpg]]

Figure 2.2 – Allocating hardware resources for the VM

This will depend on your computer’s configuration.

Here’s what I recommend for the RAM allocation (**Base Memory** on VirtualBox):

-   If you have 16 GB or more on your computer, allocate 6 GB (like I did in _Figure 2__.2_) or a bit more; this is going to be enough.
-   If you have 8 GB RAM, allocate 4 GB.
-   For less than 8 GB, adjust the RAM value (you can set a value now and modify it later in the settings) so that you can start the VM, open **VS Code** and Firefox with a few tabs, and your machine doesn’t slow down too much. If things get too slow, consider using a more powerful machine for learning ROS.

For the CPU allocation, allocate half of your CPUs. So, if your computer has 8 CPUs, set the value to 4. In my setup, I have 4 CPUs and 8 logical processors, so I chose 4 CPUs. Try not to go under 2 as it will be very slow with only 1 CPU. If in doubt, try one setting now; you can change it later.

All in all, it’s better to stay in the green zone. If you have to push to the orange zone, then make sure that when you run your VM, you don’t run anything else on your computer (or maybe just a web browser with one tab or a PDF reader for this book).

Click **Next**. The last thing to do for now is to configure the virtual hard disk that will be created for the VM:

![[attachments/B22403_02_3.jpg]]

Figure 2.3 – Creating a virtual disk for the VM

Here are the settings you must choose for this screen:

1.  You can keep the default option (**Create a Virtual Hard Disk Now**). The default size is 25 GB. To learn ROS 2, I recommend using 30 to 40 GB minimum. Anyway, the size of the VM will start low and expand as you install more things, so you can set a higher maximum without blocking resources.
2.  Keep the **Pre-allocate Full Size** box unchecked.

Click **Next**. You will now get a recap of all options you chose in the previous steps. Then, click **Finish**. You will see the newly created VM on the left in VirtualBox Manager.

Before you start the VM, there are a few more things we need to configure. Open the settings for the VM (either select it and click on the **Settings** button or right-click on the VM and choose **Settings**).

Modify the following three settings:

-   **System** | **Acceleration**: Uncheck the **Enable Nested** **Paging** box
-   **Display**: Uncheck the **Enable 3D Acceleration** box (this one may already be unchecked)
-   **Display**: Increase **Video Memory** to 128 MB if possible

With these settings, you can probably avoid unexpected behaviors and problems with the graphical interface in the VM.

Note

Every computer is different, with different hardware configurations. What works for me and lots of people might not work the same for you. If you experience weird behavior when running the VM, maybe try again by modifying those previous three settings. Test only one change at a time.

### Step 4 – starting the VM and finishing the installation

Now that the VM has been configured correctly, we need to start it to install Ubuntu, using the Ubuntu **.iso** file that we’ve downloaded and added to the settings.

To start the VM, double-click on it in **VirtualBox Manager**, or select it and click on the **Start** button.

You will get a boot menu. The first choice is **Try or Install Ubuntu** and it should already be selected. Press _Enter_.

Wait a few seconds; Ubuntu will start with the installation screen. Follow the configuration through the different windows:

1.  Choose your language. I recommend you keep **English** so you have the same configuration as me.
2.  Skip the **Accessibility** menu, unless you need to set up a bigger font size, for example.
3.  Select your keyboard layout.
4.  Connect to the internet. To do so, choose **Use** **wired connection**.
5.  At this point, you might have a screen asking you to update the installer. In this case, click **Update now**. Once finished, click **Close installer**. Look for **Install Ubuntu 24.04 LTS** on the VM desktop and double-click on it. This will start the installation again from _Step 1_; repeat _Steps 1_ to _4_.
6.  When you’re asked how you would like to install Ubuntu, choose **Interactive installation**.
7.  For the applications to install with Ubuntu, go with **Default selection**. This will reduce the space used by the VM and you still get a web browser, as well as all the core basics—we don’t need more than that to install ROS 2.
8.  When asked if you want to install recommended proprietary software, choose **Install third-party software for graphics and** **Wi-Fi hardware**.
9.  For the disk setup, select **Erase disk and install Ubuntu**. There’s no risk here as we’re _erasing_ the empty virtual disk we’ve just created (if you were installing Ubuntu as a dual boot, you would have to choose another option).
10.  Choose a username, computer name, and password. Keep things simple here. For example, I use **ed** for the user and **ed-vm** for the computer’s name. Also, make sure that the password is typed correctly, especially if you’ve changed the keyboard layout previously.
11.  Select your time zone.
12.  On the recap menu, click **Install** and wait a few minutes.
13.  When the installation is complete, a popup will ask you to restart. Click **Restart now**.
14.  You will see a message stating **Please remove the installation medium, then press ENTER**. There’s no need to do anything here—just press _Enter_.
15.  After booting, you will get another welcome screen popup. You can skip all the steps and click **Finish** to exit the popup.

With that, Ubuntu has been installed.

Just to finish things properly, open a Terminal window and upgrade the existing packages (it’s not because you just installed Ubuntu that all packages are up to date):

```
$ sudo apt update
$ sudo apt upgrade
```

![[attachments/3.png]] **Quick tip**: Enhance your coding experience with the **AI Code Explainer** and **Quick Copy** features. Open this book in the next-gen Packt Reader. Click the **Copy** button (**1**) to quickly copy code into your coding environment, or click the **Explain** button (**2**) to get the AI assistant to explain a block of code to you.

![[attachments/image_(2).png]]

![[attachments/4.png]] **The next-gen Packt Reader** is included for free with the purchase of this book. Unlock it by scanning the QR code below or visiting [https://www.packtpub.com/unlock/9781835881408](https://www.packtpub.com/unlock/9781835881408).

![[attachments/9781835881408.png]]

That’s it for the installation. However, there’s one more thing specific to VirtualBox that we must do so that the VM window works correctly.

### Step 5 – Guest Additions CD Image

At this point, if you try to resize the window where the VM is running, you will see that the desktop resolution doesn’t change. Also, if you try to copy/paste some text or code between your host and the VM, it probably won’t work.

To fix this, we need to install what’s called the _Guest Additions_ _CD Image_.

First, open a Terminal and run the following command (note that copy/paste doesn’t work yet, so you have to type it manually). This will install a few dependencies, all of which are required for the next step:

```
$ sudo apt install build-essential gcc make perl dkms
```

Then, on the top menu of the VM window, click **Devices** | **Insert Guest Additions** **CD image**.

You will see a new CD image on the Ubuntu menu (on the left). Click on it—this will open a file manager. Inside the file manager, right-click on the empty space and choose **Open in Terminal**. See the following figure for more clarification:

![[attachments/B22403_02_4.jpg]]

Figure 2.4 – Opening the Guest Additions CD Image folder in the Terminal

You will then be inside a Terminal, where you can find a file named **VBoxLinuxAdditions.run**. You need to run this file with admin privileges:

```
$ sudo ./VBoxLinuxAdditions.run
```

After this, you might get a popup asking you to reboot the VM. Click **Restart Now**. If you don’t get this popup, simply run **sudo reboot** in the Terminal.

When the VM starts again, the screen should resize automatically when you change the size of the window. If it doesn’t work the first time, I found that running the command once again (**sudo ./VBoxLinuxAdditions.run**) and rebooting may solve the problem.

You can then right-click on the disk and choose **Eject**, and you’re all set. To enable copy/paste between the host and the VM, go to the top menu and click **Devices** | **Shared Clipboard** | **Bidirectional**.

Now, your VM is fully installed and configured. If you’ve chosen that path, this will allow you to complete at least _Part 1_ and _Part 2_ of this book. For _Part 3_, as mentioned previously, you might encounter some issues, especially when using Gazebo. When you reach this point, I suggest that you install Ubuntu natively with a dual boot.

## Installing ROS 2 on Ubuntu

Now that you have Ubuntu 24.04 installed on your computer (either as a dual boot or in a VM), let’s install ROS 2. As discussed previously, we will install ROS Jazzy here. If you’re using a different Ubuntu distribution, make sure to use the appropriate ROS 2 distribution.

The best way to install ROS 2 is to follow the instructions on the official documentation website, for binary packages installation: [https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html). There are quite a few commands to run, but don’t worry— all you need to do is copy and paste them one by one.

Note

As the installation instructions are often updated, the following commands you see in this book may differ slightly from the ones from the official documentation. If so, copy/paste from the official instructions.

Now, let’s start the ROS 2 installation.

## Setting the locale

Make sure you have a locale that supports UTF-8:

```
$ locale
```

With this, you can check that you have UTF-8. If in doubt, just run those commands one by one (I do this every time):

```
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8
```

Now, check again; you’ll see UTF-8 this time:

```
$ locale
```

## Setting up the sources

Run those five commands one by one (note that it’s better to copy/paste them from the official documentation directly):

```
$ sudo apt install software-properties-common
$ sudo add-apt-repository universe
$ sudo apt update && sudo apt install curl -y
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro\/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr\/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2\/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo\tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

The main goal of these commands is to add the ROS packages server to your **apt** sources. Run the following command:

```
$ sudo apt update
```

You should now see additional lines, something like this:

```
Get:5 http://packages.ros.org/ros2/ubuntu noble InRelease [4,667 B]
Get:6 http://packages.ros.org/ros2/ubuntu noble/main amd64 Packages [922 kB]
```

Those new sources (**packages.ros.org**) will allow you to fetch the ROS 2 packages when installing with **apt**. Here, you can also see **noble**, which means the ROS 2 packages are for Ubuntu Noble (24.04). Since you know how Ubuntu and ROS distributions are linked, this also means ROS Jazzy.

Now that we have the correct sources, we can finally install ROS 2.

## Installing ROS 2 packages

As a best practice, upgrade all your existing packages before you install any ROS 2 packages:

```
$ sudo apt update
$ sudo apt upgrade
```

Now, you can install ROS 2 packages. As shown in the documentation, you can choose between **Desktop Install** or **ROS-Base Install**.

ROS 2 is not just one piece of software or package. It’s a collection of many packages:

-   **ROS-Base**: This contains the bare minimum packages to make ROS 2 work correctly
-   **ROS Desktop**: This contains all the packages in ROS-Base, plus lots of additional packages, so you get access to more tools, simulations, demos, and so on

Since you’re installing ROS 2 on a computer with a desktop, and you want to have access to as many functionalities as you can, choose ROS Desktop Install. If you were to install ROS on a limited environment, such as Ubuntu Server (no desktop) on a Raspberry Pi board, then ROS-Base would make sense.

Install ROS Desktop by running the following command:

```
$ sudo apt install ros-distro-desktop
```

Replace **distro** with the distribution name (using lowercase). So, if you want to install ROS Jazzy, you will use the following command:

```
$ sudo apt install ros-jazzy-desktop
```

As you can see, when you run this command, a few hundred new packages will be installed. This can take some time, depending on your internet connection speed, as well as the performance of your computer (or VM).

If you see an error stating _unable to locate package ros-jazzy-desktop_, for example, it probably means you’re trying to install a ROS 2 distribution on the wrong Ubuntu version. This is a common error, so make sure to use the correct Ubuntu/ROS 2 pairing (as seen previously in this chapter).

Note

From this, it’s quite easy to see how to install any other ROS 2 package. You just need to write **ros**, then the distribution name, and then the package name (using dashes, not underscores): **ros-distro-package-name**. For example, if you want to install the **abc\_def** package on ROS Jazzy, then you will need to run **sudo apt** **install ros-jazzy-abc-def**.

Once the installation is done, you can also install the ROS development tools. We’ll need these in the remainder of this book to compile our code and create ROS programs. For this command, no ROS distribution needs to be specified; it’s the same for all of them:

```
$ sudo apt install ros-dev-tools
```

Once you’ve done this, ROS 2 will be installed, and you’ll have all the ROS 2 tools you need.

I also recommend that you frequently update the ROS 2 packages you’ve installed. To do so, simply run **sudo apt update** and **sudo apt upgrade**, like you would update any other package.

## Setting up the environment for ROS 2

At this point, open a Terminal and run the following command:

```
$ ros2
ros2: command not found
```

You will get an error message saying that the **ros2** command can’t be found. As we will see later, **ros2** is a command-line tool we can use to run and test our programs from the Terminal. If this command isn’t working, it means that ROS 2 hasn’t been set up correctly.

Even if ROS 2 is installed, there’s one more thing you need to do in every new session (or Terminal) where you want to use ROS 2: you need to **source** it in the environment.

## Sourcing ROS 2 in the environment

To do that, source this bash script from where ROS 2 is installed:

```
$ source /opt/ros/distro/setup.bash
```

Replace **distro** with the current distribution name you are using. For ROS Jazzy, run the following command:

```
$ source /opt/ros/jazzy/setup.bash
```

After you run this, try executing the **ros2** command again. This time, you should get a different message (usage message). This means that ROS 2 is correctly installed and set up in your environment.

Note

We will see how to use the **ros2** command line later in this book. For now, you can just use it to see if ROS 2 has been set up correctly or not.

## Adding the source line to the .bashrc file

You must source this bash script every time you open a new session or Terminal. To make things easier and so you don’t forget about it, let’s just add this command line to the **.****bashrc** file.

If you don’t know what **.bashrc** is, simply put, it’s a bash script that will run every time you open a new session (that can be an SSH session, a new Terminal window, and so on). This **.bashrc** file is specific to each user, so you will find it in your home directory (as a hidden file because of the leading dot).

You can add the source line to the **.bashrc** file with this command:

```
$ echo 'source /opt/ros/distro/setup.bash' >> ~/.bashrc
```

Replace **distro** with your ROS distribution name. You could also just open the **.bashrc** file directly with any text editor—gedit, nano, or Vim—and add the source line at the end. Make sure it’s only added once.

Once you’ve done this, any time you open a new Terminal, you can be sure that this Terminal is correctly sourced, and thus you can use ROS 2 in it.

Now, to make a final check, open a Terminal and run this command (there’s no need to understand anything for now; it’s just to verify the installation):

```
$ ros2 run turtlesim turtlesim_node
```

This will print a few logs, and you should see a new window with a turtle in the middle. To stop the program, press _Ctrl_ + _C_ in the Terminal where you ran the command.

That’s it for installing and configuring ROS 2. I will just give you a few more tips on what development tools can be useful when developing with ROS.

## Extra tools for ROS development

Apart from the mandatory previous steps to install and set up ROS 2, any other development tool is up to you. If you have your favorite way of working with a Terminal, your favorite text editor, or your favorite **integrated development environment** (**IDE**), this is perfectly fine.

In this section, I will show you a few tools that lots of ROS developers use (me included) and that I think can help you get a better experience when developing with ROS.

## Visual Studio Code

**Visual Studio Code** (**VS Code**) is a quite popular IDE used by many developers. What makes it nice for us is its good support for ROS development.

VS Code is free to use and open source; you can even find its code on GitHub. To install VS Code, open a Terminal and run the following command:

```
$ sudo snap install code --classic
```

The installation just requires one line and uses Ubuntu’s Snap feature. After installing it, you can open VS Code by searching for it in the Ubuntu applications, or simply by running **code** in the Terminal.

Now, start VS Code and go to the **Extensions** panel—you can find it on the left menu.

There, you can search for the ROS extension by typing **ros**. There are quite a few; choose the one developed by Microsoft. This extension is compatible with both ROS 1 and ROS 2, so there’s no problem here:

![[attachments/B22403_02_5.jpg]]

Figure 2.5 – The ROS extension to install in VS Code

Install this extension. This will also install a bunch of other extensions, notably Python and C++ extensions, which are quite useful when writing code.

On top of this, I also usually install the **CMake** extension by **twxs** (just type **cmake** and you’ll find it). With this, we get nice syntax highlighting when writing into **CMakeLists.txt** files, which is something we will have to do quite often with ROS 2.

## The Terminal and other tools

As you develop with ROS 2, you will often need to open several Terminals: one for compiling and installing, a few to run the different programs of your application, and a few more for introspection and debugging.

It can become quite difficult to keep track of all the Terminals you use, so as best practice, it’s nice to have a tool that can easily handle multiple Terminals in one window.

There are quite a few tools for doing this. The one I’m going to talk about here is called **terminator**. Not only does it have a funny name, but it’s also super practical to use.

To install **terminator**, run the following command:

```
$ sudo apt install terminator
```

Then, you can find it from the applications menu, run it, right-click on the left bar menu, and choose **Pin to Dash** so that it stays there and becomes easy to start.

You can find all the commands for **terminator** online, but here are the most important ones to get started:

-   _Ctrl_ + _Shift_ + _O_: split the selected Terminal horizontally.
-   _Ctrl_ + _Shift_ + _E_: split the selected Terminal vertically.
-   _Ctrl_ + _Shift_ + _X_: make the current Terminal fill the entire window. Use again to revert.
-   _Ctrl_ + _Shift_ + _W_: close a Terminal.

![[attachments/B22403_02_6.jpg]]

Figure 2.6 – Terminator with four Terminals

Whenever you split a Terminal, this Terminal becomes two different Terminals, each being one session. Thus, you can easily split your Terminal into four or six; this will be enough to run most of your ROS 2 applications. Since we previously added the line to source ROS 2 in the **.bashrc** file, you can use ROS 2 directly in each new Terminal.

## Summary

When choosing a ROS 2 distribution, I recommend that you pick the latest LTS distribution, given that it’s a few months old and contains all the functionalities you need.

To install and set up ROS 2, you first need to have Ubuntu installed. Each ROS 2 distribution is linked to a specific Ubuntu distribution.

For ROS 2 Jazzy, you must install Ubuntu 24.04. The best option is to have it natively with a dual boot, but to get started quickly, you can also choose to install it in a VM. Then, you can install the ROS 2 packages.

After this, it’s important to source the environment for ROS 2 by sourcing a bash script from the ROS installation folder. You can add the line to source this script into your **.bashrc** file so that you don’t need to do this every time you open a new Terminal.

Finally, to get a better development experience with ROS, I suggest using VS Code with the ROS extension, and a tool that allows you to split the Terminal into multiple Terminals, such as **terminator**.

With this setup, you are fully ready to start using ROS 2. In the next chapter, you will run your first ROS 2 programs and discover the core concepts.

<table id="table001-11"><colgroup></colgroup><colgroup><col></colgroup><colgroup><col></colgroup><tbody><tr><td><h4>Unlock this book’s exclusive benefits now</h4><p>This book comes with additional benefits designed to elevate your learning experience.</p></td><td rowspan="2"><p><img alt="" width="246" height="238" src="attachments/9781835881408.png"></p><p lang="en-US" xml:lang="en-US"><a href="https://www.packtpub.com/unlock/9781835881408" target="_blank" rel="noopener noreferrer">https://www.packtpub.com/unlock/9781835881408</a></p></td></tr><tr><td><p><em>Note: Have your purchase invoice ready before </em><span><em>you begin.</em></span></p></td></tr></tbody></table>