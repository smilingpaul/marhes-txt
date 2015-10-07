# TXT1 LPC2378 Development Environment Setup #

---

## Introduction ##

This page introduces the user on how to setup the TXT1 LPC2378 Development Environment on Ubuntu.  This environment is needed in order to program and make changes to the lower lever controller on the TXT1 robotic platforms.  These platforms are a part of UNM MARHES Research Laboratory.


## Setting up the Development Environment ##

Before we begin, the correct hardware needs to be obtained.

**Things Needed**
  * Olimex LPC2378 Development Board (ordered from Sparkfun)
  * Olimex ARM-USB-OCD JTAG Programmer/Debugger (ordered from Sparkfun)
  * Computer with Ubuntu installed (these instructions for Ubuntu 10.04)
    * Internet Connection
    * USB Port

**Setup Steps**
  1. Install OpenOCD
    * Open a terminal
    * Type `sudo apt-get install openocd`
  1. Install CodeSourcery Lite for ARM EABI
    * Go to http://www.codesourcery.com/sgpp/lite/arm/portal/subscription3053
    * Download this version: Lite 2010q1-188
    * Download the IA32 GNU/Linux Installer
    * In the terminal, type `sudo dpkg-reconfigure -plow dash`
    * Select _No_
    * Type `/bin/sh ./"path to package"/arm-"version"-arm-none-eabi.bin` where `"path to package"` the path to the downloaded installer and `"version"` is the version of the installer.
    * Go through the installation steps
      * Typical
      * Directory: `/home/marhes/CodeSourcery/Sourcery_G++_Lite`
      * Modify PATH
  1. Install Eclipse (Optional is you don't like IDEs)
    * Go to http://www.eclipse.org/downloads/
    * Click _Eclipse IDE for C/C++ Developers_
    * Download the correct type of eclipse for your computer
    * Extract the archive to `/home/marhes`
  1. Install Eclipse Addons
    * Install CDT
      * Open Eclipse from the terminal by typing `~/eclipse/eclipse`
      * Create your workspace for the microcontroller project
      * Go to _Help -> Install New Software_
      * Click on _Add_
      * In the name field, enter _CDT_
      * In the location field, enter _http://download.eclipse.org/tools/cdt/releases/helios_
      * Select _OK_
      * Click both checkmarks for _CDT Main Features_ and _CDT Optional Features_
      * Click _Next, Next, Agree, Finish_
      * Restart Eclipse
    * Install Zylin CDT
      * Do the same as above except in the name field enter _ZylinCDT_ and in the location field enter _http://www.zylin.com/zylincdt_

This finishes the development environment setup.

## Setting Up a Project ##

This is how to setup Eclipse for a microcontroller project

  1. Create a Project
    * In Eclipse, go to `File -> New -> Makefile Project with Existing Code`
    * Then browse for the folder where the existing code is
    * Select C Only
    * Make sure the toolchain is _None_
    * Select _Finish_
  1. Setup how the code compiles
    * Go to _Project -> Properties_
    * Expand _C/C++ Build_
    * Click _Environment_
    * Click _Add_
    * In the Name field, enter _PATH_
    * In the Value field, enter `/home/marhes/CodeSourcery/Sourcery_G++_Lite/bin`
    * Click _Enter_ then _Apply_
    * Next try compiling your project
  1. Configure the external tool
    * Go to _Run -> External Tools -> External Tools Configurations..._
    * Add new launch configuration
    * In the Name field, enter `OpenOCD`
    * In the Location field, enter `/usr/bin/openocd`
    * In the Working Directory field, enter `${workspace_loc:/marhes-txt}`
    * In the Arguements field, enter `-f olimex-arm-usb-ocd.cfg -f lpc23xx.cfg`
    * Go to the Common tab
    * Check the `External Tools` checkbox
    * Click _Apply_ and then _Close_

This concludes setting up the Eclipse project