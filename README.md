# ai-motor-driver
The goal of this project is to create an extra layer between the cmd_vel ROS topic and the wheel driver.
The aim of this layer is to be an AI based wheel driver. This way you don't need to fine tune the
wheel driver, and then re-tune to different surfaces like carpet, wood floor or outside terrain, but
it can learn by itself how to move the wheels the best way.

# Requirements

- You need to have ROS1 installed and configured. 
- Tested on ROS-melodic, Ubuntu 18
- Python 3.6+ is required

# Installation

    pip3 install pipenv

    git clone https://github.com/ezsolt23/ai_diff_driver.git
    cd ./ai_diff_driver/src
    ./init_dependencies.sh

# Training the network offline

First set up your wheel size (tyre_circumference) and distance between the middle of your wheels (wheel_track).
Then you can do some fine tuning in calcVelocities() to adjust the function to your wheel parameters. I am using a 
pair of howerboard wheels, so it assumes you have a similar wheel.

To start the simulation launch this file:

    ./start_simulator.sh
    
# Running the AI wheel driver

You need to run two nodes. One is a very basic driver to ODrive which sends velocity commands to the wheels based on
the messages received from **cmd_wheel_current_l** and **cmd_wheel_current_r** ROS topics. The two topics are fed by 
the other node, ai_driver_node, which you can start with start_ai_node.sh. You can reimplement the odrive_node to
match your specific wheel. It should be refactored to an external repository, but now it is good here I think.

# Special Thanks

I have written this project in memory of my father, Zoltán Egri. We started our first robot project together back in 2002
and he was always a great colleague, excellent designer and engineer and a beloved father. Unfortunately he passed away in May 04 2020
so he could never see the finished project. I have written and open sourced this project as a honor to my beloved father. 
I love you dad! Rest in peace!
