# ai-motor-driver

# Requirements

- You need to have ROS1 installed and configured. 
- Tested on ROS-melodic, Ubunu 18
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
the messages recoeved from **cmd_wheel_current_l** and **cmd_wheel_current_r** ROS topics. The two topics are fed by 
the other node, ai_driver_node, which you can start with start_ai_node.sh. You can re implement the odrive_node to
match your specific wheel. It should be refactored to an external repository, but now it is good here I think.
