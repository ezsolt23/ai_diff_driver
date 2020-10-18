#!/bin/bash

python3 -m vent ./venv

pip3 install slacklient tensorflow keras keras-rl

python_bin=`pipenv --venv`

echo $python_bin

ln -s $python_bin/bin/python ./python
chmod u+x ./python
