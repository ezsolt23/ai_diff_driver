#!/bin/bash

python_bin=`pipenv --venv`

echo $python_bin

ln -s $python_bin/bin/python ./python
chmod u+x ./python
