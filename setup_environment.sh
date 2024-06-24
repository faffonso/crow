#!/bin/bash

# # Define the name of the virtual environment
# VENV_NAME="env"

# # Create the virtual environment
# mkdir external
# cd external
# python3 -m venv $VENV_NAME

# # Activate the virtual environment
# source $VENV_NAME/bin/activate

# Install required packages using pip
pip install -r requirements.txt

echo "Virtual environment '$VENV_NAME' created and packages installed successfully."
cd ../