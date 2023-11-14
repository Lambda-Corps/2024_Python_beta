# Introduction
This repository is putting together a simulator codebase prepping for the 2024 FRC season.  It is intended to use the beta releases of robotpy, with Phoenix 6 beta from CTRE. The robot is going to simulate a 4-Falcon drivebase, with a Navx Gyro for rotation using Command-Based programming.

## Installation instructions
The following instructions are the latest beta packages as of this document, ensure the packages don't have newer beta versions to ensure you're testing the latest code.  You can go to https://pypi.org/search/ to search for the package name, and on the left side there is a link to the release history.
* ```pip install robotpy==2024.0.0b3```
* ```pip install robotpy-navx==2024.0.0b1```
* ```pip install robotpy-commands-v2==2024.0.0b3```
* ```pip install phoenix6```
* ```pip install robotpy-ctre==2024.0.0b1```

## Virtual Environment Instructions
You can test the beta packages side by side a normal Python installation by using virtual environments. The virtual environment will let you install the packages, activate the environment, and run the code against the different set of python packages.
* Create the virtual environment directory named ***venv***
    ```python3 -m virtualenv venv```
* Acivate the environment to be your new isolated environment
    ```source venv/bin/activate```
After success activation you will see the prompt now include your directory name ***(venv)***

When you are done with the virtual environment, you simply need to deactivate it to come back to your normal shell session. If you want to re-activate the virtual environment, run the above source command again.  To deactivate, simply execute the deactivate command.
    ```deactivate``` at which point you'll see the prompt change to no longer have the ***(venv)*** prefix.
* Verify that your python installation is coming from the virtual environment.  The command result should include your current directory with the virtual environment addition.
    ```which python```
    As an example, the results of a successful virtual environment will look as follows:
    ```
    (venv) user@system 2024_Python_beta % which python                               
    /Users/user/robotics/2024_Python_beta/venv/bin/python
    ```
* Install the robotpy, navx, commands, and phoenix 6 packages as above:
* ```pip install robotpy==2024.0.0b3```
* ```pip install robotpy-navx==2024.0.0b1```
* ```pip install robotpy-commands-v2==2024.0.0b3```
* ```pip install phoenix6```
* ```pip install robotpy-ctre==2024.0.0b1```