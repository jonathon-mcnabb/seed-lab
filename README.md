# SEED Lab
The Github repository for Group 9:
* Gabriel Alcantar-Lopez
* Gregory Meklo
* Jonathon McNabb
* Luke Henke

# Organization
Assignment submissions are grouped by folders.

* `Assignment_1_and_2/`
    * `computer-vision/`
        * contains the python code for the computer vision subsystem
* `Mini Project`
    * `mini_deliverables/` contains the some of the sub-deliverable solutions that were used in the buildup for the main solution
    * `final_controller.ino` is the main controller for the mini project
    * `motor_simulation.ino` is the Arduino code that records the step response of the wheel
    * `rad.py` is the Python code for discovering the quadrant the marker is contained in.
* `Demo1/`
    * `cv_angle.py` is the python code for measuring and displaying the angle on the LCD
    * `arduino_controller/` contains the Arduino code for the final controller, split apart into several files:
        * `arduino_controller.ino` is the 'main' file that is loaded
        * `constants.h` contains all the `#DEFINE`s
        * `functions.h` contains generic functions that are useful in many files (e.g. `iController`, `pController`)
        * `phi_controller.h` contains the inner/outer angle control
        * `rho_controller.h` contains the inner/outer forward movement control


Additionally, `notify_on_startup` contains the python script ran on startup.
