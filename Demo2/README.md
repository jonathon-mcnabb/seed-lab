This folder contains the code for Demo2.

* `main.py`: This file is all the code that is ran for the computer vision.
* `demo_2_controller/`: This folder contains the Arduino code for the implementation of the Demo2 feedback controller.
    * `functions.h` -- a helper file that just contains useful functions that do not need access to any global videos
    * `constants.h` -- a helper file that holds all the `#DEFINE`s for the project.
    * `points_buffer.h ` -- a helper file that handles operating the queue used for keeping track of points in the path the robot is meant to draw out.
    * `demo_2_controller.ino` -- the main shebang. Contains the `setup()` and `loop()` functions that handle the majority of the work. Also contains a few helper functions to make `loop()` less huge.
