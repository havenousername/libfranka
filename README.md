# libfranka: C++ library for Franka Emika research robots

[![Build Status][travis-status]][travis]
[![codecov][codecov-status]][codecov]


This is fork of official libfranka c++ library for the Franka Emika robots. See the [Franka Control Interface (FCI) documentation][fci-docs] for more information about what `libfranka` can do and how to set it up. The [generated API documentation][api-docs] also gives an overview of its capabilities. 

The target of this documentation is to explain which parts of the original codebase were extended and how to run those scripts. For this task you need to have an operational robot (no emulated environment!) and the connection to the computer with the host address of the robot (to send the commands).

## Changes added 
The subject of this patch was to add certain scripts from the assignment 3 of the Intelligest Machine Programming Lab in TUM. Scripts could be found in the directory `examples` under with the following regex pattern `examples/0x_program_name.cpp`. Before trying out tasks, please ensure that you have finished all steps which are defined in the [documentation](https://frankaemika.github.io/docs/getting_started.html).

## How to run each script
To run in terminal do:
1. `cd build`
2. `cmake .. && make`
3. `./0x_file_name.cpp` run file which is of interest. 

## Tasks
   
###  1. Create arbitrary cpp file and run it on the robot
**Source Code: `examples/01_something.cpp`**.

For this task we have created simple cpp file with one cout statement.

###  2. Trajectory recording 
**Source Code:  `examples/02_trajectory_recording.cpp`**.

1. For this task we created `examples/json_utils.cpp` small library with helpful methods like creating a json file or transforming from the array data types of the positions to the JSON string
2. Trajectory recording happens each type read event listener is called. We use intervals to optimize recording procedure. 
3. After the recording is finished, the robot is left in free guiding mode
###  3. Infinite sign trajectory
**Source Code:  `examples/03_generate_infinite_sign.cpp`**.


For this task we have used our knowledge of mathematical functions, specifically the analytical self-repeating sin function to create a infinite movement along the spiral trajectory.
We made two versions using two different approaches, however, only sin version was recorded: [sine infinite sign movement](https://drive.google.com/file/d/1BOk-PA2bxK_l6TTx6VPaWiDZS4XZZE6J/view). 

Cosine version of the program has a higher velocity. However, both strategies could be controlled via their versions of constants. You can switch between strategies using `MotionStrategy` enum.


###  4. Replay trajectory and record it again
**Source Code:  `examples/04_replay_trajectory.cpp`**.

In this task we had to generate trajectory which was recorded in
the step 2 and record the trajectory which we get as the result of
the movement. Point of this task is to have 2 different json files in the
end, so that later we can analyse the impact of the error coming from the robot. 
We have modified impedance control from file `examples/cartesian_impedance_control.cpp` and used `examples/json_utils.cpp`
library to handle IO and serialization tasks.

### 5. Analysis and plotting 
**Source Code:  `examples/05_generate_plot_trajectory.ipynb`**.

In this step we have plotted the changes of positions artifacts of step 2 and step 4 respectively, 
to check how well the recorded trajectory could be recreated. For more on analysis you have address the documentation document `doc/IMPLAssignment3.pdf`.   


## License

`libfranka` is licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[api-docs]: https://frankaemika.github.io/libfranka
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/libfranka.svg?branch=master
[travis]: https://travis-ci.org/frankaemika/libfranka
[codecov-status]: https://codecov.io/gh/frankaemika/libfranka/branch/master/graph/badge.svg
[codecov]: https://codecov.io/gh/frankaemika/libfranka
