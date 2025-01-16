
# <span style="color:rgb(213,80,0)">MATLAB Support for Niryo Ned2 Robots</span>

With the Robotics System Toolbox and ROS Toolbox, you can easily connect to and control Niryo Ned2 Robots, leveraging MATLAB's advanced capabilities in robotics and automation.

This repository requires [MATLAB](https://www.mathworks.com/products/matlab.html)® R2024b or later, [Robotics System Toolbox](https://www.mathworks.com/products/robotics.html) and [ROS Toolbox](https://www.mathworks.com/products/ros.html).

## Why Use Niryo Ned2 Robots with MATLAB?
-   **Interactive Learning:** Experiment with real\-world robotics applications, bridging the gap between theory and practice. Simulate, control, and visualize robot operations all in one place. 
-   **Project\-based exploration:** Create projects that challenge creativity and problem\-solving skills, from basic control tasks to advanced AI\-driven robotics. 
-  **Comprehensive Curriculum:** Leverage MathWorks' extensive educational resources to design courses that cover fundamental to advanced robotics concepts. Use real\-time data and simulations to make abstract concepts tangible. 
# Getting started

To get started, install and look at *FirstSetup.mlx* for more information on using the toolbox.

| **File** <br>  | **Description** <br>   |
| :-- | :-- |
| Ned2\_Simulation.mlx <br>  | Control a simulated model of Niryo Ned2. <br>   |
|  | <img src="README_media/image_0.gif" width="628" alt="image_0.gif"> <br>   |
| Ned2\_Hardware.mlx <br>  | Control a real Niryo Ned2 robot (requires ROS Toolbox) <br>   |
|  | <img src="README_media/image_1.gif" width="627" alt="image_1.gif"> <br>   |


## Core Functions
| **Function** <br>  | **Description** <br>   |
| :-- | :-- |
| SetPose() <br>  | Control robot joint angles. <br>   |
| PlanTrajectory() <br>  | Plan a trajectory that follows multiple waypoints <br>   |
| Move() <br>  | Use Inverse Kinematics to find a joint configuration for a given XYZ position <br>   |

## Features to Explore
-  [**Stateflow**](https://www.mathworks.com/help/stateflow/getting-started.html): Develop complex logic and state machines for advanced control systems, making your robot smarter and more autonomous. 
-  [**Motion and Path Planning**](https://www.mathworks.com/help/robotics/motion-and-path-planning.html): Experiment with different path planning algorithms to optimize robot movements with collision avoidance, enhancing efficiency and performance. 
-  [**Image Processing**](https://www.mathworks.com/help/images/index.html) **and** [**Computer Vision**](https://www.mathworks.com/help/vision/recognition-object-detection-and-semantic-segmentation.html): Integrate sensors and use MATLAB's image processing capabilities to add perception to your robot, enabling it to interact with its environment intelligently. 

# Related Courseware Modules
### 
<div style="display: flex">
  <table>
    <tr>
      <td colspan="2" style="text-align:center;">
        <a href="https://www.mathworks.com/matlabcentral/fileexchange/130124-robotic-manipulators">Robotic Manipulators</a>
      </td>
      <td colspan="2" style="text-align:center;">
        <a href="https://www.mathworks.com/matlabcentral/fileexchange/136364-applied-linear-algebra">Applied Linear Algebra: Robotics</a>
      </td>
    </tr>
    <tr>
      <td style="text-align:center;">
        <img src="README_media/image_2.png" width="146" alt="image_2.png"><br>
      </td>
      <td>
        Available on:<br>
        <a href="https://github.com/MathWorks-Teaching-Resources/Robotic-Manipulators">
          <img src="README_media/image_3.png" width="81" alt="image_3.png">
        </a><br>
        <a href="https://matlab.mathworks.com/open/github/v1?repo=MathWorks-Teaching-Resources/Robotic-Manipulators&project=RoboticManipulators.prj">
          <img src="README_media/image_4.png" width="131" alt="image_4.png">
        </a><br>
        <a href="https://github.com/MathWorks-Teaching-Resources/Robotic-Manipulators">GitHub</a><br>
      </td>
      <td style="text-align:center;">
        <img src="README_media/image_5.png" width="147" alt="image_5.png"><br>
      </td>
      <td>
        Available on:<br>
        <a href="https://www.mathworks.com/matlabcentral/fileexchange/136364-applied-linear-algebra">
          <img src="README_media/image_6.png" width="81" alt="image_6.png">
        </a><br>
        <a href="https://matlab.mathworks.com/open/github/v1?repo=MathWorks-Teaching-Resources/Applied-Linear-Algebra&project=AppliedLinAlg.prj">
          <img src="README_media/image_7.png" width="131" alt="image_7.png">
        </a><br>
        <a href="https://github.com/MathWorks-Teaching-Resources/Applied-Linear-Algebra">GitHub</a><br>
      </td>
    </tr>
  </table>
</div>

Explore our other [<u>modular courseware content</u>](https://www.mathworks.com/matlabcentral/fileexchange/?q=tag%3A%22courseware+module%22&sort=downloads_desc_30d).

# Self\-Paced Online Courses
| [**Multibody Simulation Onramp**](https://matlabacademy.mathworks.com/details/multibody-simulation-onramp/ormb) <br>  | [**MATLAB Onramp**](https://matlabacademy.mathworks.com/details/matlab-onramp/gettingstarted) <br>  | [**Stateflow Onramp**](https://matlabacademy.mathworks.com/details/stateflow-onramp/stateflow) <br>   |
| :-: | :-: | :--: |
| <img src="README_media/image_8.png" width="171" alt="image_8.png"> <br>  | <img src="README_media/image_9.png" width="172" alt="image_9.png"> <br>  | <img src="README_media/image_10.png" width="171" alt="image_10.png"> <br>   |

# License
The license is available in the <samp>license.txt</samp> file within this repository.

# Contribute
Looking for more? Find an issue? Have a suggestion? Please contact the [<u>MathWorks teaching resources team</u>](mailto:%20onlineteaching@mathworks.com). If you want to contribute directly to this project, you can find information about how to do so in the [<u>CONTRIBUTING.md</u>](./CONTRIBUTING.md) page on GitHub.

Copyright 2024-2025 The MathWorks, Inc.