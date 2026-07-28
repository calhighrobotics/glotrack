# Glotrack

> Simple global field localization providing a ground-truth reference, designed for VEX Robotics

Glotrack is built for aspiring programmers to test out their own localization algorithms by providing a ground-truth pose reference. It's built upon [OpenCV](https://opencv.org/) for the Apriltag detection and [Dear ImGui](https://github.com/ocornut/imgui) for the UI.

## Quick Start

Download the binary for your system from Github Releases and run!
Alternatively, Glotrack can be built from source.

Follow these steps to use the program:

1. Setup a camera between 5 and 10 feet away from the field. Ideally, it should be facing normal to the field plane for maximum tracking accuracy.
2. Print out 5 16h5 Apriltags, IDs 0 through 4. Any online generator will work, but I recommend the [tool created by Limelight](https://tools.limelightvision.io/apriltag-generator). Place Apriltags 1 through 4 around the field in the frame of the camera you just setup. They should be as far away as possible from each other to best capture your field's geometry. Apriltag ID 0 will be reserved for your robot.
3. Launch Glotrack. If you're on a laptop with an integrated camera, you will probably want to switch to *Camera 2* to select the external camera.
4. Calibrate the camera, toggling the upper right checkbox if the window doesn't automatically appear upon first launch. Print out [this calibration template](https://github.com/opencv/opencv/blob/5.x/doc/pattern.png). Place the checkerboard pattern in the camera frame at various angles and capture images. Make sure that the pattern is flat and not distorted in any way. When enough images have been gathered, calculate the intrisnic camera matrices. This is a mathematical representation of the camera lens' distortion that we will use to flatten out the image later.
5. Adjust the localization parameters in the bottom right window, and then click *Start Localization Pipeline*! You should be able to see a 2D representation of the robot's movement on the left side graph.
6. Start logging the robot's position to the terminal or to a file!

### Building from source

After installing these dependencies (`OpenCV OpenGL glfw3`) and cloning the repository, run the following commmands in the repository's root directory:

1. `cmake -B build`.
2. `cmake --build build`.

The path of the generated executable is `./build/tracker`.

## License

Copyright (C) 2026 DataTopping. This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.
