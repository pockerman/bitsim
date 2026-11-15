\page bitrl_example_31 BitRL Example 31: A line following robot with a Raspberry Pi camera

In \ref bitrl_example_30 real motors moved a simulated robot. Here a real camera guides it. A
Raspberry Pi 5 with a camera looks down at a sheet of paper with a square drawn on it and
streams what it sees. That picture is the floor of the simulated world. The differential drive
robot of \ref bitrl_example_14 carries a camera that looks down at the floor just ahead of it.
It sees a patch of the latest picture, finds the line in it, and steers to follow the line
around the square.

The loop runs live: the robot always sees the latest picture, so if the paper moves while it
drives, it sees the line where the line is now.

\f[
\text{Pi camera} \xrightarrow{\text{MJPEG over HTTP}} \text{floor}
\xrightarrow{\text{robot pose}} \text{robot camera image}
\xrightarrow{\text{line}} (v, \omega) \xrightarrow{\text{Chrono}} \text{new pose}
\f]

## What you need

- A Raspberry Pi 5 with a camera module, for example the Camera Module 3, and Raspberry Pi OS.
- A sheet of paper with a square on it in thick dark lines. Black electrical tape works well.
  The line should be about 3 to 5 % of the picture's height wide, and the square should fill
  about half the picture.
- A stand that holds the camera above the paper, looking straight down, so that it sees the
  whole square.
- bitrl built with Chrono and OpenCV, see \ref bitrl_install.

Without a Pi, the camera server draws a square itself; see \ref bitrl_example_31_run.

## The pieces

| Piece | Where | What it does |
| ----- | ----- | ------------ |
| `camera_server.py` | the Pi | FastAPI app that streams the camera's frames as MJPEG at `/stream` |
| \ref bitrl::sensors::backends::MJPEG_HTTPCameraBackend "MJPEG_HTTPCameraBackend" | bitrl | receives the stream on a thread of its own and keeps the latest frame |
| \ref bitrl::sensors::FloorImage "FloorImage" | bitrl | the picture as a floor: which pixel is at which point of the world |
| \ref bitrl::sensors::DownwardCamera "DownwardCamera" | bitrl | the robot's camera: what it sees of a floor from a pose |
| \ref bitrl::rb::bitrl_chrono::CHRONO_DiffDriveRobot "CHRONO_DiffDriveRobot" | bitrl | the robot, with its camera read from the robot file |
| `find_square`, `read_line`, `LineFollower` | `example_31.cpp` | find the square, see the line, steer |

## The camera server

`camera_server.py` has three endpoints:

| Endpoint | Gives |
| -------- | ----- |
| `GET /stream` | a `multipart/x-mixed-replace` response: one JPEG after another, as long as you read |
| `GET /frame` | the latest frame as one JPEG |
| `GET /info` | the source, frame size, frame rate and how many frames it has taken |

A thread of its own reads the camera and keeps the latest frame as JPEG. Each client of
`/stream` gets that frame whenever it changes, at most `--fps` times a second, 20 by default.
A slow client skips frames instead of falling behind. On the Pi the frames come from
<a href="https://github.com/raspberrypi/picamera2">Picamera2</a>. With `--synthetic` the server
draws a square on paper instead: slightly rotated, lit unevenly, noisy and drifting a little,
so that the simulator does not get a perfect picture.

Open `http://raspberrypi.local:8000/stream` in a browser to see what the camera sees.

## The floor

The picture covers a floor `--floor-width` metres wide, 3 m by default, centred on the world
origin. A 640 x 480 picture makes the floor 3 m x 2.25 m, about 4.7 mm per pixel. The square
of the synthetic camera is then 1.36 m across and its line 8.5 cm wide, a good size for a robot
with its wheels 0.4 m apart.

At the start the simulator looks for the square in the first picture. It is the largest dark
outline with a hole in it. Halfway between its outer and inner edges is the centre line. The
robot starts in the middle of the bottom side, facing along it with the inside of the square on
its left, so it goes around counterclockwise. The square is used only for the start and to
measure how well the robot follows the line; the robot itself sees only through its camera.

## The robot's camera

The robot file `robots/bitrl_diff_drive_robot.json` describes the camera:

| Setting | Value | Meaning |
| ------- | ----- | ------- |
| `type` | `downward-camera` | a camera that looks straight down |
| `offset` | 0.35 m | how far ahead of the axle centre the middle of its view is |
| `view_length` | 0.3 m | how much floor it sees along the robot's heading |
| `view_width` | 0.4 m | and across it |
| `image_width`, `image_height` | 160 x 120 | the size of its image |
| `update_rate` | 20 /s | images per second |

The top row of its image is the far edge of the view and column 0 is the robot's left, as a
camera that faces forward and tilts down would show it.
\ref bitrl::sensors::DownwardCamera::view "view" turns the pose of the robot into an affine map
from the camera's pixels to the floor's pixels and samples the picture with it, so it costs
the same whatever the pose. \ref bitrl::sensors::DownwardCamera::to_robot "to_robot" goes the
other way, from a pixel of the camera's image to metres ahead of and to the left of the robot.

## Seeing the line

The paper is not equally bright everywhere, so a fixed threshold would not work under every
lamp. The paper's brightness is taken as the level that 90 % of the pixels are darker than, and
a pixel is line if it is darker than 0.6 times that. The middle of the line pixels, in robot
coordinates, is \f$(f, l)\f$: \f$f\f$ metres ahead and \f$l\f$ metres to the left. If less than
1 % of the image is line, the camera has lost it.

## Steering

The robot uses pure pursuit. It drives along the circle that starts where it is, heading the
way it faces, and passes through \f$(f, l)\f$. That circle has the curvature

\f[
\kappa = \frac{2 l}{f^2 + l^2}
\f]

so with forward speed \f$v\f$ the turn rate is \f$\omega = v \kappa\f$. On a straight side the
line is ahead, \f$l \approx 0\f$, and the robot drives straight. In a corner the camera sees the
line bend away to the left. The middle of the line pixels moves left, \f$l\f$ grows and the
robot turns. It slows down as \f$|l|\f$ grows, to 30 % of its cruise speed of 0.25 m/s.

If the camera loses the line the robot stops and turns towards where it last saw the line.
After 5 s without it, it gives up.

Because the camera looks 0.35 m ahead, the robot starts to turn before its axle reaches the
corner, so the axle centre cuts the corners by a few centimetres. The farther ahead the camera
looks, the earlier the robot sees a corner and the more it cuts it.

## Real time

The picture is real, so the simulation keeps to real time: after each step it waits until the
wall clock catches up. The robot takes a camera image every 50 ms of the latest frame. With the
server on the same computer that frame is on average about 25 ms old; over WiFi from a Pi,
expect more.

If the stream stops, for example when the Pi goes off the network, the robot stops. The
backend opens the stream again every half second, and once frames arrive the robot drives on.

\anchor bitrl_example_31_run
## Running the example

On the Pi, install Picamera2 and OpenCV from Raspberry Pi OS, and FastAPI and uvicorn in a
virtual environment that can also see them:

@code{.sh}
sudo apt install python3-picamera2 python3-opencv
python3 -m venv --system-site-packages ~/bitrl-camera
~/bitrl-camera/bin/pip install fastapi uvicorn
~/bitrl-camera/bin/python camera_server.py
@endcode

`camera_server.py` is in `examples/example_31/pi_camera_server`; copy it to the Pi. Then, on the
computer with bitrl:

@code{.sh}
./example_31 --render http://raspberrypi.local:8000/stream
@endcode

Use the Pi's IP address if `raspberrypi.local` does not resolve. `--render` opens a window with
the camera's picture, the square that was found, the robot, what its camera sees, in the top
left corner, and the path of its axle centre.

Without a Pi, run the server on the same computer with generated frames:

@code{.sh}
pip install -r examples/example_31/pi_camera_server/requirements.txt
python3 examples/example_31/pi_camera_server/camera_server.py --synthetic
./example_31 --render http://localhost:8000/stream
@endcode

## Output

With the synthetic camera:

@code
Camera picture 640 x 480 pixels, the floor is 3.000 x 2.250 m
Square: side 1.354 m, line 0.084 m wide, centre (0.023, 0.021)
Start at (-0.012, -0.654) heading -2.981 deg

    t [s]   x [m]   y [m]  head [deg]  line left [m]  v [m/s]  w [rad/s]  off line [m]  picture age [ms]
     0.01  -0.012  -0.654        -3.0         0.0023    0.247      0.010         0.000                16
     1.00   0.179  -0.664        -3.1         0.0227    0.222      0.070         0.000                14
     2.00   0.442  -0.669         3.5         0.0691    0.164      0.373         0.008                10
     3.01   0.559  -0.646        26.2         0.1616    0.075      0.322         0.038                13
     4.01   0.589  -0.622        46.7         0.1413    0.075      0.227         0.063                 9
...
    35.00  -0.055  -0.713        -1.6         0.0088    0.239      0.035         0.062                27

The robot went once around the square in 35.170 s of simulation, 35.168 s of wall time
Distance of the axle centre from the line: mean 0.035 m, largest 0.120 m (the line is 0.084 m wide)
Camera images: 704, without the line in them 0
Stream: 20.0 frames/s, mean age of the picture used 25.1 ms, reconnections 0
@endcode

The robot never lost the line. Its axle centre was 3.5 cm from the centre line on average,
less than half the line's width, and at most 12 cm from it, where it cut the corners.

## Things to try

1. Move the paper slowly while the robot drives. How fast can you move it before the robot loses
   the line?
2. Change `offset` in the robot file to 0.2 m and to 0.5 m. How do the corners and the largest
   distance from the line change?
3. Run the server with `--fps 5`. The robot sees the floor less often and later. What happens at
   the corners, and at what frame rate does it lose the line?
4. Raise the cruise speed, `CRUISE_SPEED` in `example_31.cpp`. Pure pursuit with a fixed look
   ahead has a speed above which it cannot take a corner; find it.
5. Draw a circle instead of a square. `find_square` will not find it; make the start work from
   any closed line.

## The code

The simulator:

\include example_31.cpp

The camera server:

\include camera_server.py
