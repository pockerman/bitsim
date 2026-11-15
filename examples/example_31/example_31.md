\page bitrl_example_31 BitRL Example 31: A line following robot with a Raspberry Pi camera

In \ref bitrl_example_30 real motors moved a simulated robot. Here a camera on a Raspberry Pi 5
guides it. The Pi streams pictures of a line drawn on paper, and each picture is the floor of
the simulated world. The differential drive robot of \ref bitrl_example_14 carries a camera that
looks down at the floor just ahead of it. It sees a patch of the latest picture, finds the line
in it, and steers to follow the line.

The pictures come from one of two endpoints of the Pi's server:

| Endpoint | Pictures | The robot |
| -------- | -------- | --------- |
| `/odysseus/camera/square-path`, the default | generated, no camera needed. Every 5 s the picture shows the next side of a square | follows each side, waits at its end and turns onto the next side when it appears |
| `/odysseus/camera`, with `--pi-camera` | the Pi camera, looking down at a square drawn on paper | follows the line around the square |

Either way the loop runs live: the robot always sees the latest picture, so if the line moves
while it drives, it sees the line where the line is now.

\f[
\text{Pi} \xrightarrow{\text{MJPEG over HTTP}} \text{floor}
\xrightarrow{\text{robot pose}} \text{robot camera image}
\xrightarrow{\text{line}} (v, \omega) \xrightarrow{\text{Chrono}} \text{new pose}
\f]

## What you need

- bitrl built with Chrono and OpenCV, see \ref bitrl_install.
- A Raspberry Pi 5 that runs the server, or, without a Pi, `camera_server.py` on the same
  computer; see \ref bitrl_example_31_run.

For `--pi-camera` you also need

- a camera module on the Pi, for example the Camera Module 3,
- a sheet of paper with a square on it in thick dark lines. Black electrical tape works well.
  The line should be about 3 to 5 % of the picture's height wide, and the square should fill
  about half the picture,
- a stand that holds the camera above the paper, looking straight down, so that it sees the
  whole square.

## The pieces

| Piece | Where | What it does |
| ----- | ----- | ------------ |
| the Pi's server, or `camera_server.py` | the Pi | FastAPI app that streams the square path and the camera as MJPEG |
| \ref bitrl::sensors::backends::MJPEG_HTTPCameraBackend "MJPEG_HTTPCameraBackend" | bitrl | receives the stream on a thread of its own and keeps the latest frame |
| \ref bitrl::sensors::FloorImage "FloorImage" | bitrl | the picture as a floor: which pixel is at which point of the world |
| \ref bitrl::sensors::DownwardCamera "DownwardCamera" | bitrl | the robot's camera: what it sees of a floor from a pose |
| \ref bitrl::rb::bitrl_chrono::CHRONO_DiffDriveRobot "CHRONO_DiffDriveRobot" | bitrl | the robot, with its camera read from the robot file |
| `find_square`, `find_segment`, `read_line`, `LineFollower` | `example_31.cpp` | find the line, see it, steer |
| `SquarePathDriver` | `example_31.cpp` | drives the square path side by side |

## The server

The simulator uses the endpoints of the Odysseus robot's server. `camera_server.py`, in
`examples/example_31/pi_camera_server`, has the same endpoints, so you can use it on a Pi of your
own or on any computer:

| Endpoint | Gives |
| -------- | ----- |
| `GET /odysseus/camera/square-path` | the square path: an MJPEG stream of one side of a square at a time |
| `GET /odysseus/camera` | the camera: an MJPEG stream, a `multipart/x-mixed-replace` response of one JPEG after another |
| `GET /odysseus/camera/frame` | the latest camera frame as one JPEG |
| `GET /odysseus/camera/info` | the source, frame size and frame rate |

The square path needs no camera. Its square is centred in the picture and half as high as the
picture. Every `--period` seconds, 5 by default, the picture switches to the next side, going
counterclockwise as seen from above: bottom, right, top, left. Only the current side is drawn.

For the camera, a thread of its own reads the camera and keeps the latest frame as JPEG. Each
client of `/odysseus/camera` gets that frame whenever it changes, at most `--fps` times a
second, 20 by default; a slow client skips frames instead of falling behind. On the Pi the
frames come from <a href="https://github.com/raspberrypi/picamera2">Picamera2</a>. With
`--synthetic` the server draws a whole square on paper instead: slightly rotated, lit
unevenly, noisy and drifting a little, so that the simulator does not get a perfect picture.

## The floor

The picture covers a floor `--floor-width` metres wide, 3 m by default, centred on the world
origin. A 640 x 480 picture makes the floor 3 m x 2.25 m, about 4.7 mm per pixel. The square
of the square path is then 1.125 m long and the line 8.4 cm wide, a good size for a robot with
its wheels 0.4 m apart.

The simulator uses the line in the pictures to start the robot and to measure how well the
robot follows it; the robot itself sees only through its camera.

- On the square path it waits until the picture switches to the next side, so that the robot
  starts with the whole period ahead of it. The side is the largest long, thin dark blob. The
  robot starts at its beginning, facing along it so that the centre of the picture is on its
  left. Each new side is added to the path as it appears.
- With the camera it looks for the square in the first picture: the largest dark outline with
  a hole in it. Halfway between its outer and inner edges is the centre line. The robot starts
  in the middle of the bottom side, facing along it with the inside of the square on its left.

Either way the robot goes around counterclockwise.

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
line is ahead, \f$l \approx 0\f$, and the robot drives straight. It slows down as \f$|l|\f$
grows, to 30 % of its cruise speed.

With the camera the corners of the square are in the picture. The camera sees the line bend
away to the left, the middle of the line pixels moves left, \f$l\f$ grows and the robot turns.
Because the camera looks 0.35 m ahead, it starts to turn before its axle reaches the corner, so
the axle centre cuts the corners by a few centimetres. If the camera loses the line the robot
stops and turns towards where it last saw it. After 5 s without it, it gives up. The cruise
speed is 0.25 m/s.

## The square path

On the square path a corner is never in the picture: the line simply ends until the next side
appears. Pure pursuit cannot see that coming, and turning on the spot where the line leaves the
camera's view, 0.2 m short of the corner, would leave the next side to the side of the robot,
out of its camera's view. But the robot knows one more thing: the path goes counterclockwise,
so the next side always starts at the corner and leaves it to the left. The `SquarePathDriver`
uses that:

1. **Follow** the side with pure pursuit. Once the end of the side comes into view, the top row
   of line pixels is below the top of the image, it notes where the corner is: half a line
   width short of the end.
2. **To the corner**: when the end has passed under the camera, it drives on to the corner by
   its own pose, slowing down as it gets close.
3. **Turn** left on the spot until it faces at right angles to the side it followed. That is
   the side of the path nearest to it, not the latest one: if the robot is late, the next side
   may already be showing.
4. **Wait** until its camera sees a line. The next side starts under the robot and runs straight
   ahead of it.

A side and its corner have to be done before the next side appears, here in 5 s. The side is
1.125 m long, so the robot drives at 0.35 m/s and turns at up to 1.5 rad/s: about 3.2 s for the
side and 1 s for the turn. The turn overshoots by up to about 17 degrees, because the motors
take a moment to stop, and the robot weaves a little as it settles onto the next side. If it is too
slow, the next side appears before it has reached the corner. While it still sees the old side
end it carries on to the corner; if the old side disappears before its end came into view, the
robot cannot know where the corner is, and waits for a line it cannot see. After 15 s it gives
up.

## Real time

The picture is real, so the simulation keeps to real time: after each step it waits until the
wall clock catches up. The robot takes a camera image every 50 ms of the latest frame. With the
server on the same computer that frame is on average about 25 ms old; over WiFi from a Pi,
expect more.

If the stream stops, for example when the Pi goes off the network, the robot stops. The
backend opens the stream again every half second, and once frames arrive the robot drives on.

\anchor bitrl_example_31_run
## Running the example

Give the example the address of the server; `--pi-camera` switches from the square path to the
camera:

@code{.sh}
./example_31 --render http://raspberrypi.local:8000              # the square path
./example_31 --render --pi-camera http://raspberrypi.local:8000  # the Pi camera
@endcode

Use the Pi's IP address if its name does not resolve. `--render` opens a window with the
picture, the line as the simulator knows it, the robot, what its camera sees, in the top left
corner, and the path of its axle centre.

To run `camera_server.py` on a Pi, install Picamera2 and OpenCV from Raspberry Pi OS, and FastAPI
and uvicorn in a virtual environment that can also see them:

@code{.sh}
sudo apt install python3-picamera2 python3-opencv
python3 -m venv --system-site-packages ~/bitrl-camera
~/bitrl-camera/bin/pip install fastapi uvicorn
~/bitrl-camera/bin/python camera_server.py
@endcode

Without a Pi, run it on the same computer. `--synthetic` makes `/odysseus/camera` draw a square
instead of using a camera; the square path is always there:

@code{.sh}
pip install -r examples/example_31/pi_camera_server/requirements.txt
python3 examples/example_31/pi_camera_server/camera_server.py --synthetic
./example_31 --render http://localhost:8000
./example_31 --render --pi-camera http://localhost:8000
@endcode

## Output

On the square path of `camera_server.py`:

@code
Pictures 640 x 480 pixels, the floor is 3.000 x 2.250 m
Side: 1.125 m long, line 0.084 m wide
Start at (0.565, -0.565) heading 90.000 deg

    t [s]   x [m]   y [m]  head [deg]  line left [m]  v [m/s]  w [rad/s]  off line [m]  picture age [ms]
     0.01   0.565  -0.565        90.0         0.0000    0.350      0.000         0.000                 8
     1.00   0.565  -0.295        90.0         0.0000    0.350      0.000         0.000                 3
     2.00   0.565   0.140        90.0         0.0000    0.350      0.000         0.000                49
     3.01   0.565   0.500        90.0           lost    0.150      0.000         0.000                 6
     4.01   0.569   0.567       130.2           lost    0.000      1.425         0.008                 1
     5.01   0.607   0.564      -163.1           lost    0.000      0.000         0.042                46
     6.01   0.511   0.559       176.7         0.0096    0.333      0.053         0.001                43
...
    22.00   0.480  -0.516        42.0         0.1397    0.105      0.355         0.048                12

The robot went once around the square in 22.230 s of simulation, 22.228 s of wall time
Distance of the axle centre from the line: mean 0.023 m, largest 0.067 m (the line is 0.084 m wide)
Camera images: 445, without the line in them 105
Stream: 19.9 frames/s, mean age of the picture used 23.9 ms, reconnections 0
@endcode

The robot started on the right side, which was the side that appeared while it waited. At 3 s
the end of the side had passed under its camera, the line is lost, and it drove on to the
corner and turned. By 6 s the top side had appeared and it followed it. Its axle centre was
2.3 cm from the line on average, and at most 6.7 cm, while it settled onto a side after a turn.

With `--pi-camera` and the synthetic camera:

@code
The robot went once around the square in 34.660 s of simulation, 34.660 s of wall time
Distance of the axle centre from the line: mean 0.034 m, largest 0.117 m (the line is 0.085 m wide)
Camera images: 694, without the line in them 0
Stream: 20.0 frames/s, mean age of the picture used 24.3 ms, reconnections 0
@endcode

The robot never lost the line. It cut the corners, as it turns before its axle reaches them.

## Things to try

1. Run the server with `--period 4`, then 3. At what period can the robot no longer keep up with
   the square path, and what does it do then?
2. Move the paper slowly while the robot drives with `--pi-camera`. How fast can you move it before the robot loses
   the line?
3. Change `offset` in the robot file to 0.2 m and to 0.5 m. How do the corners and the largest
   distance from the line change?
4. Run the server with `--fps 5`. The robot sees the floor less often and later. What happens at
   the corners, and at what frame rate does it lose the line?
5. Raise the cruise speed, `CRUISE_SPEED` in `example_31.cpp`. Pure pursuit with a fixed look
   ahead has a speed above which it cannot take a corner; find it.
6. Draw a circle instead of a square. `find_square` will not find it; make the start work from
   any closed line.

## The code

The simulator:

\include example_31.cpp

The camera server:

\include camera_server.py
