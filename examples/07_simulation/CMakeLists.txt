BITRL_ADD_EXAMPLE(chrono_rigid_body REQUIRES CHRONO SMOKE_TEST)
# These run until their window is closed
BITRL_ADD_EXAMPLE(chrono_rigid_body_simulation REQUIRES CHRONO)
BITRL_ADD_EXAMPLE(chrono_diff_drive REQUIRES CHRONO SMOKE_TEST)
BITRL_ADD_EXAMPLE(line_following_infrared_sonar REQUIRES CHRONO IRRLICHT OPENCV SMOKE_TEST)
