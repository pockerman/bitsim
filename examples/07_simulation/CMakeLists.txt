BITSIM_ADD_EXAMPLE(chrono_rigid_body REQUIRES CHRONO SMOKE_TEST)
# These run until their window is closed
BITSIM_ADD_EXAMPLE(chrono_rigid_body_simulation REQUIRES CHRONO)
BITSIM_ADD_EXAMPLE(chrono_diff_drive REQUIRES CHRONO SMOKE_TEST)
BITSIM_ADD_EXAMPLE(line_following_infrared_sonar REQUIRES CHRONO IRRLICHT OPENCV SMOKE_TEST)
