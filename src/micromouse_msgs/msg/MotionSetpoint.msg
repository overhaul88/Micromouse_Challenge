# Tier 2 (floodfill solver) -> Tier 1 (PID controller) motion command.
# Hold target_heading while moving forward at target_speed; target_speed
# of 0 means rotate in place toward target_heading (an in-place turn).

float64 target_heading   # absolute heading in the odom frame, radians, wrapped to [-pi, pi]
float64 target_speed     # forward speed along target_heading, m/s

