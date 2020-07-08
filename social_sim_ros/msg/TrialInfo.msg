# Message containing the information of running a single A-B navigation trial

Header      header
float64     dist_to_target  # Distance to target at the end of the trial
float64     dist_to_ped     # Minimum distance to any pedestrian throughout the trial
uint32      num_collisions  # Number of times that the robot collided with any object
bool        run_complete    # True if the trial run is complete, false if terminated because of timeout
float64     time_elapsed    # Time elapsed since the start of the trial (in seconds)