# Message defining an entry of a person

Header      header         # Age of the track
uint64      track_id       # Unique ID for each person

# The following fields are extracted from the Kalman state x and its covariance C

# Pose of the track
geometry_msgs/Pose      pose

# Velocity of the track
geometry_msgs/Twist     twist
