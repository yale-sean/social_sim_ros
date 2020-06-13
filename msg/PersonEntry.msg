# Message defining an entry of a person

time        stamp          # Age of the track
uint64      track_id       # Unique ID for each person

# The following fields are extracted from the Kalman state x and its covariance C

# Pose of the track (z value and orientation might not be set, check if corresponding variance on diagonal is > 99999)
geometry_msgs/PoseWithCovariance    pose

# Velocity of the track (z value and rotational velocities might not be set, check if corresponding variance on diagonal is > 99999)
geometry_msgs/TwistWithCovariance   twist