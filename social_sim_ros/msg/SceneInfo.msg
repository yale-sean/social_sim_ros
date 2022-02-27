# Message containing the parameters for Unity Scene trials

Header                  header
string                  scenario_name        # Which scene we are running
geometry_msgs/Pose      robot_start_pose     # Pose of robot start location
geometry_msgs/Pose      robot_target_pose    # Pose of robot target location
uint16                  num_people           # Number of people in scene
uint16                  num_groups           # Number of groups in scene
string                  environment          # Which environment
