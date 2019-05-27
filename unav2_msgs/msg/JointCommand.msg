
# The joint command sent to the actuator

# sequence ID: consecutively increasing ID
uint16 seq

# define the interpretation of Command.
int8 mode
# Enter the joint into failsafe mode
int8 FAILSAFE = -1
int8 DISABLED = 0
int8 POSITION = 1
int8 VELOCITY = 2
int8 EFFORT = 3
uint8 count
float32[] command
