
# The joint command sent to the actuator

# sequence ID: consecutively increasing ID
uint16 seq
time stamp

# define the interpretation of Command.
int8 commandMode

# Enter the joint into failsafe mode
int8 FAILSAFE = -1
int8 POSITION = 1
int8 VELOCITY = 2
int8 EFFORT = 3
int8 INVALID = 4
uint8 count
float32[] command
