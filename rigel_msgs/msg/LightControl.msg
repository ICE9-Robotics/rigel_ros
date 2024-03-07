# LightControl messages control the lights on the robot.
# The light_id field is the id of the light to control:
#   0: Front light
#   1: Pan tilt light
#   2: Rear light  
# The output field controls the intensity of the light, from 0 to 100, where 0 being off and 100 being full intensity.

uint8 light_id
uint8 output