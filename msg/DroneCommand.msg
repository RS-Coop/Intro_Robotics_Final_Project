#For publishing movement commands to drone from swarm
#Identifies drone
string drone_id

#x, y, z, angular
string[] cmd_type

#Positive(1), or negative(-1) (Note: Follows unit circle)
int8[] direction

#How much movement in [-1,1]
int8[] intensity
