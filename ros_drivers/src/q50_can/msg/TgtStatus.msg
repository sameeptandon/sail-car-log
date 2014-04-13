#------------------------------------------
#Target objects info for ACC(Adaptive Cruisee Control)
#
#output 6 objects info
#66ms update
#------------------------------------------

Header header

#Object ID, Range:0-63
#ID 0 is invalid
uint8 obj_id 

#Object Status 
# 0: No Object
# 1: Valid New(only first cycle of a new object)
# 2: Valid
uint8  status


# Object Distance[m], Bitrate:0.1m
float32 dist 

#Lateral Distance[m], Bitrate:0.1m, Left:+ Right:-, 
float32 lat_dist 

#Relative speed[m/s], Bitrate:0.025m/s, Approach:- Depart:+
float32 relative_spd 

#Object Trajectory[m], Bitrate:0.1m 
float32 traj 

#Obstacle Probability[%], Bitrate:1%, Range:0-100% 
uint8 obst_probab 

#Obstacle Exist Probability[-]
# 0 invalid
# 1 <25%
# 2 <50%
# 3 <75%
# 4 <90%
# 5 <99%
# 5 <99%
# 6 <99.9%
# 7 <99.99%
uint8 exist_probab

#Messsage Counter, Range:0-7
uint8 msg_cnt_a

#-----------------

#Object Relative Acceleration[m/s2], Bitrate: 0.0625m/s2, +:Acceleration -:Decceleration
float32 relative_acc

#Object Width[m], Bitrate: 0.01m
float32 width

#Object Width[m], Bitrate: 0.01m
# 0: Point
# 1: Car
# 2: Truck
# 3: Pedestrian
# 4: Motorcycle
# 5: Bycycle
# 6: Wide
# 7: Unclassified
uint8 type

#Dynamic Property
# 0: Undefined
# 1: Standing
# 2: Stopped
# 3: Moving
# 4: Oncome
# 5: Cross
uint8  dyn_prop

#Object Length[m], Bitrate: 0.1m
float32 length

#Lost reason
uint8 lost_reason

#Messsage Counter, Range:0-7
uint8 msg_cnt_b