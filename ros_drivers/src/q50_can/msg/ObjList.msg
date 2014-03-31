#------------------------------------------
#20 objects information
#
#Within +-20m laterally from UNIT(Radar)
#Within 100m longitudinary from UNIT(Radar
#20 closest objects from UNIT
#Including moving and stationary objects
#Using 7 CAN ID(20ms update) * 3 times, it sends 20 objects info and 1 starter info
#------------------------------------------

Header header

#Object ID, Range:0-63,   Id 0 and 62 are invalid. Id 61 means the starter (no obj info) 
uint8 obj_id 

#Object Measured Flag,  0: Not measured 1: Measured(Valid)
#Note that you always need to check this flag.
#There are the cases that some value is put on the other virible but not measured. 
bool isMeasurd

# Object Distance[m] Bitrate:0.1m, Range 0-100m
float32 dist 

#Lateral Distance[m] Bitrate:0.1m Left:+ Right:-, Rangge:+-20m
float32 lat_dist 

#Relative speed[m/s] Bitrate:0.025m/s Approach:- Depart:+
float32 relative_spd 

#Object Dynamic porperty 
# 0 unclassified
# 1 standing
# 2 stopped
# 3 moving
# 4 oncoming
uint8 dyn_prop  

#Object RCS[m], Bitrate:0.5, Range:-64 ~ 63
float32 rcs

#Object Width[m], Bitrate: 0.1m
float32 width

#Object Length[m], Bitrate: 0.1m
float32 length

#Messsage Counter, Range:0-3
uint8 msg_cnt