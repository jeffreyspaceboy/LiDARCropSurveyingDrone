#Get input GPS, will be updated in ECEN 404 when we have access to Drone GPS module
gps_longitude = input("What are the starting longitude coordinates? (eg. 119.1507209256): ");
# split at decimal per gps output
gps_long_lst = gps_longitude.split(".")
# splitting sat decimal, but keep decimal

# print(gps_long_lst)

# convert from list to int
gps_long_coor = [int(i) for i in gps_long_lst]

# gps location readings
long_degrees = gps_long_coor[0]
long_mins = gps_long_coor[1] * 60
print(long_mins)
long_secs = 1


# Determine if location is north or south
if gps_long_coor[0] < 0:
    print(long_degrees,"degrees")
    print(long_mins,"minutes and")
    print(long_secs,"seconds South")
else:
    print(long_degrees,"degrees")
    print(long_mins,"minutes and")
    print(long_secs,"seconds North")

# # Determine if location is west or east
# if gps_coor[1] < 0:
#     print("East")
# else:
#     print("West")