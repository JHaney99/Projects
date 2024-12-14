import Object_Detection as obj
import CCI_Measurement as cci
import time

coordinates = obj.main()
print(coordinates)

min_range = 300 #Distance in mm

if type(coordinates) == tuple and (coordinates[2] < min_range):
    time.sleep(5)
    cci.main()
