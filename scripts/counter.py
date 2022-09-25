f = open("/media/burhan/D/all_tasks/FAST-LIO-MODIFIED/data/synch_data.txt", "r")
counter = 0
for x in f:
    if x == "imu\n":
        counter += 1
print(counter)