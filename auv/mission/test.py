import time

for i in range(2):
    print(i)
    if i == False:
        print("[INFO] Setting depth to 0.0")
    elif i == True:
        print("[INFO] Setting depth to 0.5")
    start_time = time.time()
    while time.time() - start_time < 2:
        pass