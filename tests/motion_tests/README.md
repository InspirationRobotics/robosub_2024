# Explanation of the Tests

## depth_test.py

It tests a relative depth change of -0.2 meters (go up by 0.2 meters) after having been at a depth of 0.8 meters for 20 seconds. Relative depth change was a new functionality during the 2024 competition season and was utilized in the buoy mission to ensure the AUV was at the same depth as the buoy.

## dvl_test.py

It tests utilizing a Doppler Velocity Log (DVL) to move the AUV backward a distance of 1 meter. DVLs were successfully integrated into the software in the 2024 season, but Onyx's DVL did not always send data to the Jetson. Graey's DVL is more reliable. Because of this, the DVL was not implemented in competition.

## motion_test.py

This test file was drafted during competition week in 2024 and was an attempt to implement roll and pitch functionality into the AUV. In QGroundControl (which manually controls the AUV), there was a setting called roll_pitch_toggle that switched forward/lateral to pitch/roll, respectively. The commented `rc.button_press(256)` was supposed to virtually push a QGroundControl button to perform this switch. However, it was found that execution of this method did not perform that switch, and the AUV was disarmed 3 seconds after execution. It was a failed test.

## roll_test.py

This test file was drafted during competition week in 2024 and attempted to implement roll and pitch functionality into the AUV. This file attempted to utilize available data channels to the flight controller to start a pitch, but this did nothing. 

## servo_test.py

This test file was an attempt to test movement of the servo connected with the marker dropper. When we ran this file, nothing happened. After utilizing a servo tester (no software), we found that the servo was not receiving any power. We did not provide power supply via the servo controller before the competition, so the marker dropper was not implemented. 

## set_heading_test.py

This test file tested the ability of the AUV to orient itself to a particular azimuth heading utilizing the IMU onboard the PixHawk - though the functionality existed in previous years, the roster in 2024 was new and was unfamiliar with how it worked. It worked most of the time, but in some tests the AUV would get stuck in a position with an error just above 5 degrees. We forgot about this, and it came back to bite us during the finals run. In the set_heading method, there is now code to break out of the loop for a change of less than 3 degrees in 3 seconds.

## thruster_test.py

This test file was made to troubleshoot actuation inconsistencies of hardcoded movement instructions observed during prequalification runs. The presence of sleep commands was likely the source of the actuation inconsistencies.