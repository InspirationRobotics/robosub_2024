# TODO: See whether this works when connected with the sub.

from auv.utils import deviceHelper

camera = "forwardOak"
print(deviceHelper.dataFromConfig(camera))