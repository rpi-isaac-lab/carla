# Given by Neel Bhatt
import evdev
from evdev import ecodes,InputDevice,ff,list_devices
device = list_devices()[0]
evtdev = InputDevice(device)
FULL_FF = 65535
force = 0.5
val = int(FULL_FF*force) # Max of FULL_FF
evtdev.write(ecodes.EV_FF,ecodes.FF_AUTOCENTER,val)
#evtdev.write(ecodes.EV_FF,ecodes.FF_CONSTANT,0)
#effect = ff.Effect(ecodes.FF_RAMP,-1,0)
#evdev.events.InputEvent()
#evtdev.upload_effect(effect)
#evtdev.write(ecodes.EV_ABS,ecodes.ABS_X,255)
#print(device)
#print(evtdev.capabilities(verbose=True))
