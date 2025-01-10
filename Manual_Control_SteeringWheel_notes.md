File: Manual_control_steeringwheel.py
1. Class “blendedcontrol”
   *	Changes the wheel inputs
   *	Agent is the car steering; blended control takes steering wheel input and the agent’s and blends them together
   *	To blend control, modify line 481 self._control = self._agent.act(self._control, world) to have the blending – probably make a separate function to modify all the thing separately (brake, steering, etc.)
2.	Class “agent”
    *	To turn on blended control:
        - Set “in_control” to “true”
        - Uncomment line 513 (controls, self.in_control = self.pd_controller(controls,world)
    *	To turn off blended control:
        - Set “in_control” to “false”
        - Comment line 513 (controls, self.in_control = self.pd_controller(controls,world)
    *	To change how the autodrive works, change the agent
