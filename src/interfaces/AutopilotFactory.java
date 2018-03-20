package interfaces;

import autopilot.AutoPilot;
import interfaces.Autopilot;
import internal.Physics.PhysXEngine;

public class AutopilotFactory {
	
	public static Autopilot createAutopilot(){
		return new AutoPilot();
	}
	
}
