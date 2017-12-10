package interfaces;

import autopilot.AutoPilot;

public class AutopilotFactory {
	
	public static Autopilot createAutopilot(){
		return new AutoPilot();
	}
	
}
