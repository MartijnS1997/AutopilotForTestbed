package interfaces;

import autopilot.Path;

public interface Autopilot_v2 {
    AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs_v2 inputs);
    AutopilotOutputs timePassed(AutopilotInputs_v2 inputs);
    void simulationEnded();
	void setPath(Path path);
}