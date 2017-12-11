package autopilot;

import interfaces.AutopilotInputs;
import interfaces.AutopilotOutputs;

import static java.lang.Math.*;

/**
 * Created by Martijn on 30/10/2017.
 * Appended and edited by Anthony Rathï¿½ on 6/11/2017
 * A class of Autopilot Controllers
 */
public abstract class AutoPilotController {

    /**
     * Constructor for the autopilotController
     * @param autoPilot
     */
    public AutoPilotController(AutoPilot autoPilot){
        this.associatedAutopilot = autoPilot;
        this.setPreviousInputs(dummyData);
        this.currentInputs = dummyData;
    }


    public abstract AutopilotOutputs getControlActions();


    /*
     * Supplementary control methods
     */
    protected void rollControl(ControlOutputs outputs){
        AutopilotInputs input = this.getCurrentInputs();
        float roll = input.getRoll();

        if(roll >= this.getRollThreshold()){
            outputs.setRightWingInclination(-this.getMainStableInclination()); //MAIN_STABLE_INCLINATION*(abs(roll)/ROLL_THRESHOLD)));
        }
        else if(roll <= -this.getRollThreshold()){
            outputs.setLeftWingInclination(-this.getMainStableInclination()); //, MAIN_STABLE_INCLINATION*(abs(roll)/ROLL_THRESHOLD)));
        }else{
            // change nothing
        }
    }

    /**
     * Checks if the current control outputs are realisable under the angle of attack constraint provided
     * by the autopilot configuration. If not the controls are adjusted to fit the constraints
     * @param controlOutputs the control outputs to be checked
     */
    protected void angleOfAttackControl(ControlOutputs controlOutputs){

        //first check if the current and the previous steps are initialized, if not so delete all control actions
        //and set to standard value
        if(this.getCurrentInputs() == null || this.getPreviousInputs() == null){
            controlOutputs.reset();
            return;
        }
        //first prepare all the variables
        PhysXEngine.PhysXOptimisations optimisations = this.getAssociatedAutopilot().getPhysXOptimisations();
        AutopilotInputs inputs = this.getCurrentInputs();
        Vector orientation = extractOrientation(inputs);
        Vector velocity = this.getVelocityApprox();
        Vector rotation = this.getRotationApprox();
        float angleOfAttack = this.getAssociatedAutopilot().getConfig().getMaxAOA();
        
        if(Float.isNaN(orientation.getxValue() + velocity.getxValue() + rotation.getxValue())||
        		Float.isNaN(orientation.getyValue() + velocity.getyValue() + rotation.getyValue())||
        		Float.isNaN(orientation.getzValue() + velocity.getzValue() + rotation.getzValue())){
        	//System.out.println("Nan Value in Parameters \n-> Orientation: " + orientation + "\n-> Rotation: " + rotation + "\n-> Velocity: " + velocity);
        }
        //change until the controls fit
        AOAControlMainLeft(controlOutputs, optimisations,angleOfAttack, orientation, rotation, velocity);
        AOAControlMainRight(controlOutputs, optimisations, angleOfAttack, orientation, rotation, velocity);
        AOAControlHorStabilizer(controlOutputs, optimisations, angleOfAttack, orientation, rotation, velocity);
        AOAControlVerStabilizer(controlOutputs, optimisations, angleOfAttack, orientation, rotation, velocity);
    }




    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlMainLeft(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack,  Vector orientation, Vector rotation, Vector velocity){
        float inclinationBorder1 = optimisations.getMaxLeftMainWingInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxLeftMainWingInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getLeftWingInclination();

        float realisableInclination = setBetween(desiredInclination, inclinationBorder1, inclinationBorder2, this.getInclinationAOAMargin());

        controlOutputs.setLeftWingInclination(realisableInclination);

        return desiredInclination == realisableInclination;
    }

    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlMainRight(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack, Vector orientation, Vector rotation, Vector velocity){
        float inclinationBorder1 = optimisations.getMaxRightMainWingInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxRightMainWingInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getRightWingInclination();

        float realisableInclination = setBetween(desiredInclination, inclinationBorder1, inclinationBorder2, getInclinationAOAMargin());

        controlOutputs.setRightWingInclination(realisableInclination);

        return desiredInclination == realisableInclination;
    }

    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlHorStabilizer(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack, Vector orientation, Vector rotation, Vector velocity){

        float inclinationBorder1 = optimisations.getMaxHorStabInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxHorStabInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getHorStabInclination();

        float realisableInclination = setBetween(desiredInclination, inclinationBorder1, inclinationBorder2, this.getInclinationAOAMargin());

        controlOutputs.setHorStabInclination(realisableInclination);

        return desiredInclination == realisableInclination;
    }

    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlVerStabilizer(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack, Vector orientation, Vector rotation, Vector velocity){

        float inclinationBorder1 = optimisations.getMaxVerStabInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxVerStabInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getVerStabInclination();

        float realisableInclination = setBetween(desiredInclination, inclinationBorder1, inclinationBorder2, this.getInclinationAOAMargin());

        controlOutputs.setVerStabInclination(realisableInclination);

        return desiredInclination == realisableInclination;
    }


    /*
    Helper methods
     */
    //TODO account for the fact that the distance between the borders could be smaller than the error margin
    private float setBetween(float value, float border1, float border2, float errorMargin){
        //first check if the value isn't already between the borders:
        float[] borders = sortValue(border1, border2);
        float lowerBorder = borders[0];
        float upperBorder = borders[1];
        //check if it is already between the borders
        if(value >= lowerBorder && value <= upperBorder)
            return value;

        //if not so, set it between with a given error margin
        //check if the value is closest to the lower border
        if(abs(lowerBorder - value) <= abs(upperBorder - value)){
            return lowerBorder - signum(lowerBorder)*errorMargin;
        }else{
            return upperBorder - signum(upperBorder)*errorMargin;
        }

    }

    /**
     * Sorts the two values
     * @param value1 the first value to be sorted
     * @param value2 the second value to be sorted
     * @return an array of size 2 with the smallest value first and the largest second.
     */
    private float[] sortValue(float value1, float value2){

        float[] sortedArray = new float[2];
        if(value1 <= value2) {
            sortedArray[0] = value1;
            sortedArray[1] = value2;
        }else{
            sortedArray[0] = value2;
            sortedArray[1] = value1;
        }

        return sortedArray;
    }

    protected float getTotalMass(){
        AutoPilot autopilot = this.getAssociatedAutopilot();
        float mainWings = autopilot.getMainWingMass()*2;
        float stabilizers = autopilot.getStabilizerMass()*2;
        float engine = autopilot.getEngineMass();

        return mainWings + stabilizers + engine;
    }

    /**
     * Calculate an approximation of the velocity
     * @return the approximation of the velocity
     * elaboration: see textbook numerical math for derivative methods, the
     * derivative of f(k+1) - f(k-1) / (2*timeStep) has O(hÂ²) correctness
     */
    public Vector getVelocityApprox(){
        //get the inputs at moment k - 1 for the derivative
        AutopilotInputs prevInputs = this.getPreviousInputs();
        //get the inputs at moment k
        AutopilotInputs currentInputs = this.getCurrentInputs();
        float prevTime = prevInputs.getElapsedTime();
        float currentTime = currentInputs.getElapsedTime();

        Vector prevPos = extractPosition(prevInputs);
        Vector currentPos = extractPosition(currentInputs);

        Vector posDiff = currentPos.vectorDifference(prevPos);
        float timeDiff = currentTime - prevTime;

        return posDiff.scalarMult(1/timeDiff);
    }

    private Vector getRotationApprox(){
        AutopilotInputs prevInputs = this.getPreviousInputs();
        //get the inputs at moment k
        AutopilotInputs currentInputs = this.getCurrentInputs();

        float prevTime = prevInputs.getElapsedTime();
        float currentTime = currentInputs.getElapsedTime();

        Vector prevOrient = extractOrientation(prevInputs);
        Vector currentOrient = extractOrientation(currentInputs);

        Vector orientDiff = currentOrient.vectorDifference(prevOrient);
        float timeDiff = currentTime - prevTime;

        // the given rotation vector is given in heading pitch and roll components
        Vector rotationHPR = orientDiff.scalarMult(1/timeDiff);
        // convert back to the world axis rotation vector
        return PhysXEngine.HPRtoRotation(rotationHPR, currentOrient);
    }

    private void logControlActions(ControlOutputs outputs, String controlString){

        controlString += "Left wing inclination: "  + outputs.getLeftWingInclination()*RAD2DEGREE + "\n";
        controlString += "Right wing inclination: " + outputs.getRightWingInclination()*RAD2DEGREE + "\n";
        controlString += "Horizontal stabilizer inclination: " + outputs.getHorStabInclination()*RAD2DEGREE + "\n";
        controlString += "Vertical wing inclination" + outputs.getVerStabInclination()*RAD2DEGREE + "\n";

        // write the controls to the recorder
        FlightRecorder recorder = this.getFlightRecorder();
        recorder.appendControlLog(controlString);
    }

    /**
     * determines the largest value of both entries, if both are negative an exception is thrown
     * @param entry1 the first entry to check
     * @param entry2 the second entry to check
     * @return the largest entry of both
     * @throws IllegalArgumentException thrown if both entries are negative
     */
    private float getMaxPos(float entry1, float entry2) throws IllegalArgumentException{
        if(entry1 >= entry2 && entry1 >= 0){
            return entry1;
        }else if(entry2 >= 0){
            return entry2;
        }else{
            return 0.0f;
        }
    }

    /**
     * determines the smallest value of both entries, if both are positive an exception is thrown
     * @param entry1 the first entry to check
     * @param entry2 the second entry to check
     * @return the smallest entry of both
     * @throws IllegalArgumentException thrown if both entries are positive
     */
    private float getMinNeg(float entry1, float entry2) throws IllegalArgumentException{
        if(entry1 <= entry2 && entry1 <= 0){
            return entry1;
        }else if(entry2 <= 0){
            return entry2;
        }else{
            return 0.0f;
        }
    }

    /*
    Getters and setters
     */

    /**
     * Getter for the main stable inclination constant
     * @return floating point nb containing the stable inclination
     */
    protected abstract float getMainStableInclination();

    /**
     * Getter for the stabilizer stable inclination constant
     * @return floating point nb containing the stabilizer stable inclination
     */
    protected abstract float getStabilizerStableInclination();

    /**
     * Getter for the roll threshold constant
     * @return floating point nb containing the roll threshold
     */
    protected abstract float getRollThreshold();

    /**
     * Getter for the inclination margin for the AOA
     * @return floating point nb containing the AOA Margin
     */
    protected abstract float getInclinationAOAMargin();

    /**
     * Extractor of the orientation in vector format
     * @param inputs the autopilotInput object containing the current inputs
     * @return a vector containing the orientation of the drone in vector format
     */
    private static Vector extractOrientation(AutopilotInputs inputs){
        return new Vector(inputs.getHeading(), inputs.getPitch(), inputs.getRoll());
    }

    /**
     * Extractor of the orientation in vector format
     * @param inputs the autopilotInput object containing the current inputs
     * @return a vector containing the position of the drone in vector format
     */
    private static Vector extractPosition(AutopilotInputs inputs){
        return new Vector(inputs.getX(), inputs.getY(), inputs.getZ());
    }


    /**
     * Setter for the current inputs of the drone, the old currentInputs are automatically
     * set previous inputs
     * @param currentInputs the current input for the autopilot
     */
    public void setCurrentInputs(AutopilotInputs currentInputs) {
        //first write to the previous outputs
        this.setPreviousInputs(this.getCurrentInputs());

        //then write to the new ones.
        this.currentInputs = new DeepCopyInputs(currentInputs);
    }

    /**
     * Returns the previous Autopilot Inputs
     */
    protected AutopilotInputs getPreviousInputs() {
        return previousInputs;
    }

    /**
     * Setter for the autopilot inputs
     * @param previousInputs the previous inputs
     */
    private void setPreviousInputs(AutopilotInputs previousInputs){
        this.previousInputs = new DeepCopyInputs(previousInputs);
    }

    /**
     * getter for the associated Autopilot
     * @return the associated autopilot
     */
    public AutoPilot getAssociatedAutopilot() {
        return associatedAutopilot;
    }

    /**
     * Getter for the current inputs
     * @return the current inputs of the drone
     */
    public AutopilotInputs getCurrentInputs() {
        return currentInputs;
    }

    /**
     * Getter for the flight recorder of the drone
     * @return the flight recorder of the drone
     */
    public FlightRecorder getFlightRecorder() {
        return flightRecorder;
    }

    /**
     * Setter for the flight recorder of the drone
     * @param flightRecorder the desired flight recorder
     */
    public void setFlightRecorder(FlightRecorder flightRecorder) {
        this.flightRecorder = flightRecorder;
    }



    /**
     * Variable that stores the associated autopilot
     */
    private AutoPilot associatedAutopilot;
    /**
     * Variable that stores the current inputs of the controller
     */
    private AutopilotInputs currentInputs;
    /**
     * Variable that stores the previous inputs of the controller
     */
    private AutopilotInputs previousInputs;
    /**
     * Variable that stores the flight recorder of the controller
     */
    private FlightRecorder flightRecorder;
    
    protected boolean firstRun = true;

    private final static float RAD2DEGREE = (float) (180f/PI);


    /*
    Error messages
     */
    private final static String NO_POS_MAX = "No positive maximum found";
    private final static String NO_NEG_MIN = "No negative minimum found";

    private  static AutopilotInputs dummyData = new AutopilotInputs() {
        @Override
        public byte[] getImage() {
            return new byte[0];
        }

        @Override
        public float getX() {
            return 0;
        }

        @Override
        public float getY() {
            return 0;
        }

        @Override
        public float getZ() {
            return 0;
        }

        @Override
        public float getHeading() {
            return 0;
        }

        @Override
        public float getPitch() {
            return 0;
        }

        @Override
        public float getRoll() {
            return 0;
        }

        @Override
        public float getElapsedTime() {
            return 0;
        }
    };


    protected class ControlOutputs implements AutopilotOutputs{

        protected ControlOutputs(){
            // empty
        }

        private void reset(){
            this.setRightWingInclination(AutoPilotController.this.getMainStableInclination());
            this.setLeftWingInclination(AutoPilotController.this.getMainStableInclination());
            this.setHorStabInclination(AutoPilotController.this.getStabilizerStableInclination());
            this.setVerStabInclination(AutoPilotController.this.getStabilizerStableInclination());

        }

        @Override
        public float getThrust() {
            return this.thrust;
        }

        @Override
        public float getLeftWingInclination() {
            return this.leftWingInclination;
        }

        @Override
        public float getRightWingInclination() {
            return this.rightWingInclination;
        }

        @Override
        public float getHorStabInclination() {
            return this.horStabInclination;
        }

        @Override
        public float getVerStabInclination() {
            return this.verStabInclination;
        }

        /**
         * Setter for the Thrust
         * @param thrust the desired thrust
         */
        public void setThrust(float thrust) {
            this.thrust = thrust;
        }

        /**
         * Setter for the left wing inclination
         * @param leftWingInclination
         */
        public void setLeftWingInclination(float leftWingInclination) {
            this.leftWingInclination = leftWingInclination;
        }

        /**
         * Setter for the right wing inclination
         * @param rightWingInclination the desired right wing inclination
         */
        public void setRightWingInclination(float rightWingInclination) {
            this.rightWingInclination = rightWingInclination;
        }

        /**
         * Setter for the horizontal stabilizer inclination
         * @param horStabInclination the desired horizontal stabilizer inclination
         */
        public void setHorStabInclination(float horStabInclination) {
            this.horStabInclination = horStabInclination;
        }

        /**
         * Setter for the vertical stabilizer inclination
         * @param verStabInclination the desired vertical stabilizer inclination
         */
        public void setVerStabInclination(float verStabInclination) {
            this.verStabInclination = verStabInclination;
        }

        public String getConfigMode() {
            return configMode;
        }

        public void setConfigMode(String configMode) {
            this.configMode = configMode;
        }

        //initialize the writes to the stable state of the drone
        private float thrust = 0.f;
        private float leftWingInclination = AutoPilotController.this.getMainStableInclination();
        private float rightWingInclination = AutoPilotController.this.getMainStableInclination();
        private float horStabInclination = AutoPilotController.this.getStabilizerStableInclination();
        private float verStabInclination = AutoPilotController.this.getStabilizerStableInclination();
        private String configMode;
        @Override
        public String toString() {
            return "ControlOutputs{" +
                    "thrust=" + thrust +
                    ", leftWingInclination=" + leftWingInclination +
                    ", rightWingInclination=" + rightWingInclination +
                    ", horStabInclination=" + horStabInclination +
                    ", verStabInclination=" + verStabInclination +
                    '}';
        }
    }

    protected class PIDController {
        /**
         * Constructor for a PID controller object
         * @param gainConstant the constant for the gain of de PID controller (also denoted as Kp)
         * @param integralConstant the constant for the integral of the PID controller (also denoted as Ki)
         * @param derivativeConstant the constant for the derivative of the PID controller (also denoted as Kd)
         */
        protected PIDController(float gainConstant, float integralConstant, float derivativeConstant){
            // set the constants
            this.gainConstant = gainConstant;
            this.integralConstant = integralConstant;
            this.derivativeConstant = derivativeConstant;
        }

        /**
         * Constructs a PID controller with the gain, integral and derivative parameters set to 1.0
         */
        protected PIDController(){
            this(1.0f, 1.0f, 1.0f);
        }

        /**
         * Calculates the output for the current inputs of the PID controller
         * @param input the input signal of the controller (from the feedback loop)
         * @param elapsedTime the elapsed time during the simulation
         * @return the output of the PID controller for the given inputs
         */
        protected float getPIDOutput(float input, float elapsedTime){

            // variables needed for calculation
            float setPoint = this.getSetPoint();
            float prevError = this.getPreviousError();
            float integral = this.getIntegral();
            float Kp = this.getGainConstant();
            float Ki = this.getIntegralConstant();
            float Kd = this.getDerivativeConstant();
            float deltaTime = elapsedTime - this.getPreviousTime();

            //determine the PID control factors
            float error = setPoint - input;
            float derivative = (error - prevError)/deltaTime;
            integral = integral + error*deltaTime;

            // calculate the output
            float output = Kp * error + Ki*integral + Kd*derivative;

            // save the state
            this.setIntegral(integral);
            this.setPreviousError(error);
            this.setPreviousTime(elapsedTime);

            return output;
        }

        private float getIntegral() {
            return integral;
        }

        private void setIntegral(float integral) {
            this.integral = integral;
        }

        private float getPreviousError() {
            return previousError;
        }

        private void setPreviousError(float previousError) {
            this.previousError = previousError;
        }

        private float getSetPoint() {
            return setPoint;
        }

        protected void setSetPoint(float setPoint) {
            this.setPoint = setPoint;
        }

        public float getPreviousTime() {
            return previousTime;
        }

        public void setPreviousTime(float previousTime) {
            this.previousTime = previousTime;
        }

        private float getGainConstant() {
            return gainConstant;
        }

        private float getIntegralConstant() {
            return integralConstant;
        }

        public float getDerivativeConstant() {
            return derivativeConstant;
        }

        private float integral = 0.0f;
        private float previousError = 0.0f;
        private float setPoint = 0.0f;
        private float previousTime = 0.0f;
        private float gainConstant;
        private float integralConstant;
        private float derivativeConstant;
      
    }
    
    /**
     * A class that takes a deep copy of the given input
     * @author Martijn
     */
    private class DeepCopyInputs implements AutopilotInputs{
    	
    	public DeepCopyInputs(AutopilotInputs input){
    		this.orientation = new Vector(input.getHeading() , input.getPitch(), input.getRoll());
    		this.position = new Vector(input.getX(), input.getY(), input.getZ());
    		this.elapsedTime = input.getElapsedTime();
    		this.image = input.getImage();
    	}

		@Override
		public byte[] getImage() {
			// TODO Auto-generated method stub
			return this.image;
		}

		@Override
		public float getX() {
			// TODO Auto-generated method stub
			return this.position.getxValue();
		}

		@Override
		public float getY() {
			// TODO Auto-generated method stub
			return this.position.getyValue();
		}

		@Override
		public float getZ() {
			// TODO Auto-generated method stub
			return this.position.getzValue();
		}

		@Override
		public float getHeading() {
			// TODO Auto-generated method stub
			return this.orientation.getxValue();
		}

		@Override
		public float getPitch() {
			// TODO Auto-generated method stub
			return this.orientation.getyValue();
		}

		@Override
		public float getRoll() {
			// TODO Auto-generated method stub
			return this.orientation.getzValue();
		}

		@Override
		public float getElapsedTime() {
			// TODO Auto-generated method stub
			return this.elapsedTime;
		}
		
		private Vector orientation = new Vector();
		private Vector position = new Vector();
		private float elapsedTime = 0.0f;
		private byte[] image;
    	
    }

}
