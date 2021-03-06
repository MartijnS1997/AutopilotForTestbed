package autopilot;

import interfaces.AutopilotInputs;
import interfaces.AutopilotOutputs;

import static java.lang.Math.*;
import static java.lang.Math.abs;
import static java.lang.Math.pow;

/**
 * Created by Martijn on 7/12/2017.
 * A controller for the 30 degree maxAOA
 */
public class BetaController extends AutoPilotController{

    public BetaController(AutoPilot autoPilot){
        super(autoPilot);
    }

    @Override
    public AutopilotOutputs getControlActions(){
    	
    	if(super.firstRun){
    		//super.firstRun = false;
    		//return new ControlOutputs();
    	}
    	
        ControlOutputs outputs = new ControlOutputs();
        AutoPilotCamera APCamera = this.getAssociatedAutopilot().getAPCamera();
        AutopilotInputs currentInputs = this.getCurrentInputs();
        PIDController xPIDController = this.getxPID();
        PIDController yPIDController = this.getyPID();

        APCamera.loadNewImage(currentInputs.getImage());
        float elapsedTime = this.getCurrentInputs().getElapsedTime();

        Vector center;
        //System.out.println("######################### NEW ITERATION ###########################");
        AutopilotInputs inputs = this.getCurrentInputs();
//        System.out.println("Elapsed Time: " + inputs.getElapsedTime());
//        System.out.println("Position: " + inputs.getX() + "; " + inputs.getY() + "; " + inputs.getZ());
//        System.out.println("Orientation: " + inputs.getHeading() + "; " + inputs.getPitch() + "; " + inputs.getRoll());
        
        AutopilotInputs prevInput = this.getPreviousInputs();
//        System.out.println("Prev Elapsed: " + prevInput.getElapsedTime());
//        System.out.println("Prev Position: " + prevInput.getX() + "; " + prevInput.getY() + "; " + prevInput.getZ());
//        System.out.println("Prev Orientation: " + prevInput.getHeading() + "; " + prevInput.getPitch() + "; " + prevInput.getRoll());
//        
//        System.out.println("Time passed between current and previous: " + (inputs.getElapsedTime()-prevInput.getElapsedTime()));
//        
//        System.out.println("inputs objects current: " + this.getCurrentInputs() + "; previous: " + this.getPreviousInputs());
        
        try{
            center = APCamera.getCenterOfNCubes(1);
        }catch(NoCubeException e){
            center = new Vector(-10, 0, 4);
        }
        float xPosition = xPIDController.getPIDOutput(-center.getxValue(), elapsedTime);
        float yPosition = yPIDController.getPIDOutput(-center.getyValue(), elapsedTime);
        int nbColumns = APCamera.getNbColumns();
        int nbRows = APCamera.getNbRows();
        float cubeCoeff = (float) min(MAX_CUBE_COEFF, sqrt(nbRows*nbColumns)/center.getzValue());
        //System.out.println("PID positions x= " + xPosition + " ; y= " + yPosition);
        //System.out.println("Cube coefficients: " + cubeCoeff);
        xControlActions(outputs, xPosition,cubeCoeff);
        yControlActions(outputs, yPosition, cubeCoeff, currentInputs.getPitch());
        setThrustOut(outputs, cubeCoeff);

        //System.out.println("Outputs Horizontal: " + outputs.getHorStabInclination()*RAD2DEGREE + "; Vertical: " + outputs.getVerStabInclination()*RAD2DEGREE );
        
        //System.out.println("Uncorrected outputs: " + outputs);
        
        rollControl(outputs);
        
        //System.out.println("Roll control outputs: " + outputs);
        
        angleOfAttackControl(outputs);
        
        //System.out.println("Corrected outputs: " + outputs);
        //System.out.println("approx velo: " + getVelocityApprox());
        return outputs;
    }

    private void xControlActions(ControlOutputs outputs, float xPos, float cubeCoeff){
        float verticalStabIncl;
        float rightMainIncl = MAIN_STABLE_INCLINATION;
        float leftMainIncl = MAIN_STABLE_INCLINATION;
        float roll = this.getCurrentInputs().getRoll();
        //System.out.println("Roll: " + this.currentInputs.getRoll());
        if(abs(xPos) > X_THRESHOLD){
            // cube coeff: to increase pitch for faraway objects
            // squared for large corrections if large error
            verticalStabIncl = (float) (signum(xPos) * min(MAX_VER_STAB_ANGLE, STANDARD_VER_STAB_INCL*pow(abs(xPos)/1f,2))); //*cube coeff
            rightMainIncl = (float) (-signum(xPos)*sqrt(abs(roll))*TURNING_INCLINATION +  MAIN_STABLE_INCLINATION);
            leftMainIncl = (float) (signum(xPos)*sqrt(abs(roll))* TURNING_INCLINATION +  MAIN_STABLE_INCLINATION);

        }else{
            verticalStabIncl = STABILIZER_STABLE_INCLINATION;
        }

        outputs.setVerStabInclination(verticalStabIncl);
        outputs.setRightWingInclination(rightMainIncl);
        outputs.setLeftWingInclination(leftMainIncl);
    }

    private void yControlActions(ControlOutputs outputs, float yPos, float cubeCoeff, float pitch){
        float horizontalStabIncl;
        //TODO verify if this works
        yPos = (float) (yPos*(1+pitch*2/PI));
        if(abs(yPos) > Y_THRESHOLD){
            horizontalStabIncl = (float) (-signum(yPos) * min(MAX_HOR_STAB_INCLINATION, STANDARD_HOR_STAB_INCLINATION*cubeCoeff*pow(abs(yPos)/1f,2)));
        }else{
            horizontalStabIncl = STABILIZER_STABLE_INCLINATION;
        }
        outputs.setHorStabInclination(horizontalStabIncl);
    }

    private void setThrustOut(ControlOutputs outputs, float cubeCoeff){
        //Todo implement: write the output to the outputs
        float pitch = this.getCurrentInputs().getPitch();
        float maxThrust =  this.getAssociatedAutopilot().getConfig().getMaxThrust();
        int threshold = Math.round(THRESHOLD_DISTANCE);
        float gravity = this.getAssociatedAutopilot().getConfig().getGravity();

        // Thrust
        
        float thrust = (float) ((maxThrust/4) + THRUST_FACTOR*this.getTotalMass()*gravity*cubeCoeff);
        //System.out.println("thrust: " + thrust);
        outputs.setThrust(Math.max(Math.min(thrust, maxThrust), 0));
        if (getVelocityApprox().getzValue() > 16.5f){
        	outputs.setThrust(0f);
        }
    }

    /*
    Getters and setters
     */

    @Override
    protected float getMainStableInclination() {
        return MAIN_STABLE_INCLINATION;
    }

    @Override
    protected float getStabilizerStableInclination() {
        return STABILIZER_STABLE_INCLINATION;
    }

    @Override
    protected float getRollThreshold() {
        return ROLL_THRESHOLD;
    }

    @Override
    protected float getInclinationAOAMargin() {
        return ERROR_INCLINATION_MARGIN;
    }

    private PIDController getxPID() {
        return xPID;
    }

    private PIDController getyPID() {
        return yPID;
    }

    private PIDController xPID = new PIDController(1.f, 0.0f, 0.0f);
    private PIDController yPID = new PIDController(1.f, 0.0f, 0.0f);
    private PIDController rollPID = new PIDController(1f, 0.0f, 0.0f);


    private static final float STANDARD_INCLINATION = (float) (10*PI/180);
    public  static final float MAIN_STABLE_INCLINATION = (float) (10*PI/180);
    public  static final float MAIN_MAX_INCLINATION = (float) (10*PI/180);
    private static final float MAX_HOR_STAB_INCLINATION = (float) (20*PI/180);
    private static final float STANDARD_HOR_STAB_INCLINATION = (float) (10*PI/180);
    private static final float MAX_VER_STAB_ANGLE = (float) (11*PI/180f);
    private static final float STANDARD_VER_STAB_INCL = (float) (7*PI/180f);
    private static final float TURNING_INCLINATION = (float) (10*PI/180);
    private static final float ERROR_INCLINATION_MARGIN = (float) (2*PI/180);
    private static final int   BIAS = 0;
    private static final float THRESHOLD_DISTANCE = 1f;
    private static final float STANDARD_THRUST = 32.859283f*2;
    private static final float THRUST_FACTOR = 1.75f;
    private static final float THRESHOLD_THRUST_ANGLE = (float)(PI/20);
    private static final float MAX_CUBE_COEFF = 3f;
    public  static final float STABILIZER_STABLE_INCLINATION = 0.0f;
    private static final float GRAVITY = 9.81f;
    private static final float ROLL_THRESHOLD = (float) (PI * 40.0f/180.0f);
    private static final float RAD2DEGREE = (float) (180f/ PI);
    private static final float CHECK_INTERVAL = 1/20.f;
    private static final float X_THRESHOLD = 0f;
    private static final float Y_THRESHOLD = 0f;
    
    
}
