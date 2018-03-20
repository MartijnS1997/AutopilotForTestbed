package internal.Physics;

import internal.Testbed.Drone;
import internal.Testbed.DroneState;
import internal.Helper.Vector;

import static java.lang.Math.abs;

import interfaces.*;

/**
 * Created by Martijn on 14/02/2018.
 * A class used for simulating the behavior of the tyres of the drone
 */
public class ChassisPhysX {

    /**
     * Constructor of the chassis
     * @param config the configuration of the drone
     */
    public ChassisPhysX(AutopilotConfig config){
        //extract the constants for the wheels
        float tyreRadius = config.getTyreRadius();
        float tyreSlope = config.getTyreSlope();
        float dampSlope = config.getDampSlope();
        float maxBrake = config.getRMax();
        float maxFricCoeff = config.getFcMax();

        // extract the positions of the wheels
        Vector frontTyrePos = new Vector(0f, -abs(config.getWheelY()), -abs(config.getFrontWheelZ()));
        Vector rearLeftTyrePos = new Vector(-abs(config.getRearWheelX()), -abs(config.getWheelY()), abs(config.getRearWheelZ()));
        Vector rearRightTyrePos = new Vector(abs(config.getRearWheelX()), -abs(config.getWheelY()), abs(config.getRearWheelZ()));

        //construct the wheels
        this.frontTyre = new TyrePhysX(frontTyrePos, tyreRadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);
        this.rearLeftTyre = new RearTyrePhysX(rearLeftTyrePos, tyreRadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);
        this.rearRightTyre = new RearTyrePhysX(rearRightTyrePos, tyreRadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);


    }

    /**
     * Checks if the chassis touches the ground, used in validation for usage of drone constructor in the air
     * @param orientation the orientation of the drone
     * @param position the position of the drone
     * @return true if and only if one of the tyres touches the ground
     */
    protected boolean touchesGround(Vector orientation, Vector position){

        TyrePhysX frontTyre = this.getFrontTyre();
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        TyrePhysX rearRightTyre = this.getRearRightTyre();
        boolean groundTouched = frontTyre.getTyreDistanceToGround(orientation, position) <= frontTyre.getTyreRadius();
        groundTouched = groundTouched || (rearLeftTyre.getTyreDistanceToGround(orientation, position) <= rearLeftTyre.getTyreRadius());
        return groundTouched || (rearRightTyre.getTyreDistanceToGround(orientation, position) <= rearRightTyre.getTyreRadius());
    }


    //TODO complete this
    public Vector netChassisForces(DroneState state, AutopilotOutputs inputs, float deltaTime){
        //first calculate the known forces exerted by the tires
        TyrePhysX frontTyre = this.getFrontTyre();
        Vector frontTyreForce = frontTyre.getNetForceTyre(state, inputs.getFrontBrakeForce(), deltaTime, state.getPrevFrontTyreDelta() );
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        Vector rearLeftTyreForce = rearLeftTyre.getNetForceTyre(state, inputs.getLeftBrakeForce(), deltaTime, state.getPrevRearLeftTyreDelta());
        TyrePhysX rearRightTyre = this.getRearRightTyre();
        Vector rearRightTyreForce = rearRightTyre.getNetForceTyre(state, inputs.getRightBrakeForce(), deltaTime, state.getPrevRearRightTyreDelta());

        Vector[] forces = {frontTyreForce, rearLeftTyreForce, rearRightTyreForce};

        //System.out.println("Total chassis force: " + Vector.sumVectorArray(forces));

        return Vector.sumVectorArray(forces);
    }

    /**
     * Calculates the net moment exerted by the chassis on the drone (in drone axis system)
     * @param state the state of the drone at moment of invoking the method
     * @param deltaTime the time passed between two simulation steps
     * @return the net chassis moment in the drone axis system
     */
    public Vector netChassisMoment(DroneState state, AutopilotOutputs inputs, float deltaTime){
        //System.out.println("NEW ITERATION, ROLL: " + state.getOrientation().getzValue());
        TyrePhysX frontTyre = this.getFrontTyre();
        //System.out.println("Front-tyre: ");
        Vector frontTyreForce = frontTyre.getNetForceTyre(state, inputs.getFrontBrakeForce(), deltaTime, state.getPrevFrontTyreDelta() );
        //System.out.println("front Force: " + frontTyreForce);
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        //System.out.println("Left-Tyre: ");
        Vector rearLeftTyreForce = rearLeftTyre.getNetForceTyre(state, inputs.getLeftBrakeForce(), deltaTime, state.getPrevRearLeftTyreDelta());
        //System.out.println("rear left Force" + rearLeftTyreForce);
        TyrePhysX rearRightTyre = this.getRearRightTyre();
        //System.out.println("Right-trye: ");
        Vector rearRightTyreForce = rearRightTyre.getNetForceTyre(state, inputs.getRightBrakeForce(), deltaTime, state.getPrevRearRightTyreDelta());
        //System.out.println("rear right Force" + rearRightTyreForce);

        Vector frontTyreMoment = frontTyre.getNetMomentTyre(state, frontTyreForce);
        //System.out.println("Front tyre moment: " + frontTyreMoment);
        Vector rearLeftTyreMoment = rearLeftTyre.getNetMomentTyre(state, rearLeftTyreForce);
        //System.out.println("Rear left tyre moment: " + rearLeftTyreMoment);
        Vector rearRightTyreMoment = rearRightTyre.getNetMomentTyre(state, rearRightTyreForce);
        //System.out.println("Rear right tyre moment: " + rearRightTyreMoment);

        Vector[] moments = {frontTyreMoment, rearLeftTyreMoment, rearRightTyreMoment};
        //System.out.println("Net moment on chassis: " + Vector.sumVectorArray(moments));
        //System.out.println("\n");
        return Vector.sumVectorArray(moments);
    }

    /**
     * Sets the correct tyre delta for all the tyres of the drone
     * @param drone the drone where the tyre delta needs to be initialized
     */
    public void setDroneTyreDelta(Drone drone){
        Vector orientation = drone.getOrientation();
        Vector position = drone.getPosition();
        if(!drone.getPhysXEngine().chassisTouchesGround(orientation, position)){
            throw new IllegalStateException("Chassis is not Touching the ground");
        }
        //acquire the tyres
        TyrePhysX frontTyre = this.getFrontTyre();
        TyrePhysX rearLeftTyre = this.getRearLeftTyre();
        TyrePhysX rearRightTyre = this.getRearRightTyre();

//        System.out.println("prev front delta: " + frontTyre.calcRadiusDelta(orientation, position));
//        System.out.println("prev rear delta: " + rearLeftTyre.calcRadiusDelta(orientation, position));

        //then set the deltas
        drone.setPrevFrontTyreDelta(frontTyre.calcRadiusDelta(orientation, position));
        drone.setPrevRearLeftTyreDelta(rearLeftTyre.calcRadiusDelta(orientation, position));
        drone.setPrevRearRightTyreDelta(rearRightTyre.calcRadiusDelta(orientation, position));
    }

    /**
     * Returns true if one of the tyres has crashed
     * @param orientation the orientation of the drone
     * @param position the position of the drone
     * @return true if the chassis experienced a crash
     */
    public boolean checkCrash(Vector orientation, Vector position){
        if(this.getFrontTyre().checkCrash(orientation, position))
            return true;
        else if(this.getRearLeftTyre().checkCrash(orientation, position))
            return true;
        else if(this.getRearRightTyre().checkCrash(orientation, position))
            return true;
        else {
            return false;
        }
    }


    public TyrePhysX getFrontTyre() {
        return frontTyre;
    }

    public TyrePhysX getRearLeftTyre() {
        return rearLeftTyre;
    }

    public TyrePhysX getRearRightTyre() {
        return rearRightTyre;
    }

    /**
     * Instance variables: the tyres of the chassis
     */
    TyrePhysX frontTyre;
    RearTyrePhysX rearLeftTyre;
    RearTyrePhysX rearRightTyre;
}
