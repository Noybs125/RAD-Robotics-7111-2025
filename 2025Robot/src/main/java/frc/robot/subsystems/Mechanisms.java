package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kMechanisms;
import frc.robot.utils.encoder.WpiEncoder;
import frc.robot.utils.motor.ArmSimMotor;
import frc.robot.utils.motor.CTREMotor;
import frc.robot.utils.motor.ElevatorSimMotor;
import frc.robot.utils.motor.Motor;
import frc.robot.utils.motor.TwoMotors;

public class Mechanisms extends SubsystemBase {


    // Elevator motor id's are 8 and 2. one must be inverted
    private Motor elevator;
    private Motor wrist;
    private double elevatorSetpoint = 0;
    private double wristSetpoint = 0;
    private boolean isManual = false;
    private double lowerLimit;
    private double upperLimit;
    private MechanismsState state = MechanismsState.Default;
    private double elevatorMaxSpeed = Constants.kMechanisms.elevatorMaxSpeed;
    private double maxWristSpeed = kMechanisms.maxWristSpeed;
    private double maxElevatorSpeed = kMechanisms.elevatorMaxSpeed;

    /**
     * States for choosing what position to set mechanisms to.
     * States include: "ReefL1" through "ReefL4", "AlgaeL2" and "AlgaeL3", "AlgaeProcessor", "AlgaeNet", "Store", "CoralFeeder", and "Climb".
     */
    public enum MechanismsState{
        ReefL1,
        ReefL2,
        ReefL3,
        ReefL4,
        AlgaeL2,
        AlgaeL3,
        AlgaeProcessor,
        AlgaeNet,
        Store,
        StoreCoral,
        CoralFeeder,
        Climb,
        Manual,
        Default,
    };

    private double elevatorMinimumLength = 0.14925;
    private double wristLength = 1;
    private Mechanism2d mech2d = new Mechanism2d(5, 5);
    private MechanismRoot2d elevatorMech2dRoot = mech2d.getRoot("Elevator Root", 1, 1);
    private MechanismRoot2d wristMech2dRoot = mech2d.getRoot("Wrist Root", 2, 1);
    private MechanismLigament2d elevatorMech2d = elevatorMech2dRoot.append(new MechanismLigament2d("elevator", elevatorMinimumLength, 90));
    private MechanismLigament2d wristMech2d = wristMech2dRoot.append(new MechanismLigament2d("wrist", wristLength, 0));

    /**
     * Constructor for Mechanisms Class.
     * Initilizes testEncoder, elevator, and wrist objects using WpiEncoder, ElevatorSimMotor, and ArmSimMotor respectively.
     * @see -WpiEncoder class is located under frc.robot.utils.encoder.WpiEncoder.
     * @see -ElevatorSimMotor class is located under frc.robot.utils.motor.ElevatorSimMotor.
     * @see -ArmSimMotor class is located under frc.robot.utils.motor.ArmSimMotor.
     */
    public Mechanisms(){
        PIDController twoMotorsPID = Constants.kMechanisms.elevatorPID;
        SimpleMotorFeedforward twoMotorsSMFF = null /*new SimpleMotorFeedforward(0, 0)*/;
        WpiEncoder twoMotorsEncoder = new WpiEncoder(0, 1);

        elevator = new TwoMotors(
            new CTREMotor(8, twoMotorsEncoder, 1, twoMotorsPID, twoMotorsSMFF, 
                new ElevatorSimMotor(null, Constants.kSimulation.elevatorSimGearRatio, Constants.kSimulation.elevatorPid, Constants.kSimulation.elevatorFF, Constants.kSimulation.elevatorSimConstants), Constants.kMechanisms.elevator1Config()),
            new CTREMotor(2, twoMotorsEncoder, 1, twoMotorsPID, twoMotorsSMFF, 
                new ElevatorSimMotor(null, Constants.kSimulation.elevatorSimGearRatio, Constants.kSimulation.elevatorPid, Constants.kSimulation.elevatorFF, Constants.kSimulation.elevatorSimConstants), Constants.kMechanisms.elevator2Config()));

        wrist = new CTREMotor(14, null, kMechanisms.wristGearRatio, Constants.kMechanisms.armPID, Constants.kMechanisms.wristFF, new ArmSimMotor(null, null, null, null), Constants.kMechanisms.wristConfig());

        wrist.setSpeedLimits(Constants.kMechanisms.maxWristSpeed, -Constants.kMechanisms.maxWristSpeed);
    }

    /**
     * Sets wristSetpoint equal to setPoint input and sets the isManual boolean to false.
     * WristSetpoint is the variable that controls where the wrist to the end effector is going to try to go.
     * @see -Affects elevator setpoint
     * @see -Inputs for "moveElevThenArm" and "moveArmThenElev"
     * @param setPoint -Type "double", used to set the setpoint for the wrist, and eventually move the wrist to this variable. UNIT UNKNOWN.
     */
    public void setWristSetpoint(double setPoint) {
        wristSetpoint = setPoint;
        isManual = false;
    }

    /**
     * Sets elevatorSetPoint equal to setPoint input and sets the isManual boolean to false.
     * ElevatorSetPoint is the variable that controls where the elevator is trying to go to
     * @see -Affects elevator setpoint
     * @see -Inputs for "moveElevThenArm" and "moveArmThenElev"
     * @param setPoint -Type "double", used to set the setpoint for the wrist, and eventually move the wrist to this variable.TODO: SetPoint UNIT UNKNOWN.
     */
    public void setElevatorSetpoint(double setPoint) {
        elevatorSetpoint = setPoint;
        isManual = false;
    }
    /**
     * Uses circumference of elevator winch and relative position of encoder to calculate elevator height, 
     * then uses that value devided by the max height to calculate percentage of max height. 
     * Uses getPosition method.
     * @return Calculated elevator height in percent of max height.
     * @see -Link to getPosition: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#get().
     */
    public double getElevatorHeight(){
        double elevatorHeight;

        if ( elevator != null) {
            elevatorHeight = elevator.getPosition();
        }
        else {
            elevatorHeight = 0;
        }
        
        return elevatorHeight;
    }

    /**
     * Sets the elevator motor speed, raising or lowering the elevator.
     * (If using CTRE) Uses set method.
     * @param speed -Type "double", variable limits between -1.0 and 1.0. -1.0 is full speed down, and 1.0 is full speed up. TODO: make sure -1 is down and 1 is up.
     * @see -Link to set(double) method: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#set(double).
     */
    public void setElevatorSpeed(double speed) {
        isManual = true;
        if(elevator.getPosition() >= Constants.kMechanisms.elevatorMaxPosition){
            if(speed > 0){
                speed = 0;
            }else if(speed < -Constants.kMechanisms.elevatorMaxSpeed){
                speed = -Constants.kMechanisms.elevatorMaxSpeed;
            }
        }else if(elevator.getPosition() <= Constants.kMechanisms.elevatorMinPosition){
                if(speed > Constants.kMechanisms.elevatorMaxSpeed){
                    speed = Constants.kMechanisms.elevatorMaxSpeed;
                }else if(speed < 0){
                    speed = 0;
                }
        }
        if (speed == 0){
            elevatorSetpoint = elevator.getPosition();
            isManual = false;
        }
        elevator.setSpeed(speed);
    }

    /**
     * Sets the end effector's wrist speed, turning it clockwise or counterclockwise.
     * (If using CTRE) Uses set method
     * @param speed -Type "double", variable limits between -1.0 and 1.0. -1.0 is full speed down, and 1.0 is full speed up. TODO: make sure -1 is down and 1 is up.
     * @see -Link to set(double) method: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#set(double).
     */
    public void setWristSpeed(double speed) {
        isManual = true;
        if(wrist.getPosition() >= Constants.kMechanisms.maxWristPosition
            || wrist.getPosition() <= Constants.kMechanisms.minWristPosition){
                if(speed > Constants.kMechanisms.maxWristSpeed){
                    speed = Constants.kMechanisms.maxWristSpeed;
                }else if(speed < -Constants.kMechanisms.maxWristSpeed){
                    speed = -Constants.kMechanisms.maxWristSpeed;
                }
        }
        if(speed == 0){
            wristSetpoint = wrist.getPosition();
            isManual = false;
        }
        
        if(speed <= kMechanisms.maxWristSpeed * maxWristSpeed)
        {
            wrist.setSpeed(speed);
        }
        else
        {
            wrist.setSpeed(kMechanisms.maxWristSpeed * maxWristSpeed);
        }
    }

    /**
     * Sets both elevator setpoint and wrist setpoint, which directly states where the elevator and wrist should go to respectively.
     * @see -Affects "moveElevThenArm" inputs, and "moveArmThenElev" inputs.
     * @param wristSetpoint -Type "double", Sets the wristSetPoint variable, which directly controls where the wrist should try to go to.TODO: wristSetpoint UNIT UNKNOWN.
     * @param elevatorSetpoint -Type "double", Sets the elevatorSetpoint variable, which directly controls where the elevator should try to go to.TODO: elevatorSetpoint UNIT UNKNOWN.
     */
    public void setAllMechanismsSetpoint(double wristSetpoint, double elevatorSetpoint) {
        setWristSetpoint(wristSetpoint);
        setElevatorSetpoint(elevatorSetpoint);
    }

    /**
     * Sets the state for mechanisms, deciding what handleState does.
     * States includes: "Reef1" through "Reef4", "AlgaeL2" and "AlgaeL3", "AlgaeProcessor", "AlgaeNet", "Store", "CoralFeeder" and "Climb".
     * @param state -Type "MechanismsState", controls the state that the "handleState" method is in. 
     */
    public void setState(MechanismsState state){
        this.state = state;
    }

    /**
     * Updates the class's elevator setpoint variable and waits for the elevator to reach position before updating the arm setpoint.
     * Uses isAtSetpoint method.
     * @param elevatorSetpoint -Type "double", current elevator setpoint. Used to update the class's elevator setpoint.
     * @param wristSetpoint -Type "double", current wrist setpoint. Used to update the class's elevator setpoint.
     * @param deadzone -Type "double", amount of error allowed to determine if the elevator is in position.
     * @see -isAtSetpoint method is located under: frc.robot.utils.motor.CTREMotor.
     */
    public void moveElevThenArm(double elevatorSetpoint, double wristSetpoint, double deadzone){
        setElevatorSetpoint(elevatorSetpoint);
        if(elevator.isAtSetpoint(deadzone)){
            setWristSetpoint(wristSetpoint);
        }else{
            setWristSetpoint(this.wristSetpoint);
        }
    }
    /**
     * Updates the class's arm setpoint variable and waits for the arm to reach position before updating the elevator setpoint.
     * Uses isAtSetpoint method.
     * @param elevatorSetpoint -Type "double", current elevator setpoint. Used to update the class's elevator setpoint.
     * @param wristSetpoint -Type "double", current wrist setpoint. Used to update the class's elevator setpoint.
     * @param deadzone -Type "double", amount of error allowed to determine if the wrist is in position.
     * @see -isAtSetpoint method is located under: frc.robot.utils.motor.CTREMotor.
     */
    public void moveArmThenElev(double wristSetpoint, double elevatorSetpoint, double deadzone){
        setWristSetpoint(wristSetpoint);
        if(wrist.isAtSetpoint(deadzone)){
            setElevatorSetpoint(elevatorSetpoint);
        }else{
            setElevatorSetpoint(this.elevatorSetpoint);
        }
    } 

    /**
     * Sets the elevator setpoint and the wrist setpoint based on the current mechanisms state.
     * States include: "ReefL1" through "ReefL4", "AlgaeL2" and "AlgaeL3", "AlgaeProcessor", "AlgaeNet", "Store", "CoralFeeder", and "Climb".
     */
    private void handleState() {
        defaultState();
        switch (state) {
            case ReefL1:
                if(elevator.getPosition() >= 0.6){
                    moveElevThenArm(0.58, 0.31325, 0.05);
                }else{
                    if(wrist.isAtSetpoint(0.05)){
                        moveElevThenArm(0.148, 0.31325, 0.05);
                    }else{
                        moveArmThenElev(0.31325, 0.58, 0.05);
                    }
                }
                break;
                
            case ReefL2:
                maxWristSpeed = 0.15;
                moveElevThenArm(0.425, 0.39725, 0.01);
                break;

            case ReefL3:
                moveElevThenArm(0.645, 0.39725, 0.01);
                break;

            case ReefL4:
                moveElevThenArm(1.01, 0.40122, 0.2);
                if(elevator.isAtSetpoint(0.2)){
                    elevatorMaxSpeed = 0.25;
                }else{
                    elevatorMaxSpeed = kMechanisms.elevatorMaxSpeed;
                }
                break;

            case AlgaeL2:
                moveElevThenArm(0.4444, 0.3104, 0.01);
                break;

            case AlgaeL3:
                moveElevThenArm(0.647, 0.3104, 0.01);
                break;

            case AlgaeProcessor:
                setAllMechanismsSetpoint(0.31325, 0.154);
                break;

            case AlgaeNet:
                maxWristSpeed = 0.15;
                moveElevThenArm(0.8431, 0.0502, 0.01);
                break;

            case Store:
            case StoreCoral:
                moveElevThenArm(0.148, 0.31325, 0.05);
                break;

            case CoralFeeder:
                setAllMechanismsSetpoint(-0.0727, 0.1676);
                break;

            case Climb:
                setAllMechanismsSetpoint(0, 0);
                break;
            
            case Manual:
                break;

            default:
                elevatorMaxSpeed = Constants.kMechanisms.elevatorMaxSpeed;
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;
        }
    }
    public void periodic() {
        handleState();
        elevator.setSpeedLimits(elevatorMaxSpeed, -elevatorMaxSpeed);
        wrist.setSpeedLimits(maxWristSpeed, -maxWristSpeed);
        if (!isManual){
            elevator.setSetpoint(elevatorSetpoint, false);
            wrist.setSetpoint(wristSetpoint, false);
        }

        // Full speed for wrist and elevator default
        maxElevatorSpeed = elevatorMaxSpeed;

        // Safety booleans for safty logic
        // Order of what moves and if it should reduce maximum speed
        boolean unsafeWrist = false;
        boolean unsafeWristSpeeds = false;
        boolean unsafeElev = false;
        boolean unsafeElevSpeeds = false;

        // Will not move mechanism if true
        boolean deniedElev = false;
        boolean deniedWrist = false;

        // Safety logic for states on which thing moves first.

        /*CHECKS INCLUDE:
         * if elevator is low enough for wrist to hit robot, do not let wrist move far enough to hit robot if elevator stays low enough to be a danger,
         * else if the elevator will move outside of the dangerous range, move elevator then the wrist.
         * 
         * if wrist is angled enough to hit robot, do not let the elevator move far enough to hit the robot if the wrist stays angled enough to be a danger,
         * else if the wrist will move outside of the dangerous range, move wrist then elevator.
         * 
         * if the elevator is high enough that the wrist max rotation speed could be a danger, limit the max wrist speed.
         * 
         * if the elevator is high enough that the elevator max speed could be a danger, limit the max elevator speed. (in the case of near max elev height, ect. may not be used)
         */
         /* 
        // If the elevator wants to move down beyond where the wrist can move where ever
        if(elevatorSetpoint < kMechanisms.elevatorMinSafeWristHeight)
        {
            //say a collision from moving the wrist is possible
            unsafeWrist = true;
        }
        // If the wrist wants to move beyond the safe limits
        if(wristSetpoint < kMechanisms.wristMinSafeRotation || wristSetpoint > kMechanisms.wristMaxSafeRotation)
        {
            //say a collision from moving the elevator down is possible.
            unsafeElev = true;
        }
        //if the wrist is currently in a position it can hit the robot at an elevator height,
        if(wrist.getPosition() < kMechanisms.wristMinSafeRotation || wrist.getPosition() > kMechanisms.wristMaxSafeRotation)
        {
            //and if the wrist plans on staying there, AND if the elevator wants to move to a spot the wrist could hit the robot,
            if(unsafeWrist&& unsafeElev)
            {
                //keep the elevator from moving.
                deniedElev = true;
            }
        }
        //if the elevator is below the minimum safe height for the wrist,
        if(elevator.getPosition() < kMechanisms.elevatorMinSafeWristHeight)
        {
            //and the elevator plans on staying there, AND the wrist wants to move beyond the safe bounds,
            if(unsafeElev && unsafeWrist)
            {
                //keep the wrist from moving.
                deniedWrist = true;
            }
        }
        //if the elevator is high enough that the max wrist speed can cause problems,
        if(elevator.getPosition() > kMechanisms.elevatorMaxSafeWristHeight)
        {
            //say the wrist should not move at max speed.
            unsafeWristSpeeds = true;
        }
        // if the elevator is high enough that its max speed can cause problems,
        if(elevator.getPosition() > kMechanisms.elevatorMaxSafeHeight)
        {
            //say the elevator should not move at max speed.
            unsafeElevSpeeds = true;
        }


        //if the wrist is told to not move, set the elevator setpoint to current position.
        if(deniedWrist)
        {
            //wristSetpoint = wrist.getPosition();
        }
        //if the elevator is told not to move, set the elevator setpoint to current position.
        if(deniedElev)
        {
            //elevatorSetpoint = elevator.getPosition();
        }

        //if the elevator is low enough the wrist could hit the frame AND the elevator is trying to move out of the dangerous range,
        if(unsafeWrist)
        {
            //move the elevator first, letting it get out of the way, and let the wrist move after.
            moveElevThenArm(elevatorSetpoint, wristSetpoint, 0);
        }
        else
        //if the wrist is angled enough the elevator could make it hit the frame AND the wrist is trying to move out of the dangerous range,
        if(unsafeElev)
        {
            //move the arm first, letting it get out of the way, and let the elevator move after.
            moveArmThenElev(wristSetpoint, elevatorSetpoint, 0);
        }

        //if the wrist is high enough to cause issues at max speed,
        if(unsafeWristSpeeds)
        {
            //reduce it to the reduced speed limit.
            maxWristSpeed = kMechanisms.maxWristReducedSpeed;
        }
        //if the elevator is high enough to cause issues at max speed,
        if(unsafeElevSpeeds)
        {
            //reduce it to the reduced speed limit.
            maxElevatorSpeed = kMechanisms.maxElevReducedSpeed;
        }

        // Sets the speed of the wrist and elevator to the decided max speed
        wrist.setSpeedLimits(maxWristSpeed, -maxWristSpeed);
        elevator.setSpeedLimits(elevatorMaxSpeed, -elevatorMaxSpeed);

        /*NOTE:
         *if the elevator or wrist is denied from moving, the setpoint for them will be equal to the current position they are in.
         *
         *If both the elevator and the wrist are deemed unsafe (they are trying to move to an area that makes the limits matter), BUT the elevator is not low enough OR the wrist is not angled enough,
         *it will move until it reaches one of the limited zones.
         *
         *Unsafe is defined as: A part is planning on moving to a spot in which moving the other part could cause the robot to hit itself
         *EXAMPLE: Wrist is in a position that moving the elevator down would cause it to hit itself, unsafeElev = true.
         *
         *Denied is defined as: Both parts are "unsafe", AND one of the parts are currently in the limit zones, it will cause the part not in the zone to not be allowed to move
         *EXAMPLE: Elevator is fully down, it wants to stay there, and the wrist wants to move outside of the safe area, it will cause the wrist to not move.
         *EXAMPLE 2: Elevator is extended beyond limit, it wants to move into limit, and the wrist is trying to move to its "unsafe" zone, the elevator will not be allowed to move if the wrist reaches the "unsafe zone" first,
         *OR the wrist will not be allowed to move if the elevator reaches its "unsafe" zone first.
         */

        elevator.periodic();
        wrist.periodic();

        SmartDashboard.putNumber("elevator Position", elevator.getPosition());
        SmartDashboard.putNumber("wrist Position", wrist.getPosition());
        SmartDashboard.putBoolean("elevator isAtSetpoint", elevator.isAtSetpoint(0.01));
        SmartDashboard.putBoolean("wrist isAtSetpoint", wrist.isAtSetpoint(0.01));

        SmartDashboard.putBoolean("denied wrist", deniedWrist);
        SmartDashboard.putBoolean("denied elevator", deniedElev);
        SmartDashboard.putBoolean("unsafe wrist", unsafeWrist);
        SmartDashboard.putBoolean("unsafe elevator", unsafeElev);
    }

    public void simulationPeriodic(){
        elevatorMech2d.setLength(elevator.getPosition());
        wristMech2d.setAngle(wrist.getPosition());

        SmartDashboard.putData("Mech2d", mech2d);
        SmartDashboard.putNumber("Elevator Pos", elevator.getPosition());
        SmartDashboard.putNumber("Wrist", wrist.getPosition());
    }

    private void defaultState(){
        elevatorMaxSpeed = Constants.kMechanisms.elevatorMaxSpeed;
        maxWristSpeed = Constants.kMechanisms.maxWristSpeed;
    }

    public void zeroMechanisms(){
        elevator.setPosition(0);
        wrist.setPosition(0);
        state = MechanismsState.Manual;
    }
}
