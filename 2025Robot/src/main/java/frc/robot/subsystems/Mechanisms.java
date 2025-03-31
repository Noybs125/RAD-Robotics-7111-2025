package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    private PIDController selectedArmPID = Constants.kMechanisms.armPID;

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
        SimpleMotorFeedforward twoMotorsSMFF = Constants.kMechanisms.elevatorFF; /*new SimpleMotorFeedforward(0, 0)*/;
        WpiEncoder twoMotorsEncoder = new WpiEncoder(0, 1);

        elevator = new TwoMotors(
            new CTREMotor(8, twoMotorsEncoder, 1, twoMotorsPID, twoMotorsSMFF, null, 
                new ElevatorSimMotor(null, Constants.kSimulation.elevatorSimGearRatio, Constants.kSimulation.elevatorPid, Constants.kSimulation.elevatorFF, Constants.kSimulation.elevatorSimConstants), Constants.kMechanisms.elevator1Config()),
            new CTREMotor(2, twoMotorsEncoder, 1, twoMotorsPID, twoMotorsSMFF, null,
                new ElevatorSimMotor(null, Constants.kSimulation.elevatorSimGearRatio, Constants.kSimulation.elevatorPid, Constants.kSimulation.elevatorFF, Constants.kSimulation.elevatorSimConstants), Constants.kMechanisms.elevator2Config()));

        wrist = new CTREMotor(16, null, kMechanisms.wristGearRatio, Constants.kMechanisms.armPID, Constants.kMechanisms.wristFF, Constants.kMechanisms.wristArmFF, new ArmSimMotor(null, null, null, null), Constants.kMechanisms.wristConfig());

        wrist.setSpeedLimits(Constants.kMechanisms.maxWristSpeed, -Constants.kMechanisms.maxWristSpeed);
        wrist.setPosition(0);
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
            && speed >= 0){
                wrist.setSpeed(0);
        }
        else if(wrist.getPosition() < Constants.kMechanisms.minWristPosition && speed <= 0){
            wrist.setSpeed(0); 
        }
        if(speed == 0){
            //selectedArmPID = Constants.kMechanisms.armManualPID;
            wristSetpoint = wrist.getPosition();
            isManual = false; 
        }
        
        if(speed <= -maxWristSpeed)
        {
            wrist.setSpeed(-maxWristSpeed);
        }
        else if(speed >= maxWristSpeed)
        {
            wrist.setSpeed(maxWristSpeed);
        } else {
            wrist.setSpeed(speed);
        }
        wrist.setSpeed(speed);
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
                if(elevator.getPosition() >= 0.65){
                    moveElevThenArm(0.58, 0.298, 0.1);
                }else{
                    if(wrist.isAtSetpoint(0.025)){
                        moveElevThenArm(0.131, 0.290, 0.05);
                    }else{
                        moveArmThenElev(0.290, 0.58, 0.025);
                    }
                }
                break;
                
            case ReefL2:
                //maxWristSpeed = 0.15;
                moveElevThenArm(0.425, 0.38725, 0.01);
                break;

            case ReefL3:
                moveElevThenArm(0.630, 0.38725, 0.01);
                break;

            case ReefL4:
                moveElevThenArm(1.00, 0.40122, 0.2);
                if(elevator.isAtSetpoint(0.2)){
                    elevatorMaxSpeed = 7;
                }else{
                    elevatorMaxSpeed = kMechanisms.elevatorMaxSpeed;
                }
                break;

            case AlgaeL2:
                moveElevThenArm(0.4444, 0.305, 0.01);
                break;

            case AlgaeL3:
                moveElevThenArm(0.647, 0.305, 0.01);
                break;

            case AlgaeProcessor:
                setAllMechanismsSetpoint(0.31325, 0.154);
                break;

            case AlgaeNet:
                maxWristSpeed = 1;
                moveElevThenArm(0.8431, 0.0502, 0.01);
                break;

            case Store:
                moveArmThenElev(0.0, 0.0, 0.01);
                break;
            case StoreCoral:
                moveElevThenArm(0.148, 0.31325, 0.05);
                break;

            case CoralFeeder:
                setAllMechanismsSetpoint(-0.0725, 0.1676);
                break;

            case Climb:
                setAllMechanismsSetpoint(0, 0);
                break;
            
            case Manual:
                //selectedArmPID = Constants.kMechanisms.armManualPID;
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
        elevator.setSpeedLimits(elevatorMaxSpeed, -elevatorMaxSpeed/2);
        wrist.setSpeedLimits(maxWristSpeed, -maxWristSpeed);
        if (!isManual){
            elevator.setSetpoint(elevatorSetpoint, false);
            wrist.setSetpoint(wristSetpoint, false);
        }

        // Full speed for wrist and elevator default
        maxElevatorSpeed = elevatorMaxSpeed;
        
        /**Unused */
        double potentialWristSetpoint = 0;
        /**Unused */
        boolean doArmThenElev = false;
       /*  // Safety logic for states on which thing moves first.

        // If elevator is too low and the robot wants to move wrist down too low
        if(elevator.getPosition() <= kMechanisms.elevatorMinSafeWristHeight && (wristSetpoint >= kMechanisms.wristMaxSafeRotation || wristSetpoint <= kMechanisms.wristMinSafeRotation))
        {
            // Set wrist position setpoint to the edge of the safe range
            if(wristSetpoint <= kMechanisms.wristMinSafeRotation)
            {
                potentialWristSetpoint = wristSetpoint;
                wristSetpoint = kMechanisms.wristMinSafeRotation;
            } 
            else if(wristSetpoint >= kMechanisms.wristMaxSafeRotation)
            {
                potentialWristSetpoint = wristSetpoint;
                wristSetpoint = kMechanisms.wristMaxSafeRotation;
            }
            doArmThenElev = true;
        }

        //if elevator is low but not too low, then you can go to a setpoint slightly more permissive than before
        if(elevatorSetpoint <= kMechanisms.elevatorPrecautionWristHeight)
        {
            if(wristSetpoint <= kMechanisms.wristPrecautionaryMinRotation)
            {
                potentialWristSetpoint = wristSetpoint;
                wristSetpoint = kMechanisms.wristPrecautionaryMinRotation;
            }
            else if(wristSetpoint >= kMechanisms.wristPrecautionaryMaxRotation)
            {
                potentialWristSetpoint = wristSetpoint;
                wristSetpoint = kMechanisms.wristPrecautionaryMaxRotation;
            }
            doArmThenElev = true;
        }

        //if the wrist is high enough to cause issues at max speed,
        if(elevator.getPosition() >= kMechanisms.elevatorMaxSafeWristHeight)
        {
            //reduce it to the reduced speed limit.
            maxWristSpeed = kMechanisms.maxWristReducedSpeed;
        }
        //if the elevator is high enough to cause issues at max speed,
        if(elevator.getPosition() >= kMechanisms.elevatorMaxSafeHeight)
        {
            //reduce it to the reduced speed limit.
            maxElevatorSpeed = kMechanisms.maxElevReducedSpeed;
        }

        // Sets the speed of the wrist and elevator to the decided max speed, defaults to the robot's set max speed
        wrist.setSpeedLimits(maxWristSpeed, -maxWristSpeed);
        elevator.setSpeedLimits(elevatorMaxSpeed, -elevatorMaxSpeed);*/

        elevator.periodic();
        wrist.periodic();

        SmartDashboard.putNumber("elevator Position", elevator.getPosition());
        SmartDashboard.putNumber("wrist Position", wrist.getPosition());
        SmartDashboard.putBoolean("elevator isAtSetpoint", elevator.isAtSetpoint(0.01));
        SmartDashboard.putBoolean("wrist isAtSetpoint", wrist.isAtSetpoint(0.01));

        SmartDashboard.putNumber("wristSetpoint", wristSetpoint);
        SmartDashboard.putNumber("elevatorSetpoint", elevatorSetpoint);

        SmartDashboard.putNumber("wrist speed", wrist.getSpeed());
    }

    public void simulationPeriodic(){
        elevatorMech2d.setLength(elevator.getPosition());
        wristMech2d.setAngle(wrist.getPosition());

        SmartDashboard.putData("Mech2d", mech2d);
        SmartDashboard.putNumber("Elevator Pos", elevator.getPosition());
        SmartDashboard.putNumber("Wrist", wrist.getPosition());
    }

    private void defaultState(){
        selectedArmPID = Constants.kMechanisms.armPID;
        elevatorMaxSpeed = Constants.kMechanisms.elevatorMaxSpeed;
        maxWristSpeed = Constants.kMechanisms.maxWristSpeed;
    }

    public void zeroMechanisms(){
        elevator.setPosition(0);
        wrist.setPosition(0);
        state = MechanismsState.Manual;
    }

    public Command setSuppliedWristSpeed(DoubleSupplier speed){
        return new InstantCommand(() -> setWristSpeed(speed.getAsDouble()));
    }
}
