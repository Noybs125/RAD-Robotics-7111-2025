package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Mechanisms.MechanismsState;
import frc.robot.subsystems.Swerve.SwerveState;
import frc.robot.subsystems.Vision.VisionState;

import com.ctre.phoenix6.mechanisms.MechanismState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperStructure extends SubsystemBase {
    private Swerve swerve;
    private Vision vision;
    private Field field;
    private Sensors sensors;
    private Mechanisms mechanisms;
    private Flywheels flywheels; 
    private ActualState actualRobotState;
    private ControlState controlRobotState;
    private Pose2d nearestZone;
    public boolean hasCoral;
    public boolean hasAlgae;
    public Deepclimb deepClimb;

    public SwerveState swerveState = SwerveState.DefaultState;

    /**
     * States for what constrolstate we need.
     * States include; "XButton", "AButton", "BButton", "YButton", "ReefFeeder", "DeepClimb", and "Default"
     */
    public enum ControlState {
        XButton,
        AButton,
        BButton,
        YButton,
        SelectButton,
        StartButton,
        DPadUp,
        DPadDown,
        DPadLeft,
        DPadRight,
        LeftStick,
        RightStick,
        LeftTrigger,
        RightTrigger,
        LeftBumper,
        RightBumper,
        Default,
    }

    /**
     * States for what state the robot is actively in.
     * States include; "coralL1" through "coralL4", "coralFeeder", "algaeProcessor", "algaeNet", "algaeL2", "algaeL3", "deepClimb" and "defaultState"
     */
    public enum ActualState {
        coralL1,
        coralL2,
        coralL3,
        coralL4,
        coralFeeder,
        algaeProcessor,
        algaeNet,
        algaeL2,
        algaeL3,
        deepClimbRetracted,
        deepClimbExtended,
        stow,
        stowWithCoral,
        defaultState,
    }

    /**
     * Constructor for the SuperStructure. initilizes the states for the robot, and sets all the parameters equal to local variables.
     * @param swerve -Type "Swerve", sets equal to local swerve.
     * @param vision -Type "Vision", sets equal to local vision.
     * @param field -Type "Field", sets equal to local field.
     * @param sensors -Type "Sensors", sets equal to local sensors.
     * @param mechanisms -Type "Mechanisms", sets equal to local mechanisms.
     * @param flywheels -Type "Flywheels", sets equal to local flywheels.
     */
    public SuperStructure(Swerve swerve, Vision vision, Field field, Sensors sensors, Mechanisms mechanisms, Flywheels flywheels, Deepclimb deepClimb){
        this.swerve = swerve;
        this.vision = vision;
        this.field = field;
        this.sensors = sensors;
        this.mechanisms = mechanisms;
        this.flywheels = flywheels;
        this.deepClimb = deepClimb;

        controlRobotState = ControlState.Default;
        actualRobotState = ActualState.defaultState;
        nearestZone = field.getNearestZone(swerve.getPose());
    }


    /**
     * Periodic method called 50 times per second.
     * <p>
     * Calls {@link #manageControlState()} and {@link #manageActualState()} which act on the current states.
     * Also checks if the robot has coral, algae, or nothing.
     */
    public void periodic() {
        //manageControlState();
        manageActualState();
        nearestZone = field.getNearestZone(swerve.getPose());

        if (actualRobotState == ActualState.coralFeeder && sensors.isBeamBroken()) {
            hasCoral = true;
            hasAlgae = false;
        }else if (actualRobotState == ActualState.algaeL2 && sensors.isBeamBroken() || actualRobotState == ActualState.algaeL3 && sensors.isBeamBroken()) {
            hasAlgae = true;
            hasCoral = false;
        }else {
            hasAlgae = false;
            hasCoral = false;
        }

        swerve.setSubtractedSpeed(mechanisms.getElevatorHeight());
        swerve.setRotationSetpoint(field.getNearestZone(swerve.getPose()).getRotation().getDegrees());
    }

    /**
     * Sets the actual robot state to the equivilent for control state.
     * This makes the robot turn into the state and start running the appropriate code.
     */
    private void manageControlState(){
        switch (controlRobotState) {   
            case XButton:
                //checks for whether it should align for reef or processor
                switch(actualRobotState){
                    case coralFeeder:
                        actualRobotState = ActualState.stowWithCoral;
                        break;
                    default:
                        actualRobotState = ActualState.coralFeeder;
                        break;
                }
                break;
            case AButton:
                //checks for whether it should score coral or intake algae
                actualRobotState = ActualState.coralL2;
                break;
            case BButton:
                //checks for whether it should score coral or intake algae
                actualRobotState = ActualState.coralL3;
                break;
            case YButton:
                //checks for whether it should score coral on L4 or score algae in net
                boolean isNet = false; //true if we have algae
                actualRobotState = isNet
                    ? ActualState.algaeNet
                    : ActualState.coralL4;
                break;
            case SelectButton:
                actualRobotState = ActualState.deepClimbRetracted;
                break;
        
            case StartButton:
                actualRobotState = ActualState.deepClimbExtended;
                break;

            case Default:
            default:
                actualRobotState = ActualState.defaultState;
                break;
        }
    }

    /**
     * Calls the state functions for the set state.
     * @see -{@link #coralL1()} {@link #coralL2()} {@link #coralL3()} {@link #coralL4()} {@link #coralFeeder()} {@link #algaeL2()} {@link #algaeProcessor()} {@link #algaeNet()} {@link #deepClimb()} {@link #defaultState()}
     */
    private void manageActualState(){
        switch(actualRobotState){
            case coralL1:
                coralL1();
                break;
            case coralL2:
                coralL2();
                break;
            case coralL3:
                coralL3();
                break;
            case coralL4:
                coralL4();
                break;
            case coralFeeder:
                coralFeeder();
                break;
            case algaeProcessor:
                algaeProcessor();
                break;
            case algaeNet:
                algaeNet();
                break;
            case algaeL2:
                algaeL2();
                break;
            case algaeL3:
                algaeL3();
                break;
            case deepClimbRetracted:
                deepClimbRetracted();
                break;
            case deepClimbExtended:
                deepClimbExtended();
                break;
            case defaultState:
                defaultState();
                break;
            case stow:
                stow();
                break;
            case stowWithCoral:
                stowCoral();
                break;
            default:
                break;
        }
    }

    /**
     * Sets the controlRobotState
     * @param state -Type "ControlState", controlRobotState is set equal to this.
     */
    private void setRobotState(ControlState state){
        controlRobotState = state;
        manageControlState();
    }

    /**
     * Sets the actualRobotState, this is used only in Auto
     * @param state -Type "ActualState", actualRobotState is set equal to this
     */
    private void setActualState(ActualState state){
        actualRobotState = state;

    }

    /**
     * Runs the controlRobotState input. Uses runOnce, and {@link #setRobotState(ControlState)}.
     * @param state -The state used for setRobotState.
     * @return -Type "Command", an object used to run a command.
     * @see -Link to runOnce method: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Subsystem.html#runOnce(java.lang.Runnable).
     */
    public Command setRobotStateCommand(ControlState state){
        return runOnce(() -> setRobotState(state));
    }
    /**
     * Runs the actualRobotState input. Uses runOnce, and {@link #setActualState(ActualState)}.
     * @param state -The state used for setActualState.
     * @return -Type "Command", an object used to run a command.
     * @see -Link to runOnce method: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Subsystem.html#runOnce(java.lang.Runnable).
     */
    public Command setActualStateCommand(ActualState state) {
        return runOnce(() -> setActualState(state));
    }

    /**
     * Sets the SwerveState to the "Vision" state
     */
    private void coralL1(){
        mechanisms.setState(Mechanisms.MechanismsState.ReefL1);
        swerve.setState(swerveState);
    }

    /**
     * Sets the mechanismsState to "ReefL2"
     */
    private void coralL2(){
        mechanisms.setState(Mechanisms.MechanismsState.ReefL2);
        swerve.setState(swerveState);
        //if(vision.isAtTarget(0, null, null, 0));
    }

    /**
     * Sets the mechanismsState to "ReefL3"
     */
    private void coralL3(){
        mechanisms.setState(Mechanisms.MechanismsState.ReefL3);
        swerve.setState(swerveState);
    }

    /**
     * Sets the mechanismsState to "ReefL4"
     */
    private void coralL4(){
        mechanisms.setState(Mechanisms.MechanismsState.ReefL4);
        swerve.setState(swerveState);
    }

    /**
     * Sets the mechanismsState to "CoralFeeder"
     */
    private void coralFeeder(){
        mechanisms.setState(Mechanisms.MechanismsState.CoralFeeder);
    }

    /**
     * Sets the mechanismsState to "AlgaeProcessor"
     */
    private void algaeProcessor(){
        mechanisms.setState(Mechanisms.MechanismsState.AlgaeProcessor);
    }
    
    /**
     * Sets the mechanismsState to "AlgaeNet"
     */
    private void algaeNet(){
        mechanisms.setState(Mechanisms.MechanismsState.AlgaeNet);
    }

    /**
     * Sets the mechanismsState to "AlgaeL2"
     */
    private void algaeL2(){
        mechanisms.setState(Mechanisms.MechanismsState.AlgaeL2);
    }

    /**
     * Sets the mechanismsState to "AlgaeL3"
     */
    private void algaeL3(){
        mechanisms.setState(Mechanisms.MechanismsState.AlgaeL3);
    }

    private void stowCoral(){
        mechanisms.setState(Mechanisms.MechanismsState.StoreCoral);
    }

    private void stow(){
        mechanisms.setState(Mechanisms.MechanismsState.Store);
    }

    /**
     * Sets the mechanismsState to "Climb"
     */
    private void deepClimbRetracted(){
        deepClimb.setState(Deepclimb.ClimbStates.Retracted);
    }

    private void deepClimbExtended(){
        deepClimb.setState(Deepclimb.ClimbStates.Extended);
    }

    /**
     * Checks if the elevator height is greater 20% max height, setting SwerveState to "lowerSpeed" if the case, otherwise set to "DefaultState"
     */
    private void defaultState(){

    }

    public Command setSwerveState(SwerveState swerveState) {
        return new InstantCommand(() -> this.swerveState = swerveState);
    }

    public Command useLeftAlignment(){
        return runOnce(() -> {
            vision.setState(VisionState.LeftReef);
            swerveState = SwerveState.VisionWithGyro;
        });
    }
    public Command useRightAlignment(){
        return runOnce(() -> {
            vision.setState(VisionState.RightReef);
            swerveState = SwerveState.VisionWithGyro;
        });
    }
    public Command useCenterAlignment(){
        return runOnce(() -> {
            vision.setState(VisionState.LeftReef);
            swerveState = SwerveState.VisionWithGyro;
        });
    }
    public Command useNoAlignment(){
        return runOnce(() -> {
            swerveState = SwerveState.DefaultState;
        });
    }
}
