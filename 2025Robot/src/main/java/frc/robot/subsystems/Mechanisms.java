package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kMechanisms;
import frc.robot.utils.encoder.Encoder;
import frc.robot.utils.encoder.RevEncoder;
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
    private MechanismsState state = MechanismsState.Store;

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
        CoralFeeder,
        Climb,
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
        PIDController twoMotorsPID = new PIDController(0.05, 0, 0);
        SimpleMotorFeedforward twoMotorsSMFF = null /*new SimpleMotorFeedforward(0, 0)*/;
        WpiEncoder twoMotorsEncoder = new WpiEncoder(8, 9);
        TalonFXConfiguration oneMotorInverted = new TalonFXConfiguration();
        TalonFXConfiguration twoMotorInverted = new TalonFXConfiguration();
        oneMotorInverted.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        twoMotorInverted.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        elevator = new TwoMotors(new CTREMotor(8, twoMotorsEncoder, 1, twoMotorsPID, twoMotorsSMFF, elevator, oneMotorInverted),
         new CTREMotor(2, twoMotorsEncoder, 1, twoMotorsPID, twoMotorsSMFF, elevator, twoMotorInverted));

        wrist = new CTREMotor(13);
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
            elevatorHeight = ((Math.PI * kMechanisms.elevatorWinchDiameter * elevator.getPosition() / 360.0 / 12) / kMechanisms.elevatorMaxHeight);
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
        elevator.setSpeed(speed);
        isManual = true;
    }

    /**
     * Sets the end effector's wrist speed, turning it clockwise or counterclockwise.
     * (If using CTRE) Uses set method
     * @param speed -Type "double", variable limits between -1.0 and 1.0. -1.0 is full speed down, and 1.0 is full speed up. TODO: make sure -1 is down and 1 is up.
     * @see -Link to set(double) method: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#set(double).
     */
    public void setWristSpeed(double speed) {
        wrist.setSpeed(speed);
        isManual = true;
    }

    /**
     * Sets both elevator setpoint and wrist setpoint, which directly states where the elevator and wrist should go to respectively.
     * @see -Affects "moveElevThenArm" inputs, and "moveArmThenElev" inputs.
     * @param wristSetpoint -Type "double", Sets the wristSetPoint variable, which directly controls where the wrist should try to go to.TODO: wristSetpoint UNIT UNKNOWN.
     * @param elevatorSetpoint -Type "double", Sets the elevatorSetpoint variable, which directly controls where the elevator should try to go to.TODO: elevatorSetpoint UNIT UNKNOWN.
     */
    public void setAllMechanismsSetpoint(double wristSetpoint, double elevatorSetpoint) {
        this.elevatorSetpoint = elevatorSetpoint;
        this.wristSetpoint = wristSetpoint;
        isManual = false;
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
        this.elevatorSetpoint = elevatorSetpoint;
        if(elevator.isAtSetpoint(deadzone)){
            this.wristSetpoint = wristSetpoint;
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
    public void moveArmThenElev(double elevatorSetpoint, double wristSetpoint, double deadzone){
        this.wristSetpoint = wristSetpoint;
        if(wrist.isAtSetpoint(deadzone)){
            this.elevatorSetpoint = elevatorSetpoint;
        }
    } 

    /**
     * Sets the elevator setpoint and the wrist setpoint based on the current mechanisms state.
     * States include: "ReefL1" through "ReefL4", "AlgaeL2" and "AlgaeL3", "AlgaeProcessor", "AlgaeNet", "Store", "CoralFeeder", and "Climb".
     */
    private void handleState() {
        switch (state) {
            case ReefL1:
                elevatorSetpoint = 0.16;
                wristSetpoint = 55;
                break;
                
            case ReefL2:
                elevatorSetpoint = 0.9;
                wristSetpoint = 215;
                break;

            case ReefL3:
                elevatorSetpoint = 1.6;
                wristSetpoint = 110;
                break;

            case ReefL4:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeL2:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeL3:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeProcessor:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeNet:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case Store:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case CoralFeeder:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case Climb:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;
        
            default:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;
        }
    }
    public void periodic() {


        if (isManual) {
            
        }
        else {
            elevator.setSetpoint(elevatorSetpoint);
            wrist.setSetpoint(wristSetpoint);
        }

        elevator.periodic();
        wrist.periodic();
        
        handleState();
    }

    public void simulationPeriodic(){
        elevatorMech2d.setLength(elevator.getPosition());
        wristMech2d.setAngle(wrist.getPosition());

        SmartDashboard.putData("Mech2d", mech2d);
        SmartDashboard.putNumber("Elevator Pos", elevator.getPosition());
        SmartDashboard.putNumber("Wrist", wrist.getPosition());
    }
}
