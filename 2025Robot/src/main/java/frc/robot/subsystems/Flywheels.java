package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motor.Motor;
import frc.robot.utils.motor.REVMotor;

public class Flywheels extends SubsystemBase {

    public Motor effectorWheels;
    
    /**
     * State enum for the flywheels, states include "CoralIntake", "CoralOutake", "AlgaeIntake", "AlgaeOutake".
     * These states activate what motors on the end effector do what.
     */
    public enum WheelsStates{
        CoralIntake,
        CoralOutake,
        AlgaeIntake,
        AlgaeOutake,
        Zero,
        Custom,
    };

    /**
     * Constructor for class "Flywheels". Initilizes the end effector motor.
     */
    public Flywheels(){
        effectorWheels = new REVMotor(15);
    }

    /**
     * Sets the speed of the end effector motor.
     * (If using CTRE motors) Uses setSpeed(double) method
     * @param speed -Type "Double", used to set the speed of the motor for the effector wheels. valid values are between -1.0 and 1.0.
     * @see -Link to the .set(double) method: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#set(double).
     */
    public void setSpeed(double speed){
        effectorWheels.setSpeed(speed);
    }

    /**
     * Returns the speed of the motor as a double.
     * (If using CTRE motors) Uses get method to find motor speed
     * @return -The speed of the motors, as a double between -1.0 and 1.0.
     * @see -Link to get method: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#get()
     */    
    public double getSpeed(){
        return effectorWheels.getSpeed();
    }
    
    /**
     * Sets the state of the end effector motor.
     * Unlike the method "setSpeed" in this class, uses states rather than a direct double.
     * States include; CoralIntake, CoralOutake, AlgaeIntake, AlgaeOutake.
     * (If using CTRE motors) Uses Set(double) method. Inputs must be between -1.0 or 1.0.
     * @param speed -Type "WheelsStates", states for taking or dropping algae or coral.
     * @see -Link to the setSpeed method: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#set(double
     */
    public void setSpeedState(WheelsStates speed){
        switch (speed) {
            case CoralIntake:
                effectorWheels.setSpeed(.5);
                break;
        
            case AlgaeIntake:
                effectorWheels.setSpeed(.5);
                break;

            case CoralOutake:
                effectorWheels.setSpeed(-.5);
                break;
            
            case AlgaeOutake:
                effectorWheels.setSpeed(-.5);
                break;
            case Zero:
                effectorWheels.setSpeed(0);
                break;
            case Custom:
                break;
            default:
                break;
        }
    }

    /*public Command setSuppliedSpeed(DoubleSupplier speed){
        return run(() -> effectorWheels.setSpeed(speed.getAsDouble()));
    }*/
}
