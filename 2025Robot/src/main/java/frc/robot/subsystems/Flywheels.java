package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motor.Motor;
import frc.robot.utils.motor.REVMotor;

public class Flywheels extends SubsystemBase {

    public Motor effectorWheels;
    
    /**
     * State enum for the flywheels, states include "CoralIntake", "CoralOutake", "AlgaeIntake", "AlgaeOutake".
     * These states activate what motors on the end effector do what.
     */
    private enum WheelsStates{
        CoralIntake,
        CoralOutake,
        AlgaeIntake,
        AlgaeOutake
    };

    /**
     * Constructor for class "Flywheels". Initilizes the end effector motor.
     */
    public Flywheels(){
         effectorWheels = new REVMotor(0);
    }

    /**
     * Sets the speed of the end effector using the .set method in TalonFX.
     * Review the following link for more information (for CTRE motors):
     * {@link https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#set(double)}
     * @param speed -Type "Double", used to set the speed of the motor for the effector wheels. valid values are between -1.0 and 1.0.
     */
    public void setSpeed(double speed){
        effectorWheels.setSpeed(speed);
    }

    /**
     * Returns the speed of the motor as a double.
     * Review the following link for more information (for CTRE motors):
     * {@link https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#get()}
     * @return -The speed of the motors, as a double between -1.0 and 1.0.
     */    
    public double getSpeed(){
        return effectorWheels.getSpeed();
    }
    
    /**
     * Sets the state of the end effector motor, using the .set(double) method.
     * Unlike the method "setSpeed", uses states rather than a direct double.
     * States include; CoralIntake, CoralOutake, AlgaeIntake, AlgaeOutake.
     * Review the following link for more information on the used method (for CTRE motors):
     * {@link https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/TalonFX.html#set(double)}
     * @param speed -Type "WheelsStates", states for taking or dropping algae or coral. Valid values are between -1.0 and 1.0.
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

            default:
                break;
        }
    }
}
