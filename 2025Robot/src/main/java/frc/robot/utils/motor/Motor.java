package frc.robot.utils.motor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.utils.encoder.Encoder;

/**
 * Public interface for the motors. Used to streamline code.
 * @see -Look in individual motor files for links to documentation for vendor library methods on each method
 */
public interface Motor {

    /**
     * Sets motor speed
     * @param speed -Type "double", range is -1 to 1. -1 is full speed back, 1 is full speed forward.
     */
    public void setSpeed(double speed);

    /**
     * @return Type "double", returns the speed of the motor. Ranges between -1 and 1. -1 is full speed back, 1 is full speed forward.
     * @see #
     */
    public double getSpeed();

    /**
     * Sets the motor position, overwriting the old variable.
     * @param position -Type "double", position for the motor in digrees.
     */
    public void setPosition(double position);
    
    /**
     * Gets the current position of the motor
     * @return -Type "double", the motor angle in digrees.
     */
    public double getPosition();

    /**
     * Sets the point the motor will try to go to
     * @param setPoint -Type "double". Unknown unit.
     */
    public void setSetpoint(double setPoint, boolean useSimFF);

    /**
     * Sets the PID for the motors.
     * @param p -Type "double", majorly increases the speed, if this is too low the motor will not move.
     * @param i -Type "double" it do schenanigans.
     * @param d -Type "double" minorly adjusts the end, allowing it to not ocelate.
     */
    public void setPID(double p, double i, double d);
    
    /**
     * @return -Type "PIDController", returns the pid controller object, containing all of its parameters as an object.
     */
    public PIDController getPID();

    /**
     * Sets positive and negative speed limits to the motor
     * @param positiveSpeed -Type "double", the speed limit in the positive direction
     * @param negativeSpeed -Type "double", the speed limit in the negative direction
     */
    public void setSpeedLimits(double positiveSpeed, double negativeSpeed);

    /**
     * Gets the motor encoder.
     * @return -Type "Encoder", gets the encoder object containing all of its parameters as an object.
     */
    public Encoder getEncoder();

    /**
     * Gets the current voltage of the motor.
     * @return Type "double", voltage for motor between -40.96 and 40.95 volts.
     */
    public double getVoltage();

    /**
     * Checks if the motor is at its setpoint.
     * @param deadzone -Type "double", the allowed of clearance between the setpoint and the current position confirm it is in the setpoint.
     * @return Type "boolean", true if the motor is within the setpoint + or - the deadzone, false if otherwise.
     */
    public boolean isAtSetpoint(double deadzone);

    /**
     * @return Type "SimpleMotorFeedForward", an object containing all the parameters for the feedforward as properties.
     */
    public SimpleMotorFeedforward getFeedForward();

    /**
     * Sets the feedforward object's parameters. It is used to keep the motor from falling down due to gravity or other constant forces
     * @param kS -Type "double", sets the motor feedforward kS.
     * @param kV -Type "double", sets the motor feedforward kV.
     * @param kA -Type "double", sets the motor feedforward KA.
     */
    public void setFeedFoward(double kS, double kV, double kA);

    /** Must be called by the subystems periodic method */
    public void periodic();
}
