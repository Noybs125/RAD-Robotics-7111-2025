package frc.robot.utils.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

public class MotorConstants {
    public DCMotor motor;
    public int id;
    public boolean isCCW;
    public double gearRatio;
    public double currentLimit;
    public PIDController pid;

    public MotorConstants(DCMotor motor, int id, boolean isCCW, double gearRatio, double currentLimit, PIDController pid){
        this.motor = motor;
        this.id = id;
        this.isCCW = isCCW;
        this.gearRatio = gearRatio;
        this.currentLimit = currentLimit;
        this.pid = pid;
    }
}