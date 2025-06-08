package frc.robot.utils.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

public class MotorConstants {
    public DCMotor dcMotor;
    public int id;
    public boolean isCCW;
    public boolean isBreakMode;
    public double gearRatio;
    public int currentLimit;
    public PIDController pid;

    public MotorConstants(DCMotor dcMotor, int id, boolean isCCW, boolean isBreakMode, double gearRatio, int currentLimit, PIDController pid){
        this.dcMotor = dcMotor;
        this.id = id;
        this.isCCW = isCCW;
        this.isBreakMode = isBreakMode;
        this.gearRatio = gearRatio;
        this.currentLimit = currentLimit;
        this.pid = pid;
    }
}