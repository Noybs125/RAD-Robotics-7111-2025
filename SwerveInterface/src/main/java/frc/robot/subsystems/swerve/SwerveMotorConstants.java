package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveMotorConstants {
    public DCMotor dcMotor;
    public int id;
    public boolean isCCW;
    public boolean isBreakMode;
    public double gearRatio;
    public int currentLimit;
    public PIDController pid;
    public SparkMaxConfig sparkMaxConfig;
    public TalonFXConfiguration talonFXConfig;

    public SwerveMotorConstants(DCMotor dcMotor, int id, boolean isCCW, boolean isBreakMode, double gearRatio, int currentLimit, PIDController pid, SparkMaxConfig sparkMaxConfig){
        this.dcMotor = dcMotor;
        this.id = id;
        this.isCCW = isCCW;
        this.isBreakMode = isBreakMode;
        this.gearRatio = gearRatio;
        this.currentLimit = currentLimit;
        this.pid = pid;
    }

    public SwerveMotorConstants(DCMotor dcMotor, int id, boolean isCCW, boolean isBreakMode, double gearRatio, int currentLimit, PIDController pid, TalonFXConfiguration talonFXConfig){
        this.dcMotor = dcMotor;
        this.id = id;
        this.isCCW = isCCW;
        this.isBreakMode = isBreakMode;
        this.gearRatio = gearRatio;
        this.currentLimit = currentLimit;
        this.pid = pid;
    }
}