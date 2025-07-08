package frc.robot.subsystems.swerve.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveMotorConfig {
    public DCMotor dcMotor;
    public int id;
    public boolean isCCW;
    public boolean isBreakMode;
    public double gearRatio;
    public double momentOfInertia;
    public int currentLimit;
    public PIDController pid;
    public SimpleMotorFeedforward ff;
    public SparkBaseConfig sparkMaxConfig;
    public TalonFXConfiguration talonFXConfig;

    public SwerveMotorConfig(
        DCMotor dcMotor, int id, boolean isCCW, boolean isBreakMode, double gearRatio, double momentOfInertia, 
        int currentLimit, PIDController pid, SimpleMotorFeedforward ff, SparkBaseConfig sparkMaxConfig){
            this.dcMotor = dcMotor;
            this.id = id;
            this.isCCW = isCCW;
            this.isBreakMode = isBreakMode;
            this.gearRatio = gearRatio;
            this.momentOfInertia = momentOfInertia;
            this.currentLimit = currentLimit;
            this.pid = pid;
            this.ff = ff;
            this.sparkMaxConfig = sparkMaxConfig;

            sparkMaxConfig = isBreakMode
                ? sparkMaxConfig.idleMode(IdleMode.kBrake)
                : sparkMaxConfig.idleMode(IdleMode.kCoast);
            sparkMaxConfig
                .inverted(isBreakMode)
                .smartCurrentLimit(currentLimit);
            sparkMaxConfig.closedLoop
                .p(pid.getP())
                .i(pid.getI())
                .d(pid.getD())
                .velocityFF(ff.getKv());
    }

    public SwerveMotorConfig(
        DCMotor dcMotor, int id, boolean isCCW, boolean isBreakMode, double gearRatio, double momentOfInertia, 
        int currentLimit, PIDController pid, SimpleMotorFeedforward ff, TalonFXConfiguration talonFXConfig){
            this.dcMotor = dcMotor;
            this.id = id;
            this.isCCW = isCCW;
            this.isBreakMode = isBreakMode;
            this.gearRatio = gearRatio;
            this.momentOfInertia = momentOfInertia;
            this.currentLimit = currentLimit;
            this.pid = pid;
            this.ff = ff;
    }
}