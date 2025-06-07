package frc.robot.utils.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.DeviceConfigs.SwerveModuleConfigs;
import frc.robot.utils.encoder.CTREEncoder;
import frc.robot.utils.swerve.modules.SwerveModuleType;
import frc.robot.utils.swerve.modules.TalonFXSwerveModule;

public class DrivebaseConfig {
    
    public SwerveModuleType[] moduleTypes;
    public SwerveModuleConstants[] moduleConstants;
    public double width;
    public double length;
    public double wheelDiameter;

    public DrivebaseConfig(SwerveModuleType[] moduleTypes, SwerveModuleConstants[] moduleConstants, double width, double length, double wheelDiameter){
        this.moduleTypes = moduleTypes;
        this.moduleConstants = moduleConstants;
        this.width = width;
        this.length = length;
        this.wheelDiameter = wheelDiameter;
    }

    public static DrivebaseConfig getStormSurge(){
        double width = 0;
        double length = 0;
        double wheelDiameter = 3.75;

        double driveGearing = 6.72 / 1.0; 
        double angleGearing = 468.0 / 35.0;
        double driveCurrentLimit = 40;
        double angleCurrentLimit = 40;
        boolean driveInversion = false;
        boolean angleInversion = true;
        PIDController drivePID = new PIDController(0.1, 0.0, 0.0);
        PIDController anglePID = new PIDController(0.1, 0.0, 0.0);

        SwerveModuleConstants[] moduleConstants = new SwerveModuleConstants[]{
            new SwerveModuleConstants(
                new MotorConstants(DCMotor.getKrakenX60(1), 1, driveInversion, driveGearing, driveCurrentLimit, drivePID), 
                new MotorConstants(DCMotor.getKrakenX60(1), 2, angleInversion, angleGearing, angleCurrentLimit, anglePID), 
                new CTREEncoder(0, SwerveModuleConfigs.getCANCoder()), 0),

            new SwerveModuleConstants(
                new MotorConstants(DCMotor.getKrakenX60(1), 3, driveInversion, driveGearing, driveCurrentLimit, drivePID), 
                new MotorConstants(DCMotor.getKrakenX60(1), 4, angleInversion, angleGearing, angleCurrentLimit, anglePID), 
                new CTREEncoder(1, SwerveModuleConfigs.getCANCoder()), 0),

            new SwerveModuleConstants(
                new MotorConstants(DCMotor.getKrakenX60(1), 5, driveInversion, driveGearing, driveCurrentLimit, drivePID), 
                new MotorConstants(DCMotor.getKrakenX60(1), 6, angleInversion, angleGearing, angleCurrentLimit, anglePID), 
                new CTREEncoder(2, SwerveModuleConfigs.getCANCoder()), 0),

            new SwerveModuleConstants(
                new MotorConstants(DCMotor.getKrakenX60(1), 7, driveInversion, driveGearing, driveCurrentLimit, drivePID), 
                new MotorConstants(DCMotor.getKrakenX60(1), 8, angleInversion, angleGearing, angleCurrentLimit, anglePID), 
                new CTREEncoder(3, SwerveModuleConfigs.getCANCoder()), 0),
        };
        SwerveModuleType[] moduleTypes = new SwerveModuleType[]{
            new TalonFXSwerveModule(moduleConstants[0]),
            new TalonFXSwerveModule(moduleConstants[1]),
            new TalonFXSwerveModule(moduleConstants[2]),
            new TalonFXSwerveModule(moduleConstants[3]),
        };
        
        return new DrivebaseConfig(moduleTypes, moduleConstants, width, length, wheelDiameter);
    }

    public static DrivebaseConfig getBoxChassis(){
        SwerveModuleType[] moduleTypes = new SwerveModuleType[]{
            
        };
        SwerveModuleConstants[] moduleConstants = new SwerveModuleConstants[]{

        };
        double width = 0;
        double length = 0;
        double wheelDiameter = 4;

        return new DrivebaseConfig(moduleTypes, moduleConstants, width, length, wheelDiameter);
    }
}
