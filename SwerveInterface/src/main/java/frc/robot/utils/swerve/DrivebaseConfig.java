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
    public double driveGearing; 
    public double angleGearing;
    public boolean driveInversion;
    public boolean angleInversion;
    public PIDController drivePID;
    public PIDController anglePID;

    public DrivebaseConfig(SwerveModuleType[] moduleTypes, SwerveModuleConstants[] moduleConstants, double width, double length, double wheelDiameter, double driveGearing, double angleGearing, boolean driveInversion, boolean angleInversion, PIDController drivePID, PIDController anglePID){
        this.moduleTypes = moduleTypes;
        this.moduleConstants = moduleConstants;
        this.width = width;
        this.length = length;
        this.wheelDiameter = wheelDiameter;
        this.driveGearing = driveGearing;
        this.angleGearing = angleGearing;
        this.driveInversion = driveInversion;
        this.angleInversion = angleInversion;
        this.drivePID = drivePID;
        this.anglePID = anglePID;
    }

    public static DrivebaseConfig getStormSurge(){
        SwerveModuleConstants[] moduleConstants = new SwerveModuleConstants[]{
            new SwerveModuleConstants(1, 2, DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), new CTREEncoder(0, SwerveModuleConfigs.getCANCoder()), 0),
            new SwerveModuleConstants(1, 2, DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), new CTREEncoder(1, SwerveModuleConfigs.getCANCoder()), 0),
            new SwerveModuleConstants(1, 2, DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), new CTREEncoder(2, SwerveModuleConfigs.getCANCoder()), 0),
            new SwerveModuleConstants(1, 2, DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), new CTREEncoder(3, SwerveModuleConfigs.getCANCoder()), 0),
        };
        SwerveModuleType[] moduleTypes = new SwerveModuleType[]{
            new TalonFXSwerveModule(moduleConstants[0]),
            new TalonFXSwerveModule(moduleConstants[1]),
            new TalonFXSwerveModule(moduleConstants[2]),
            new TalonFXSwerveModule(moduleConstants[3]),
        };
        double width = 0;
        double length = 0;
        double wheelDiameter = 0;
        double driveGearing = 6.72 / 1.0; 
        double angleGearing = 468.0 / 35.0;
        boolean driveInversion = false;
        boolean angleInversion = true;
        PIDController drivePID = new PIDController(0.1, 0.0, 0.0);
        PIDController anglePID = new PIDController(0.1, 0.0, 0.0);
        
        return new DrivebaseConfig(moduleTypes, moduleConstants, width, length, wheelDiameter, driveGearing, angleGearing, driveInversion, angleInversion, drivePID, anglePID);
    }

    public static DrivebaseConfig getBoxChassis(){
        SwerveModuleType[] moduleTypes = new SwerveModuleType[]{
            
        };
        SwerveModuleConstants[] moduleConstants = new SwerveModuleConstants[]{

        };
        double width = 0;
        double length = 0;
        double wheelDiameter = 4;
        double driveGearing = 6.75 / 1.0; 
        double angleGearing = 150.0 / 7.0;
        boolean driveInversion = false;
        boolean angleInversion = true;
        PIDController drivePID = new PIDController(0.1, 0.0, 0.0);
        PIDController anglePID = new PIDController(0.1, 0.0, 0.0);
        return new DrivebaseConfig(moduleTypes, moduleConstants, width, length, wheelDiameter, driveGearing, angleGearing, driveInversion, angleInversion, drivePID, anglePID);
    }
}
