package frc.robot.subsystems.swerve.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.DeviceConfigs;
import frc.robot.DeviceConfigs.SwerveModuleConfigs;
import frc.robot.subsystems.swerve.modules.SimSwerveModule;
import frc.robot.subsystems.swerve.modules.GenericSwerveModule;
import frc.robot.subsystems.swerve.modules.TalonFXSwerveModule;
import frc.robot.utils.encoder.CTREEncoder;

public class DrivebaseConfig {
    
    public GenericSwerveModule[] moduleTypes;
    public SwerveModuleConfig[] moduleConstants;
    public double width;
    public double length;
    public double wheelDiameter;

    public DrivebaseConfig(GenericSwerveModule[] moduleTypes, SwerveModuleConfig[] moduleConstants, double width, double length, double wheelDiameter){
        this.moduleTypes = moduleTypes;
        this.moduleConstants = moduleConstants;
        this.width = width;
        this.length = length;
        this.wheelDiameter = wheelDiameter;
    }

    public static DrivebaseConfig getStormSurge(boolean isSim){
        double width = Units.inchesToMeters(21.25);
        double length = Units.inchesToMeters(23.25);
        double wheelDiameter = Units.inchesToMeters(3.75);

        double driveGearing = 6.72 / 1.0; 
        double angleGearing = 468.0 / 35.0;
        double driveMOI = 0.25;
        double angleMOI = 0.001;
        int driveCurrentLimit = 80;
        int angleCurrentLimit = 40;
        boolean driveInversion = false;
        boolean angleInversion = true;
        boolean driveBreakMode = true;
        boolean angleBreakMode = false;
        PIDController drivePID = new PIDController(50, 0.0, 0.0);
        PIDController anglePID = new PIDController(50, 0.0, 0.0);
        SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.001, 0.0);
        SimpleMotorFeedforward angleFF = new SimpleMotorFeedforward(0.001, 0.0);
        TalonFXConfiguration driveConfig = SwerveModuleConfigs.getTalonFXDrive();
        TalonFXConfiguration angleConfig = SwerveModuleConfigs.getTalonFXRotation();

        SwerveModuleConfig[] moduleConstants = new SwerveModuleConfig[]{
            new SwerveModuleConfig(
                new SwerveMotorConfig(DCMotor.getKrakenX60(1), 1, driveInversion, driveBreakMode, driveGearing, driveMOI, driveCurrentLimit, drivePID, driveFF, driveConfig), 
                new SwerveMotorConfig(DCMotor.getKrakenX60(1), 2, angleInversion, angleBreakMode, angleGearing, angleMOI, angleCurrentLimit, anglePID, angleFF, angleConfig), 
                new CTREEncoder(0, SwerveModuleConfigs.getCANCoder()), 0),

            new SwerveModuleConfig(
                new SwerveMotorConfig(DCMotor.getKrakenX60(1), 3, driveInversion, driveBreakMode, driveGearing, driveMOI, driveCurrentLimit, drivePID, driveFF, driveConfig), 
                new SwerveMotorConfig(DCMotor.getKrakenX60(1), 4, angleInversion, angleBreakMode, angleGearing, angleMOI, angleCurrentLimit, anglePID, angleFF, angleConfig), 
                new CTREEncoder(1, SwerveModuleConfigs.getCANCoder()), 0),

            new SwerveModuleConfig(
                new SwerveMotorConfig(DCMotor.getKrakenX60(1), 5, driveInversion, driveBreakMode, driveGearing, driveMOI, driveCurrentLimit, drivePID, driveFF, driveConfig), 
                new SwerveMotorConfig(DCMotor.getKrakenX60(1), 6, angleInversion, angleBreakMode, angleGearing, angleMOI, angleCurrentLimit, anglePID, angleFF, angleConfig), 
                new CTREEncoder(2, SwerveModuleConfigs.getCANCoder()), 0),

            new SwerveModuleConfig(
                new SwerveMotorConfig(DCMotor.getKrakenX60(1), 7, driveInversion, driveBreakMode, driveGearing, driveMOI, driveCurrentLimit, drivePID, driveFF, driveConfig), 
                new SwerveMotorConfig(DCMotor.getKrakenX60(1), 8, angleInversion, angleBreakMode, angleGearing, angleMOI, angleCurrentLimit, anglePID, angleFF, angleConfig), 
                new CTREEncoder(3, SwerveModuleConfigs.getCANCoder()), 0),
        };

        GenericSwerveModule[] moduleTypes;
        if(isSim){
            moduleTypes = new GenericSwerveModule[]{
                new SimSwerveModule(moduleConstants[0]),
                new SimSwerveModule(moduleConstants[1]),
                new SimSwerveModule(moduleConstants[2]),
                new SimSwerveModule(moduleConstants[3]),
            };
        }else{
            moduleTypes = new GenericSwerveModule[]{
                new TalonFXSwerveModule(moduleConstants[0]),
                new TalonFXSwerveModule(moduleConstants[1]),
                new TalonFXSwerveModule(moduleConstants[2]),
                new TalonFXSwerveModule(moduleConstants[3]),
            };
        }
        
        return new DrivebaseConfig(moduleTypes, moduleConstants, width, length, wheelDiameter);
    }

    public static DrivebaseConfig getBoxChassis(){
        GenericSwerveModule[] moduleTypes = new GenericSwerveModule[]{
            
        };
        SwerveModuleConfig[] moduleConstants = new SwerveModuleConfig[]{

        };
        double width = 0;
        double length = 0;
        double wheelDiameter = 4;
        double moi = 0.001;

        return new DrivebaseConfig(moduleTypes, moduleConstants, width, length, wheelDiameter);
    }
}
