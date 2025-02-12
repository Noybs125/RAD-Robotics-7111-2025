package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Swerve.SwerveState;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Mechanisms;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.SuperStructure;
import frc.robot.utils.encoder.RevEncoder;
import frc.robot.utils.motor.ArmSimMotor;
import frc.robot.utils.motor.CTREMotor;
import frc.robot.utils.motor.ElevatorSimMotor;
import frc.robot.utils.motor.REVMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final XboxController xbox;
  public final CommandXboxController commXbox;

  public final Swerve swerve;
  public final Auto auto;
  public final Vision vision;
  public final Mechanisms mechanisms;
  public final Flywheels flywheels;
  public final SuperStructure superStructure;
  public final Sensors sensors;

  public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    xbox = new XboxController(2);
    commXbox = new CommandXboxController(2);

    
    auto = new Auto();
    sensors = new Sensors();
    vision = new Vision(gyro);
    swerve = new Swerve(gyro, vision);
    mechanisms = new Mechanisms(
        new ElevatorSimMotor(
            null, Constants.kSimulation.elevatorSimGearRatio, Constants.kSimulation.elevatorPid,
            Constants.kSimulation.elevatorFF, Constants.kSimulation.elevatorSimConstants
            ), 
         new ArmSimMotor(null, Constants.kSimulation.armSim, Constants.kSimulation.wristPid,  null));//Constants.kSimulation.wristFF));
    flywheels = new Flywheels(new REVMotor(0));
    
    superStructure = new SuperStructure(swerve, vision, sensors, mechanisms, flywheels);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
    

    // Configure button bindings
    configureButtonBindings();
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Trigger simSetpoint1 = commXbox.rightBumper();
    Trigger simSetpoint2 = commXbox.leftBumper();
    Trigger simSetpoint3 = commXbox.leftTrigger();

    swerve.setDefaultCommand(swerve.drive(
      () -> -swerve.getTransY(), 
      () -> swerve.getTransX(),  
      () -> swerve.getRotationZ(),
      () -> swerve.getFieldRelative(), 
      false
      )
     );

    commXbox.y().onTrue(swerve.zeroGyroCommand());
    commXbox.a().onTrue(swerve.resetOdometryCommand());
    commXbox.b().onTrue(auto.pathfindToSetpoint(Auto.FieldSetpoints.Reef6));
    commXbox.x().onTrue(superStructure.setRobotStateCommand(SuperStructure.ControlState.ReefL1Processor)).onFalse(superStructure.setRobotStateCommand(SuperStructure.ControlState.Default));
    simSetpoint1.onTrue(superStructure.setRobotStateCommand(SuperStructure.ControlState.ReefL1Processor));
    simSetpoint2.onTrue(superStructure.setRobotStateCommand(SuperStructure.ControlState.ReefL2));
    simSetpoint3.onTrue(superStructure.setRobotStateCommand(SuperStructure.ControlState.ReefL3));
    }
  }
