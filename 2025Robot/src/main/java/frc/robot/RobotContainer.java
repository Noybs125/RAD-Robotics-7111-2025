package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Mechanisms;
import frc.robot.utils.encoder.RevEncoder;
import frc.robot.utils.motor.CTREMotor;
import frc.robot.utils.motor.ElevatorSimMotor;
import frc.robot.utils.motor.REVMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
  public final Vision vision;
  public final Mechanisms mechanisms;
  public final Flywheels flywheels;

  public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final SendableChooser<Command> autoChooser;

  

  public RobotContainer() {
    xbox = new XboxController(2);
    commXbox = new CommandXboxController(2);

    vision = new Vision(gyro);
    swerve = new Swerve(gyro, vision);
    mechanisms = new Mechanisms(
        new ElevatorSimMotor(
            new RevEncoder(0), Constants.kSimulation.elevatorSimGearRatio, 
            Constants.kSimulation.pid, Constants.kSimulation.ff, Constants.kSimulation.elevatorSimConstants
            ), 
        new CTREMotor(0));
    flywheels = new Flywheels(new REVMotor(0));
    
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
    swerve.setDefaultCommand(swerve.drive(
      () -> Constants.kControls.Y_DRIVE_LIMITER.calculate(0.25), 
      () -> Constants.kControls.X_DRIVE_LIMITER.calculate(-xbox.getLeftX()),  
      () -> Constants.kControls.THETA_DRIVE_LIMITER.calculate(-xbox.getRightX()),
      () -> true, 
      false
      ));

    commXbox.y().onTrue(swerve.zeroGyroCommand());
    commXbox.a().onTrue(swerve.resetOdometryCommand());
    }
  }
