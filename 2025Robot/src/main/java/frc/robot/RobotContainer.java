package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Mechanisms;
import frc.robot.utils.encoder.Encoder;
import frc.robot.utils.encoder.WpiEncoder;
import frc.robot.utils.motor.CTREMotor;
import frc.robot.utils.motor.ElevatorSimMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final Joystick leftJoy;
  public final Joystick rightJoy;

  public final XboxController xbox;
  public final CommandXboxController commXbox;
 

  public final Swerve swerve;
  public final Mechanisms mech = new Mechanisms(
    new ElevatorSimMotor(new WpiEncoder(0, 1), Constants.kSimulation.elevatorSimGearRatio, Constants.kSimulation.pid, Constants.kSimulation.ff, Constants.kSimulation.elevatorSimConstants), 
    new ElevatorSimMotor(new WpiEncoder(2, 3), Constants.kSimulation.elevatorSimGearRatio, Constants.kSimulation.pid, Constants.kSimulation.ff, Constants.kSimulation.elevatorSimConstants),
    new CTREMotor(0));

  public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  public final Vision vision;

  private final SendableChooser<Command> autoChooser;

  

  public RobotContainer() {
    leftJoy = new Joystick(Constants.kControls.LEFT_JOY_ID);
    rightJoy = new Joystick(Constants.kControls.RIGHT_JOY_ID);

    xbox = new XboxController(2);
    commXbox = new CommandXboxController(2);

    vision = new Vision(gyro);
    
    swerve = new Swerve(gyro, vision);

    

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData(autoChooser);
    

    // Configure button bindings
    configureButtonBindings();
  }
  
  public Command getAutonomousCommand() {

    return new PathPlannerAuto("New Auto");
    
 
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

      //elevatorMech2d.setLength(kElevatorMinimumLength + encoder.getPositionAsDouble());

    commXbox.y().onTrue(swerve.zeroGyroCommand());
    commXbox.a().onTrue(swerve.resetOdometryCommand());
    //commXbox.a().onTrue(new InstantCommand(() -> ElevatorSimMotor.mech2d.setElevatorSetpoint(10))).onFalse(new InstantCommand(() -> mech.setElevatorSetpoint(0)));
    }
  }
