package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Deepclimb;
import frc.robot.subsystems.Field;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Field.FieldSetpoint;
import frc.robot.subsystems.Mechanisms.MechanismsState;
import frc.robot.subsystems.SuperStructure.ControlState;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  public final CommandXboxController driverController;

  public final Swerve swerve;
  public final Field field;
  public final Vision vision;
  public final Mechanisms mechanisms;
  public final Flywheels flywheels;
  public final SuperStructure superStructure;
  public final Sensors sensors;
  //public final Deepclimb deepClimb;

  public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    xbox = new XboxController(0);
    driverController = new CommandXboxController(0);
    
    sensors = new Sensors();
    vision = new Vision(gyro);
    swerve = new Swerve(gyro, vision);
    field = new Field(swerve);
    mechanisms = new Mechanisms();
    flywheels = new Flywheels();
    //deepClimb = new Deepclimb();
    superStructure = new SuperStructure(swerve, vision, field, sensors, mechanisms, flywheels, null);
  

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Custom", new PathPlannerAuto(field.generateAutoRoutine(field.getAutoCycles())));
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
    // all of these bindings will be correctly defined when we decide controls
    Trigger leftReefAlign = driverController.leftTrigger();
    Trigger rightReefAlign = driverController.rightTrigger();
    Trigger elevatorUp = driverController.povUp();
    Trigger elevatorDown = driverController.povDown();
    Trigger armUp = driverController.povRight();
    Trigger armDown = driverController.povLeft();
    Trigger effectorIntake = driverController.rightBumper();
    Trigger effectorScore = driverController.leftBumper();

    Trigger zeroGyro = driverController.start();
    Trigger resetOdometry = driverController.a();

    Trigger l1 = driverController.x();
    Trigger l2 = driverController.a();
    Trigger l3 = driverController.b();
    Trigger l4 = driverController.y();

    swerve.setDefaultCommand(swerve.drive(
      () -> -swerve.getTransY(),
      () -> swerve.getTransX(),
      () -> swerve.getRotationZ(),
      () -> swerve.getFieldRelative(), 
      false
      )
    );

    leftReefAlign.onTrue(superStructure.setSwerveState(SwerveState.leftAlignment));
    rightReefAlign.onTrue(superStructure.setSwerveState(SwerveState.rightAlignment));

    leftReefAlign.and(rightReefAlign).onTrue(superStructure.setSwerveState(SwerveState.centerAlignment));

    leftReefAlign.negate().and(rightReefAlign.negate()).onTrue(superStructure.setSwerveState(SwerveState.DefaultState));

    elevatorUp.onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(0.1))
        .alongWith(new InstantCommand(() -> mechanisms.setState(MechanismsState.Manual))));

    elevatorDown.onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(-0.1))
        .alongWith(new InstantCommand(() -> mechanisms.setState(MechanismsState.Manual))));

    elevatorDown.negate().and(elevatorUp.negate()).onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(0)));

    armUp.onTrue(new InstantCommand(() -> mechanisms.setWristSpeed(0.1))
        .alongWith(new InstantCommand(() -> mechanisms.setState(MechanismsState.Manual))));

    armDown.onTrue(new InstantCommand(() -> mechanisms.setWristSpeed(-0.1))
        .alongWith(new InstantCommand(() -> mechanisms.setState(MechanismsState.Manual))));

    armUp.negate().and(armDown.negate()).onTrue(new InstantCommand(() -> mechanisms.setWristSpeed(0)));

    effectorIntake.onTrue(new InstantCommand(() -> flywheels.setSpeed(1)));
    effectorScore.onTrue(new InstantCommand(() -> flywheels.setSpeed(-1)));
    effectorIntake.negate().and(effectorScore.negate()).onTrue(new InstantCommand(() -> flywheels.setSpeed(0)));


    

    l1.onTrue(superStructure.setRobotStateCommand(ControlState.XButton));
    l2.onTrue(superStructure.setRobotStateCommand(ControlState.AButton));
    l3.onTrue(superStructure.setRobotStateCommand(ControlState.BButton));
    l4.onTrue(superStructure.setRobotStateCommand(ControlState.YButton));

    // change or remove each of these when we decide controls
    zeroGyro.onTrue(swerve.zeroGyroCommand());
    resetOdometry.onTrue(swerve.resetOdometryCommand());
  }
}
