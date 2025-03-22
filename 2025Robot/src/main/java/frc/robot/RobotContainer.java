package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Deepclimb;
import frc.robot.subsystems.Field;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SuperStructure.ActualState;
import frc.robot.subsystems.SuperStructure.ControlState;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Mechanisms;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.SuperStructure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public final CommandXboxController operatorController;
  public final CommandXboxController driverController;

  public final Swerve swerve;
  public final Field field;
  public final Vision vision;
  public final Mechanisms mechanisms;
  public final Flywheels flywheels;
  public final SuperStructure superStructure;
  public final Sensors sensors;
  public final Deepclimb deepClimb;

  public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    operatorController = new CommandXboxController(1);
    driverController = new CommandXboxController(0);
    

    sensors = new Sensors();
    vision = new Vision(gyro);
    swerve = new Swerve(gyro, vision);
    field = new Field(swerve);
    mechanisms = new Mechanisms();
    flywheels = new Flywheels();
    deepClimb = new Deepclimb();
    superStructure = new SuperStructure(swerve, vision, field, sensors, mechanisms, flywheels, deepClimb);

    
    NamedCommands.registerCommand("Coral Feeder", superStructure.setActualStateCommand(SuperStructure.ActualState.coralFeeder));
    NamedCommands.registerCommand("Stow", superStructure.setActualStateCommand(SuperStructure.ActualState.coralL1Stow));
    NamedCommands.registerCommand("Zero", superStructure.setActualStateCommand(SuperStructure.ActualState.stow));
    NamedCommands.registerCommand("L4", superStructure.setActualStateCommand(SuperStructure.ActualState.coralL4));
    /*NamedCommands.registerCommand("L1 Center", field.alignToNearestSetpoint(false, false).alongWith(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL1Stow)));
    NamedCommands.registerCommand("L2 Left", field.alignToNearestSetpoint(true, false).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL2)));
    NamedCommands.registerCommand("L2 Right", field.alignToNearestSetpoint(false, true).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL2)));
    NamedCommands.registerCommand("L3 Left", field.alignToNearestSetpoint(true, false).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL3)));
    NamedCommands.registerCommand("L3 Right", field.alignToNearestSetpoint(false, true).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL3)));
    NamedCommands.registerCommand("L4 Left", field.alignToNearestSetpoint(true, false).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL4)));
    NamedCommands.registerCommand("L4 Right", field.alignToNearestSetpoint(false, true).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL4)));
*/
    NamedCommands.registerCommand("Score", new InstantCommand(() -> flywheels.setSpeed(-0.7)));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> flywheels.setSpeed(1)));
    NamedCommands.registerCommand("Stop Wheels", new InstantCommand(() -> flywheels.setSpeed(0)));


    NamedCommands.registerCommand("L2 Algae", superStructure.useCenterAlignment().andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.algaeL2)));
    NamedCommands.registerCommand("L3 Algae", superStructure.useCenterAlignment().andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.algaeL3)));
    NamedCommands.registerCommand("Processor Algae", superStructure.setActualStateCommand(SuperStructure.ActualState.algaeProcessor));
    NamedCommands.registerCommand("Net Algae", superStructure.setActualStateCommand(SuperStructure.ActualState.algaeNet));

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
    Trigger centerReefAlign = leftReefAlign.and(rightReefAlign);
    Trigger elevatorUp = operatorController.povUp();
    Trigger elevatorDown = operatorController.povDown();
    Trigger armUp = new Trigger(() -> operatorController.getLeftX() > 0.15);//operatorController.povRight();
    Trigger armDown = new Trigger(() -> operatorController.getLeftX() < -0.15);//operatorController.povLeft();
    Trigger effectorIntake = operatorController.rightTrigger(0.1);
    Trigger effectorScore = operatorController.leftTrigger(0.1);
    Trigger climbUp = driverController.rightBumper();
    Trigger climbDown = driverController.leftBumper();
    Trigger algaeL2 = operatorController.rightBumper();
    Trigger algaeL3 = operatorController.leftBumper();
    Trigger algaeNet = operatorController.start();

    Trigger zeroGyro = driverController.start();

    Trigger l1 = operatorController.x();
    Trigger l2 = operatorController.a();
    Trigger l3 = operatorController.b();
    Trigger l4 = operatorController.y();
    Trigger stow = operatorController.back();

    Trigger zeroMechanisms = operatorController.rightStick();

    swerve.setDefaultCommand(swerve.drive(
      () -> -swerve.getTransY(),
      () -> swerve.getTransX(),
      () -> swerve.getRotationZ(),
      () -> swerve.getFieldRelative(), 
      false
      )
    );

    elevatorUp.onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(0.1))
        .alongWith(new InstantCommand(() -> superStructure.setActualState(ActualState.Manual))));

    elevatorDown.onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(-0.1))
        .alongWith(new InstantCommand(() -> superStructure.setActualState(ActualState.Manual))));

    elevatorDown.negate().and(elevatorUp.negate()).onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(0)));

    armUp.onTrue(mechanisms.setSuppliedWristSpeed(() -> operatorController.getLeftX() / 2)
        .alongWith(new InstantCommand(() -> superStructure.setActualState(ActualState.Manual))));

    armDown.onTrue(mechanisms.setSuppliedWristSpeed(() -> operatorController.getLeftX() / 2)
        .alongWith(new InstantCommand(() -> superStructure.setActualState(ActualState.Manual))));

    armUp.negate().and(armDown.negate()).onTrue(mechanisms.setSuppliedWristSpeed(() -> 0));

    zeroMechanisms.onTrue(new InstantCommand(() -> mechanisms.zeroMechanisms()));

    effectorIntake.onTrue(flywheels.setSuppliedSpeed(() -> operatorController.getRightTriggerAxis()));
    effectorScore.onTrue(flywheels.setSuppliedSpeed(() -> -operatorController.getLeftTriggerAxis()));
    effectorIntake.negate().and(effectorScore.negate()).onTrue(new InstantCommand(() -> flywheels.setSpeed(0)));

    //climbUp.onTrue(new InstantCommand(() -> deepClimb.setSpeed(1)));
    //climbDown.onTrue(new InstantCommand(() -> deepClimb.setSpeed(-1)));
    //climbDown.negate().and(climbUp.negate()).onTrue(new InstantCommand(() -> deepClimb.setSpeed(0)));

    algaeL2.onTrue(superStructure.setRobotStateCommand(SuperStructure.ControlState.RightBumper));
    algaeL3.onTrue(superStructure.setRobotStateCommand(SuperStructure.ControlState.LeftBumper));
    algaeNet.onTrue(superStructure.setRobotStateCommand(SuperStructure.ControlState.StartButton));
    

    l1.onTrue(superStructure.setRobotStateCommand(ControlState.XButton));
    l2.onTrue(superStructure.setRobotStateCommand(ControlState.AButton));
    l3.onTrue(superStructure.setRobotStateCommand(ControlState.BButton));
    l4.onTrue(superStructure.setRobotStateCommand(ControlState.YButton));
    stow.onTrue(superStructure.setRobotStateCommand(ControlState.SelectButton));

    rightReefAlign.onTrue(new InstantCommand(() -> field.updateCommand(true, false, true)).alongWith(Commands.print("right")));
    leftReefAlign.onTrue(new InstantCommand(() -> field.updateCommand(true, true, false)).alongWith(Commands.print("left")));
    centerReefAlign.onTrue(new InstantCommand(() -> field.updateCommand(true, false, false)).alongWith(Commands.print("center")));
    rightReefAlign.negate().and(leftReefAlign.negate()).and(centerReefAlign.negate()).onTrue(new InstantCommand(() -> field.updateCommand(false, false, false)).alongWith(Commands.print("none")));

    // change or remove each of these when we decide controls
    zeroGyro.onTrue(swerve.zeroGyroCommand());
  }
}
