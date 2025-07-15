package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverControllerID);
    public final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.driverControllerID);
    public PathPlannerAuto auto1;
    public final SwerveSubsystem swerve;
    public SendableChooser<Command> autoChooser;

    public RobotContainer() {
        swerve = new SwerveSubsystem();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("autoChooser", autoChooser);

        NamedCommands.registerCommand("test command", auto1);

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
            () -> -ControllerConstants.xDriveLimiter.calculate((Math.pow(driverController.getLeftY(), 3) / SwerveConstants.sensitivity)), 
            () -> -ControllerConstants.yDriveLimiter.calculate((Math.pow(driverController.getLeftX(), 3) / SwerveConstants.sensitivity)),  
            () -> ControllerConstants.rotationLimiter.calculate((Math.pow(driverController.getRightX(), 3) / SwerveConstants.sensitivity)),
            true,
            false
        ));

        driverController.start().onTrue(swerve.zeroGyroCommand());
    }
}
