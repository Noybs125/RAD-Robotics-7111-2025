package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.AutoCycle;

public class Field extends SubsystemBase {

    private SendableChooser<Integer> driverLocation = new SendableChooser<>();
    private SendableChooser<AutoCycle> autoCycleChooser = new SendableChooser<>();

    private SendableChooser<Integer> coralStationChooser = new SendableChooser<>();
    private SendableChooser<Integer> reefFaceChooser = new SendableChooser<>();
    private SendableChooser<Boolean> reefBranchChooser = new SendableChooser<>();
    private SendableChooser<Integer> reefLevelChooser = new SendableChooser<>();

    private Pose2d poseSetpoint = new Pose2d();
    private FieldSetpoint nearestSetpoint = FieldSetpoint.Reef1;
    private Pose2d nearestPose = new Pose2d();

    public GoalEndState endState = new GoalEndState(0, poseSetpoint.getRotation());
    public PathPlannerPath path;

    private Field2d field = new Field2d();
    private FieldObject2d fieldObjectPose = field.getObject("FieldPosition");
    private ComplexWidget fieldPublisher;

    private Swerve swerve;
    private GenericEntry xEntry = Shuffleboard.getTab("test").add("robot x", 0).getEntry();
    private GenericEntry yEntry = Shuffleboard.getTab("test").add("robot y", 0).getEntry();
    private GenericEntry yawEntry = Shuffleboard.getTab("test").add("robot yaw", 0).getEntry();

    private GenericEntry cycleAmountEntry;
    private double autoCycleAmount = 0;

    public List<Pose2d> zoneMap = new ArrayList<>();
    public List<Pose2d> zoneMapFlipped = new ArrayList<>();
    private Pose2d[] zoneArray = new Pose2d[] {
        new Pose2d(3.16, 4.05, Rotation2d.fromDegrees(0)),
        new Pose2d(3.87, 5.20, Rotation2d.fromDegrees(-60)),
        new Pose2d(5.15, 5.18, Rotation2d.fromDegrees(-120)),
        new Pose2d(5.86, 4.00, Rotation2d.fromDegrees(-180)),
        new Pose2d(5.08, 2.85, Rotation2d.fromDegrees(120)),
        new Pose2d(3.16, 4.05, Rotation2d.fromDegrees(60)),
        //new Pose2d(5.18, 2.78, Rotation2d.fromDegrees(119.01)),
        //new Pose2d(3.76, 2.80, Rotation2d.fromDegrees(60.01)),
        //new Pose2d(6.21, 0.39, Rotation2d.fromDegrees(270)),
        //new Pose2d(7.90, 6.10, Rotation2d.fromDegrees(0.00)),
    };
    public Map<FieldSetpoint, Pose2d> fieldSetpointMap = new HashMap<>();
    public Map<FieldSetpoint, Pose2d> fieldSetpointMapFlipped = new HashMap<>();
    public Map<Pose2d, FieldSetpoint> reversedSetpointMap = new HashMap<>();
    private FieldSetpoint[] fieldSetpoints = new FieldSetpoint[] {
        FieldSetpoint.Reef1,
        FieldSetpoint.Reef2,
        FieldSetpoint.Reef3,
        FieldSetpoint.Reef4,
        FieldSetpoint.Reef5,
        FieldSetpoint.Reef6,
        //FieldSetpoint.Processor,
        //FieldSetpoint.SourceLeft,
        //FieldSetpoint.SourceRight,
        //FieldSetpoint.Barge,
        //FieldSetpoint.Climb,
    };

    private List<AutoCycle> autoCycles = new ArrayList<>();

    /**
     * States for different field setpoints, resulting in a path to the setpoint described in the states for this class.
     * States include "Reef1" through "Reef6", "Processor", "SourceLeft" and "SourceRight", "Barge" and "Climb".
     */
    public enum FieldSetpoint {
        Reef1,
        Reef2,
        Reef3,
        Reef4,
        Reef5,
        Reef6,
        Processor,
        SourceLeft,
        SourceRight,
        Barge,
        Climb
    }

    /**
     * Constructor for the field class. Sets up Shuffleboard/SmartDashBoard for locations for the start of the match.
     * Uses "addOption" and "getTab"
     * @see -Link to addOption: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/smartdashboard/SendableChooser.html#addOption(java.lang.String,V.
     * @see -Link to getTab: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/shuffleboard/Shuffleboard.html#getTab(java.lang.String).
     */
    public Field(Swerve swerve) {
        this.swerve = swerve;
        fieldPublisher = Shuffleboard.getTab("Odometry").add("field odometry", field).withWidget("Field");

        Pathfinding.ensureInitialized();

        driverLocation.addOption("1", 1);
        driverLocation.addOption("2", 2);
        driverLocation.addOption("3", 3);

        for(Pose2d zone : zoneArray){
            zoneMap.add(zone);
            zoneMapFlipped.add(FlippingUtil.flipFieldPose(zone));
        }

        int poseIndex = 0;
        for(FieldSetpoint setpoint : fieldSetpoints){
            fieldSetpointMap.put(setpoint, zoneMap.get(poseIndex));
            fieldSetpointMapFlipped.put(setpoint, zoneMapFlipped.get(poseIndex));
            reversedSetpointMap.put(zoneMap.get(poseIndex), setpoint);
            poseIndex++;
        }

        Shuffleboard.getTab("Autonomous").add("DriverStation", driverLocation);

        cycleAmountEntry = Shuffleboard.getTab("Autonomous").add("Apply Cycle Amount", false).getEntry();
        
        Shuffleboard.getTab("Autonomous").add(coralStationChooser);
        Shuffleboard.getTab("Autonomous").add(reefFaceChooser);
        Shuffleboard.getTab("Autonomous").add(reefBranchChooser);
        Shuffleboard.getTab("Autonomous").add(reefLevelChooser);
    }

    /**
     * Creats a pathfind to the specified position using getAlliance to see what side of the field the robot is on,
     * Followed by creating a path to the specified position, avoiding constraints.
     * @see -GetAlliance link: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DriverStation.html#getAlliance().
     * @see -PathfindtoPose link: https://pathplanner.dev/api/java/com/pathplanner/lib/auto/AutoBuilder.html#pathfindToPose(edu.wpi.first.math.geometry.Pose2d,com.pathplanner.lib.path.PathConstraints,double).
     * @param pose -Type "Pose2d", used to give pathfindToPose a position to go toward.
     * @return -A command to go from the robot's position to the "pose" input, adhearing to constraints.
     */
    public Command pathfindToPose(Supplier<Pose2d> pose) {
        
        if(pose != null){
            if(pose.get() != null){
                SmartDashboard.putNumber("supplied posex", pose.get().getX());
                SmartDashboard.putNumber("supplied posey", pose.get().getY());
                Command com;
                if (DriverStation.getAlliance().isPresent()) {
                    com = DriverStation.getAlliance().get() == Alliance.Blue 
                        ? AutoBuilder.pathfindToPose(pose.get(), Constants.kAuto.constraints, 0)
                        : AutoBuilder.pathfindToPoseFlipped(pose.get(), Constants.kAuto.constraints, 0);
                }
                else com = AutoBuilder.pathfindToPose(pose.get(), Constants.kAuto.constraints, 0);
                return com.alongWith(Commands.print("pose isnt null"));
            }
        }
        return AutoBuilder.pathfindToPose(new Pose2d(5, 10, new Rotation2d()), Constants.kAuto.reefConstraints, 0).alongWith(Commands.print("null"));
    }

    public Pose2d transformBy(Pose2d pose, boolean left, boolean right) {
        if (left) {
            var relativePose = pose.relativeTo(new Pose2d(pose.getX(), pose.getY() - Constants.kAuto.relativeBy, pose.getRotation()));
            var poseTransform = new Transform2d(relativePose.getX(), relativePose.getY(), relativePose.getRotation());
            pose.plus(poseTransform);
            }
        else if (right) {
            var relativePose = pose.relativeTo(new Pose2d(pose.getX(), pose.getY() + Constants.kAuto.relativeBy, pose.getRotation()));
            var poseTransform = new Transform2d(relativePose.getX(), relativePose.getY(), relativePose.getRotation());
            pose.plus(poseTransform);
        }

        return pose;
    }

    /**
     * States for setting destination for the robot, using the FieldSetpoint as the specified states.
     * @see -Link to Pose2d object class: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Pose2d.html.
     * @param state -Type FieldSetpoint enum. Options include "Reef1" through "Reef6", SourceLeft and "SourceRight", "Barge" and "Climb".
     * @return -A command to the state's position, being a point on the field. Uses method "pathfindToPose", defined in this class.
     */
    public Pose2d getPoseFromSetpoint(Supplier<FieldSetpoint> state, boolean left, boolean right) {
        System.out.println("berfor state");
        if(state != null){
            if(state.get() != null){
                System.out.println("got pose from setpoint maybe " + state.get().toString());
                switch (state.get()) {
                    case Reef1:
                        poseSetpoint = new Pose2d(3.16, 4.05, Rotation2d.fromDegrees(0));
                        transformBy(poseSetpoint, left, right);
                        break;
                    case Reef2:
                        poseSetpoint = new Pose2d(3.87, 5.20, Rotation2d.fromDegrees(-60));
                        transformBy(poseSetpoint, left, right);
                        break;
                    case Reef3:
                        poseSetpoint = new Pose2d(5.15, 5.18, Rotation2d.fromDegrees(-120));
                        transformBy(poseSetpoint, left, right);
                        break;
                    case Reef4:
                        poseSetpoint = new Pose2d(5.86, 4.00, Rotation2d.fromDegrees(-180));
                        transformBy(poseSetpoint, left, right);
                        break;
                    case Reef5:
                        poseSetpoint = new Pose2d(5.08, 2.85, Rotation2d.fromDegrees(120));
                        transformBy(poseSetpoint, left, right);
                        break;
                    case Reef6:
                        poseSetpoint = new Pose2d(3.16, 4.05, Rotation2d.fromDegrees(60));
                        transformBy(poseSetpoint, left, right);
                        break;
                    case SourceLeft:
                        poseSetpoint = new Pose2d(1.07, 7.15, Rotation2d.fromDegrees(126));
                        break;
                    case SourceRight:
                        poseSetpoint = new Pose2d(1.07, 0.96, Rotation2d.fromDegrees(-125.5));
                        break;
                    case Barge:
                        poseSetpoint = new Pose2d(7.29, 6.18, Rotation2d.fromDegrees(0));
                        break;
                    case Climb:
                        switch (driverLocation.getSelected()) {
                            case 1:
                                poseSetpoint = new Pose2d(7.33, 7.27, Rotation2d.fromDegrees(0));
                                break;
                            case 2:
                                poseSetpoint = new Pose2d(7.33, 6.20, Rotation2d.fromDegrees(0));
                                break;
                            case 3:
                                poseSetpoint = new Pose2d(7.33, 5.09, Rotation2d.fromDegrees(0));
                                break;
                            default:
                                poseSetpoint = new Pose2d(4.49, 6.20, Rotation2d.fromDegrees(0));
                        }
                        break;
                    case Processor:
                        poseSetpoint = new Pose2d(6.13, 0.35, Rotation2d.fromDegrees(-90));
                        break;

                    default:
                        poseSetpoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                        break;
                }
            }
        }
        poseSetpoint = swerve.getPose();
        return poseSetpoint;
    }

    /**
     * Gets the nearest zone to the robot's current position.
     * @param robotPose -The robot's current position.
     * @return -Nearest zone to the robot from "zoneMap".
     * @see -Link to "nearest" method: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Pose2d.html#nearest(java.util.List).
     */
    public Pose2d getNearestZone(Pose2d robotPose){
        return robotPose.nearest(zoneMap);
    }

    /**
     * Generates an autonomous routine given a list of auto cycles
     * @param cycles A list of auto cycles to add to the command
     * @return The autonomous routine as a SequentialCommandGroup
     * @see AutoCycle
     * @see SequentialCommandGroup
     */
    public Command generateAutoRoutine(List<AutoCycle> cycles){
        Command command = new SequentialCommandGroup();
        for(var cycle : cycles){
            command.andThen(generateAutoCycle(cycle));
        }
        return command;
    }
    /**
     * Generates a command given an autonomous cycle
     * @param cycle The autonomous cycle to use
     * @return The autonomous cycle as a command
     */
    public Command generateAutoCycle(AutoCycle cycle){
        Command command = new InstantCommand();
        switch (cycle.feederStation) {
            case 2:
                //command = new SequentialCommandGroup(NamedCommands.getCommand("Stow"), pathfindToSetpoint(() -> FieldSetpoint.SourceLeft, false, false), NamedCommands.getCommand("Intake"));
                break;
            case 1:
                //command = new SequentialCommandGroup(NamedCommands.getCommand("Stow"), pathfindToSetpoint(() -> FieldSetpoint.SourceRight, false, false), NamedCommands.getCommand("Intake"));
                break;
            case 0:    
            default:
                command = new SequentialCommandGroup(NamedCommands.getCommand("Stow"));
                break;
        }
        FieldSetpoint targetPoseSetpoint;
        switch (cycle.reefFace) {
            case 1:
                targetPoseSetpoint = FieldSetpoint.Reef1;
                break;
            case 2:
                targetPoseSetpoint = FieldSetpoint.Reef2;
                break;
            case 3:
                targetPoseSetpoint = FieldSetpoint.Reef3;
                break;
            case 4:
                targetPoseSetpoint = FieldSetpoint.Reef4;
                break;
            case 5:
                targetPoseSetpoint = FieldSetpoint.Reef5;
                break;
            case 6:
                targetPoseSetpoint = FieldSetpoint.Reef6;
                break;
            default:
                throw new IllegalArgumentException("AutoCycle.reefFace must be a value between 1-6");
        }
        Pose2d pose;
        if(cycle.isLeft){
            pose = fieldSetpointMap.get(targetPoseSetpoint);
            var pose2 = pose.relativeTo(pose.plus(new Transform2d(0, pose.getY() + 1, pose.getRotation())));
            Transform2d transform = new Transform2d(pose2.getX(), pose2.getY(), pose2.getRotation());
            pose.transformBy(transform);
        }else{
            pose = fieldSetpointMap.get(targetPoseSetpoint);
            var pose2 = pose.relativeTo(pose.plus(new Transform2d(0, pose.getY() - 1, pose.getRotation())));
            Transform2d transform = new Transform2d(pose2.getX(), pose2.getY(), pose2.getRotation());
            pose.transformBy(transform);
        }
        command.andThen(new SequentialCommandGroup(NamedCommands.getCommand("Stow"), pathfindToPose(() -> pose)));


        switch (cycle.levelToScore) {
            case 1:
                command.andThen(NamedCommands.getCommand("ScoreL1"));
                break;
            case 2:
                command.andThen(NamedCommands.getCommand("ScoreL2"));
                break;
            case 3:
                command.andThen(NamedCommands.getCommand("ScoreL3"));
                break;
            case 4:
                command.andThen(NamedCommands.getCommand("ScoreL4"));
                break;
        
            default:
                throw new IllegalArgumentException("AutoCycle.levelToScore must be a value between 1-4");
        }
        return command;
    }

    public List<AutoCycle> getAutoCycles(){
        List<AutoCycle> rearangedCycles = new ArrayList<>();
        for(var cycle : autoCycles){
            rearangedCycles.set(cycle.executedPosition, cycle);
        }
        return rearangedCycles;
    }
    
    /**
     * Periodic function called 50 times per second
     */
    public void periodic() {
        field.setRobotPose(swerve.getPose());
        SmartDashboard.putNumber("robot x", swerve.getPose().getX());
        SmartDashboard.putNumber("robot y", swerve.getPose().getY());
        SmartDashboard.putNumber("robot yaw", swerve.getPose().getRotation().getDegrees());

        //field.setRobotPose(xEntry.getDouble(0), yEntry.getDouble(0), Rotation2d.fromDegrees(yawEntry.getDouble(0)));
        
        if (Pathfinding.isNewPathAvailable()){
            path = Pathfinding.getCurrentPath(Constants.kAuto.constraints, endState);
        }
    
        if (path != null) {
            fieldObjectPose.setPoses(path.getPathPoses());
        }

        if(autoCycleAmount != cycleAmountEntry.getInteger(0)){
            autoCycleChooser = new SendableChooser<>();
            for (int i = 1; i < cycleAmountEntry.getInteger(i); i++) {
                autoCycleChooser.addOption("Cycle " + i, new AutoCycle(i));
            }
        }
        autoCycleAmount = cycleAmountEntry.getInteger(0);
        
        if (autoCycleChooser != null && autoCycleChooser.getSelected() != null) {
            coralStationChooser = autoCycleChooser.getSelected().getCoralStationChooser();
            reefFaceChooser = autoCycleChooser.getSelected().getReefFaceChooser();
            reefBranchChooser = autoCycleChooser.getSelected().getReefBranchChooser();
            reefLevelChooser = autoCycleChooser.getSelected().getReefLevelChooser();

            autoCycleChooser.getSelected().updateAutoCycle();
        }
        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get() == Alliance.Blue){
                nearestSetpoint = reversedSetpointMap.get(swerve.getPose().nearest(zoneMap));
            }else{
                nearestSetpoint = reversedSetpointMap.get(FlippingUtil.flipFieldPose(swerve.getPose()).nearest(zoneMap));
            }
        }
        nearestPose = fieldSetpointMap.get(nearestSetpoint);
        SmartDashboard.putString("nearest setpoint", nearestSetpoint.toString());
        SmartDashboard.putNumber("nearest posex", nearestPose.getX());
        SmartDashboard.putNumber("nearest posey", nearestPose.getY());
    }

    public Command alignToNearestSetpoint(boolean isLeft, boolean isRight){
        return pathfindToPose(() -> nearestPose).alongWith(Commands.print("why isnt this working"));
        //return pathfindToPose(() -> getPoseFromSetpoint(() -> getNearestSetpoint(() -> swerve.getPose()), isLeft, isRight));
    }

    public FieldSetpoint getNearestSetpoint(Supplier<Pose2d> robotPose){
        System.out.println("This is Running!!!!!!!!!!!!!!!!!!!!!!!");
        Pose2d pose;
        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get() == Alliance.Red){
                pose = robotPose.get().nearest(zoneMapFlipped);
            }else{
                pose = robotPose.get().nearest(zoneMap);
            }
        }else pose = robotPose.get().nearest(zoneMap);
        
        
        for (FieldSetpoint setpoint : fieldSetpoints) {
            System.out.println(setpoint.toString());
            if(pose.getX() == fieldSetpointMap.get(setpoint).getX() 
                && pose.getY() == fieldSetpointMap.get(setpoint).getY()
                && pose.getRotation().getDegrees() == fieldSetpointMap.get(setpoint).getRotation().getDegrees()){
                    return setpoint;
                }
        }
        return null;
    }
}
