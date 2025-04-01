package frc.robot.utils.betterpathplanner;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CustomPathfindingCommand extends PathfindingCommand {
    private static int instances = 0;

    private final Timer timer = new Timer();
    private final PathPlannerPath targetPath;
    private Pose2d targetPose;
    private Pose2d originalTargetPose;
    private GoalEndState goalEndState;
    private final PathConstraints constraints;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final BiConsumer<ChassisSpeeds, DriveFeedforwards> output;
    private final PathFollowingController controller;
    private final RobotConfig robotConfig;
    private final BooleanSupplier shouldFlipPath;

    private PathPlannerPath currentPath;
    private PathPlannerTrajectory currentTrajectory;

    private double timeOffset = 0;
    private boolean isFinished = false;
    
    public CustomPathfindingCommand(
        Pose2d targetPose,
        PathConstraints constraints,
        double goalEndVel,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedsSupplier,
        BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
        PathFollowingController controller,
        RobotConfig robotConfig,
        Subsystem... requirements
    ){
        super(targetPose, constraints, goalEndVel, poseSupplier, speedsSupplier, output, controller, robotConfig, requirements);

        addRequirements(requirements);

        Pathfinding.ensureInitialized();

        this.targetPath = null;
        this.targetPose = targetPose;
        this.originalTargetPose =
            new Pose2d(this.targetPose.getTranslation(), this.targetPose.getRotation());
        this.goalEndState = new GoalEndState(goalEndVel, targetPose.getRotation());
        this.constraints = constraints;
        this.controller = controller;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.output = output;
        this.robotConfig = robotConfig;
        this.shouldFlipPath = () -> false;

        instances++;
    }

    @Override
    public boolean isFinished(){
        
        return isFinished;
    }

    @Override
    public void cancel(){
        super.cancel();
        isFinished = true;
    }

    @Override
    public void initialize(){
        super.initialize();
        currentTrajectory = null;
        timeOffset = 0;
        isFinished = false;

        Pose2d currentPose = poseSupplier.get();

        controller.reset(currentPose, speedsSupplier.get());

        if (targetPath != null) {
            originalTargetPose =
                new Pose2d(this.targetPath.getPoint(0).position, originalTargetPose.getRotation());
            if (shouldFlipPath.getAsBoolean()) {
                targetPose = FlippingUtil.flipFieldPose(this.originalTargetPose);
                goalEndState = new GoalEndState(goalEndState.velocityMPS(), targetPose.getRotation());
            }
        }

        Pathfinding.setStartPosition(currentPose.getTranslation());
        Pathfinding.setGoalPosition(targetPose.getTranslation());
    }

    public void execute(){
        super.execute();
        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentSpeeds = speedsSupplier.get();

        PathPlannerLogging.logCurrentPose(currentPose);
        PPLibTelemetry.setCurrentPose(currentPose);

        // Skip new paths if we are close to the end
        boolean skipUpdates =
            currentTrajectory != null
                && currentPose
                        .getTranslation()
                        .getDistance(currentTrajectory.getEndState().pose.getTranslation())
                    < 2.0;

        if (!skipUpdates && Pathfinding.isNewPathAvailable()) {
        currentPath = Pathfinding.getCurrentPath(constraints, goalEndState);

        if (currentPath != null) {
            currentTrajectory =
                new PathPlannerTrajectory(
                    currentPath, currentSpeeds, currentPose.getRotation(), robotConfig);

            // Find the two closest states in front of and behind robot
            int closestState1Idx = 0;
            int closestState2Idx = 1;
            while (closestState2Idx < currentTrajectory.getStates().size() - 1) {
                double closest2Dist =
                    currentTrajectory
                        .getState(closestState2Idx)
                        .pose
                        .getTranslation()
                        .getDistance(currentPose.getTranslation());
                double nextDist =
                    currentTrajectory
                        .getState(closestState2Idx + 1)
                        .pose
                        .getTranslation()
                        .getDistance(currentPose.getTranslation());
                if (nextDist < closest2Dist) {
                    closestState1Idx++;
                    closestState2Idx++;
                } else {
                    break;
                }
            }

            // Use the closest 2 states to interpolate what the time offset should be
            // This will account for the delay in pathfinding
            var closestState1 = currentTrajectory.getState(closestState1Idx);
            var closestState2 = currentTrajectory.getState(closestState2Idx);

            double d =
                closestState1.pose.getTranslation().getDistance(closestState2.pose.getTranslation());
            double t =
                (currentPose.getTranslation().getDistance(closestState1.pose.getTranslation())) / d;
            t = MathUtil.clamp(t, 0.0, 1.0);

            timeOffset = MathUtil.interpolate(closestState1.timeSeconds, closestState2.timeSeconds, t);

            // If the robot is stationary and at the start of the path, set the time offset to the next
            // loop
            // This can prevent an issue where the robot will remain stationary if new paths come in
            // every loop
            if (timeOffset <= 0.02
                && Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 0.1) {
            timeOffset = 0.02;
            }

            PathPlannerLogging.logActivePath(currentPath);
            PPLibTelemetry.setCurrentPath(currentPath);
        }

        timer.reset();
        timer.start();
        }
    }

    public static Command warmupCommand(){
        return new CustomPathfindingCommand(
            new Pose2d(15.0, 4.0, Rotation2d.k180deg),
            new PathConstraints(4, 3, 4, 4),
            0,
            () -> new Pose2d(1.5, 4, Rotation2d.kZero),
            ChassisSpeeds::new,
            (speeds, feedforwards) -> {},
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            new RobotConfig(
                75,
                6.8,
                new ModuleConfig(
                    0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
                0.55))
        .andThen(Commands.print("[PathPlanner] PathfindingCommand finished warmup"))
        .ignoringDisable(true);
    }
}
