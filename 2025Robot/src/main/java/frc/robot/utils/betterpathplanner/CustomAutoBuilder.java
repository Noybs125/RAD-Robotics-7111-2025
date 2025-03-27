package frc.robot.utils.betterpathplanner;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CustomAutoBuilder extends AutoBuilder {
    private static TriFunction<Pose2d, PathConstraints, Double, Command> pathfindToPoseCommandBuilder;
    private static boolean configured = false;

    public static void configure(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> resetPose,
        Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
        Consumer<ChassisSpeeds> output,
        PathFollowingController controller,
        RobotConfig robotConfig,
        BooleanSupplier shouldFlipPath,
        Subsystem... driveRequirements
    ) {
        AutoBuilder.configure(poseSupplier, resetPose, robotRelativeSpeedsSupplier, output, controller, robotConfig, shouldFlipPath, driveRequirements);
        pathfindToPoseCommandBuilder =
        (pose, constraints, goalEndVel) ->
            new CustomPathfindingCommand(
                pose,
                constraints,
                goalEndVel,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                (speeds, feedforwards) -> output.accept(speeds),
                controller,
                robotConfig,
                driveRequirements);
        CustomAutoBuilder.configured = true;
    }

    public static Command pathfindToPose(
        Pose2d pose, PathConstraints constraints, double goalEndVelocity
    ) {
        if (!isPathfindingConfigured()) {
            throw new AutoBuilderException(
                "Auto builder was used to build a pathfinding command before being configured");
        }
        if(pathfindToPoseCommandBuilder != null)
            return pathfindToPoseCommandBuilder.apply(pose, constraints, goalEndVelocity);
        else return Commands.print("Path failed to generate");
    }

    public static boolean isPathfindingConfigured(){
        return configured;
    }
}
