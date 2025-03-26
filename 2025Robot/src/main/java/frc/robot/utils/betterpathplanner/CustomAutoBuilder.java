package frc.robot.utils.betterpathplanner;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CustomAutoBuilder extends AutoBuilder {
    private static TriFunction<Pose2d, PathConstraints, Double, Command> pathfindToPoseCommandBuilder;
    private static Supplier<Pose2d> poseSupplier;
    private static Consumer<Pose2d> resetPose;
    private static Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier;
    private static BiConsumer<ChassisSpeeds, DriveFeedforwards> output;
    private static PathFollowingController controller;
    private static RobotConfig robotConfig;
    private static BooleanSupplier shouldFlipPath;
    private static boolean configured;
    private static boolean isHolonomic;

    public static void configure(
        Supplier<Pose2d> poseSupplier,
        Consumer<Pose2d> resetPose,
        Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
        BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
        PathFollowingController controller,
        RobotConfig robotConfig,
        BooleanSupplier shouldFlipPath,
        Subsystem... driveRequirements) 
    {
        AutoBuilder.configure(poseSupplier, resetPose, robotRelativeSpeedsSupplier, output, controller, robotConfig, shouldFlipPath, driveRequirements);
        CustomAutoBuilder.poseSupplier = poseSupplier;
        CustomAutoBuilder.resetPose = resetPose;
        CustomAutoBuilder.configured = true;
        CustomAutoBuilder.shouldFlipPath = shouldFlipPath;
        CustomAutoBuilder.isHolonomic = robotConfig.isHolonomic;
        pathfindToPoseCommandBuilder =
        (pose, constraints, goalEndVel) ->
            new CustomPathfindingCommand(
                pose,
                constraints,
                goalEndVel,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                output,
                controller,
                robotConfig,
                driveRequirements);
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
}
