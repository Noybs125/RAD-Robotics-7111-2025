package frc.robot.utils;

import java.io.IOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.studica.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import java.util.Optional;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.Matrix;

public class Camera extends PhotonCamera{
    private Transform3d cameraToRobotCenter;
    private Vision vision;
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonPipelineResult latestResult;
    private PhotonTrackedTarget bestTarget;
    private Transform3d bestCameraToTarget;
    public EstimatedRobotPose estRobotPose;
    public int bestTargetId;
    private AprilTagFieldLayout apriltagMap;
    private Pose2d newPose = new Pose2d();
    {
        try {
            apriltagMap = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        } catch (IOException e){
            throw new RuntimeException(e);
        }
    }

    private AHRS gyro;

    public Camera(PhotonCamera camera, Transform3d cameraToRobotCenter, EstimatedRobotPose estRobotPose, Vision vision) {
        super(camera.getName());
        this.cameraToRobotCenter = cameraToRobotCenter;
        this.vision = vision;
        this.estRobotPose = estRobotPose;
        photonPoseEstimator = new PhotonPoseEstimator(apriltagMap, PoseStrategy.AVERAGE_BEST_TARGETS, cameraToRobotCenter);
    }
    
    public void periodic(){
        latestResult = super.getLatestResult();
        
        if(latestResult.hasTargets()){
            bestTarget = latestResult.getBestTarget();
            bestCameraToTarget = bestTarget.getBestCameraToTarget();
            bestTargetId = bestTarget.getFiducialId();
        }
        Optional<EstimatedRobotPose> estPose = getEstimatedGlobalPose(vision.robotPose);
        if(estPose.isPresent()){
            if(photonPoseEstimator.getRobotToCameraTransform() != cameraToRobotCenter){
                photonPoseEstimator.setRobotToCameraTransform(cameraToRobotCenter);
            }
            estRobotPose = estPose.get();
        }
    }

    public boolean updatePose(){
        return super.getLatestResult().hasTargets();
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        if (latestResult != null)
            return photonPoseEstimator.update(latestResult);
        else
            return null;
    }
    public Pose2d getRobotPose(){
        newPose = estRobotPose.estimatedPose.transformBy(cameraToRobotCenter).toPose2d();
        return newPose;
    }
    public double getTime(){
        if(latestResult != null){
            return photonPoseEstimator.update(latestResult).get().timestampSeconds;
        }
        else{
            return Timer.getFPGATimestamp();
        }
    }
    public Matrix<N3, N1> getPoseAmbiguity(){
        double smallestDistance = Double.POSITIVE_INFINITY;
        double confidenceMultiplier = 0;
        if(estRobotPose.targetsUsed != null){
            for (var target : estRobotPose.targetsUsed) {
                var t3d = target.getBestCameraToTarget();
                var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
                if (distance < smallestDistance) {
                    smallestDistance = distance;
                }
            }
            double poseAmbiguityFactor = estRobotPose.targetsUsed.size() != 1
                ? 1
                : Math.max(1, estRobotPose.targetsUsed.get(0).getPoseAmbiguity() + Constants.vision.POSE_AMBIGUITY_SHIFTER * Constants.vision.POSE_AMBIGUITY_MULTIPLIER);
            confidenceMultiplier = Math.max(1,
                (Math.max(1, Math.max(0, smallestDistance - Constants.vision.NOISY_DISTANCE_METERS) * Constants.vision.DISTANCE_WEIGHT) * poseAmbiguityFactor) 
                / (1 + ((estRobotPose.targetsUsed.size() - 1) * Constants.vision.TAG_PRESENCE_WEIGHT)));
        }
        SmartDashboard.putNumber(super.getName(), confidenceMultiplier);
        return Constants.vision.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }

    public Transform3d getCameraToRobot(){
        return cameraToRobotCenter;
    }
}

