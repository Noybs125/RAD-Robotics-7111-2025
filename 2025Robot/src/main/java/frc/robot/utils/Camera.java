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
import edu.wpi.first.math.geometry.Pose3d;
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
    public PhotonPipelineResult latestResult;
    private PhotonTrackedTarget bestTarget;
    private Transform3d bestCameraToTarget;
    public EstimatedRobotPose estRobotPose;
    public int bestTargetId;
    public AprilTagFieldLayout apriltagMap;
    private Pose2d newPose = new Pose2d();
    {
        try {
            apriltagMap = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
        } catch (IOException e){
            throw new RuntimeException(e);
        }
    }
    
    private AHRS gyro;
    /**
     * Instantiates the Camera object to check for apriltags.
     * @param camera - The chosen camera to use.
     * @param cameraToRobotCenter - The distance from the center of the robot to the camera.
     * @param estRobotPose - The estimated Robot Pose.
     * @param vision - The vision subsystem in use.
     * 
     */
    public Camera(PhotonCamera camera, Transform3d cameraToRobotCenter, EstimatedRobotPose estRobotPose, Vision vision) {
        super(camera.getName());
        this.cameraToRobotCenter = cameraToRobotCenter;
        this.vision = vision;
        this.estRobotPose = estRobotPose;
        photonPoseEstimator = new PhotonPoseEstimator(apriltagMap, PoseStrategy.AVERAGE_BEST_TARGETS, cameraToRobotCenter);
    }
    /**
     * Runs every tick to check for Camera results and set the estimated Robot Pose.
     */
    public void periodic(){

        for(PhotonPipelineResult result : getAllUnreadResults()){
            latestResult = result;
        }
        
        if(latestResult != null && latestResult.hasTargets()){
            bestTarget = latestResult.getBestTarget();
            bestCameraToTarget = bestTarget.getBestCameraToTarget();
            bestTargetId = bestTarget.getFiducialId();
        }
        Optional<EstimatedRobotPose> estPose = getEstimatedGlobalPose(vision.robotPose);
        if(estPose != null && estPose.isPresent()){
            if(photonPoseEstimator.getRobotToCameraTransform() != cameraToRobotCenter){
                photonPoseEstimator.setRobotToCameraTransform(cameraToRobotCenter);
            }
            estRobotPose = estPose.get();
        }

        SmartDashboard.putNumber("camera " + getName() + " yaw", photonPoseEstimator.getRobotToCameraTransform().getRotation().getZ());
    }

    public boolean canSeeTarget(int id){
        boolean canSee = false;
        if(updatePose()){
            if(latestResult.hasTargets()){
                for(var target : latestResult.getTargets()){
                    if(target.getFiducialId() == id){
                        canSee = true;
                    }
                }
            }
        }
        return canSee;
    }

    public Transform3d getAlignmentToTarget(int id){
        Transform3d transform = null;
        if(canSeeTarget(id)){
            for(var target : latestResult.getTargets()){
                if(target.getFiducialId() == id){
                    transform = target.getBestCameraToTarget();
                }
            }
        }
        return transform;
    }

    public Pose3d getCameraPose(){
        return new Pose3d(cameraToRobotCenter.getTranslation(), cameraToRobotCenter.getRotation());
    }

    public PhotonTrackedTarget getBestTarget(){
        if(latestResult != null && latestResult.hasTargets()){
            return latestResult.getBestTarget();
        }
        return null;
    }

    /**
     * Updates the photonPoseEstimator with the newest apriltags.
     * @return The updated photonPoseEstimator.
     */
    public boolean updatePose(){
        if (latestResult != null) {
            return latestResult.hasTargets();
        }

        else {
            return false;
        }
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        if (latestResult != null)
            return photonPoseEstimator.update(latestResult);
        else
            return null;
    }
    /**
     * 
     * @return The Robot Pose transformed by the distance from camera to Robot Center.
     */
    public Pose2d getRobotPose(){
        newPose = estRobotPose.estimatedPose.toPose2d();
        return newPose;
    }
    /**
     * 
     * @return The Timestamp for the last apriltag seen.
     */
    public double getTime(){
        if(latestResult != null){
            return photonPoseEstimator.update(latestResult).get().timestampSeconds;
        }
        else{
            return Timer.getFPGATimestamp();
        }
    }
    /**
     * Calculates the Pose Ambiguity using Constants in kVision for each apriltag.
     * @return The Pose Ambiguity.
     */
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
                : Math.max(1, estRobotPose.targetsUsed.get(0).getPoseAmbiguity() + Constants.kVision.POSE_AMBIGUITY_SHIFTER * Constants.kVision.POSE_AMBIGUITY_MULTIPLIER);
            confidenceMultiplier = Math.max(1,
                (Math.max(1, Math.max(0, smallestDistance - Constants.kVision.NOISY_DISTANCE_METERS) * Constants.kVision.DISTANCE_WEIGHT) * poseAmbiguityFactor) 
                / (1 + ((estRobotPose.targetsUsed.size() - 1) * Constants.kVision.TAG_PRESENCE_WEIGHT)));
        }
        SmartDashboard.putNumber(getName() + " CONFIDENCE OF TAG", confidenceMultiplier);
        return Constants.kVision.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }

    public Transform3d getCameraToRobot(){
        return cameraToRobotCenter;
    }

    public PhotonTrackedTarget getTarget(int id){
        for (PhotonTrackedTarget target : latestResult.getTargets()) {
            if (target.getFiducialId() == id) {
                return target;
            }
        }
        return null;
    }
}

