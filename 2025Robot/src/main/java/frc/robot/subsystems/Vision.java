package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.utils.Camera;

public class Vision extends SubsystemBase{
    
    private final AHRS gyro;
    public Pose2d robotPose = new Pose2d();
    public Pose3d estPose3d = new Pose3d();
    private Transform2d tagAllignment = new Transform2d();
    private VisionState state = VisionState.Climb;
    private int wantedTarget = 0;

    public final Camera frontCamera = new Camera(
        new PhotonCamera("OV9281_2"), 
        Constants.kVision.cameraToRobotCenter2, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );
    public final Camera backCamera = new Camera(
        new PhotonCamera("OV9281_1"), 
        Constants.kVision.cameraToRobotCenter1, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );

    public Camera[] cameraList = new Camera[] {
        frontCamera,
        backCamera,
    };

    public int[] reefTags = new int[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

    /**
     * States for the vision state machine
     * States include: "LeftReef", "RightReef", "ReefL1", "FeederStation", "Processor", "Climb"
     */
    public enum VisionState{
        LeftReef,
        RightReef,
        ReefL1,
        FeederStation,
        Processor,
        Climb,
        CenterReef,
    }

    /**
     * Sets local gyro equal to the input gyro.
     * @param gyro -Type "AHRS"
     */
    public Vision(AHRS gyro){
        this.gyro = gyro;
    }

    /**
     * Sets local state to input state.
     * @param state -Type "VisionState"
     */
    public void setState(VisionState state){
        this.state = state;
    }

    /**
     * Changes tagAllignment based on the state.
     */
    private void handleState(){
        switch (state) {
            case LeftReef:
                tagAllignment = new Transform2d(0.1, -1.2954, Rotation2d.fromDegrees(0));
                break;

            case RightReef:
                tagAllignment = new Transform2d(0.1, 1.2954, Rotation2d.fromDegrees(0));
                break;
            case CenterReef:
                tagAllignment = new Transform2d(0.1, 0, Rotation2d.fromDegrees(0));
                break;

            case ReefL1:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;

            case FeederStation:
                tagAllignment = new Transform2d(0, 0.1, Rotation2d.fromDegrees(0));
                break;

            case Processor:
                tagAllignment = new Transform2d(0, 0.1, Rotation2d.fromDegrees(0));
                break;

            case Climb:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;
                
            default:
                break;
        }
    }

    /**
     * Periodic method called 50 times per second.
     * <p>
     * Updates the estimated position based off of any apriltags seen.
     * Uses getEstimatedGlobalPose, transformBy, and getCameraToRobot.
     * @see -getEstimatedGlobalPose is located under frc.robot.utils.Camera.
     * @see -Link to transformBy: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Pose3d.html#transformBy(edu.wpi.first.math.geometry.Transform3d).
     * @see -getCameraToRobot is located under frc.robot.utils.Camera.
     */
    public void periodic(){
        Optional<EstimatedRobotPose> estPose = null;

         for(Camera camera : cameraList){
            if(camera.getEstimatedGlobalPose(robotPose) != null){
                estPose = camera.getEstimatedGlobalPose(robotPose);
            }
            robotPose = camera.estRobotPose.estimatedPose.transformBy(camera.getCameraToRobot()).toPose2d();
            if(estPose != null){
                if(estPose.isPresent()){
                    camera.estRobotPose = estPose.get();
                }
            }

            camera.periodic();
        }
        handleState();
    }

    /**
     * Gets the camera with the lowest ambiguity that can see the specified target
     * @param id The ID of the specified target
     * @param ambiguityThreshold Value from 0-1 determining the maximum uncertainty (ambiguity) allowed. 1 allowing for more error
     * @return The camera that can see the target with the lowest ambiguity. null if none meet the condition
     */
    public Camera getBestCameraFromTarget(int id, double ambiguityThreshold) {
        Camera bestCamera = null;
        double bestAmbiguity = ambiguityThreshold;
        for(Camera camera : cameraList){
            if(camera.canSeeTarget(id)){
                for(PhotonTrackedTarget target : camera.latestResult.getTargets()){
                    if(target.getFiducialId() == id){
                        if(target.poseAmbiguity <= bestAmbiguity){
                            bestCamera = camera;
                            bestAmbiguity = target.poseAmbiguity;
                        }
                    }
                }
            }
        }
        return bestCamera;
    }

    /**
     * Gets the translation needed to align to the aprilTag.
     * @param id -Type "int", the specified apriltag.
     * @return Type "Transform2d", If no apriltag found, returns null. if an apriltag is found, updates SmartDashboard and returns a transform2d to allign with the apriltag
     */
    public Transform3d getRobotToTarget(int id){
        Camera camera = getBestCameraFromTarget(id, 1);
        Transform3d alignment = null;
        Pose3d pose;
        if(camera != null){
            pose = camera.getCameraPose().transformBy(camera.getAlignmentToTarget(id));
            alignment = new Transform3d(new Pose3d(), pose);
        }
        return alignment;
    }

    /**
     * Checks if the robot is aligned at the target or not.
     * @param id -Type "int", the specified apriltag.
     * @param setpoints -Type "Transform2d", used as a destination for the robot to sit at
     * @param deadzone -Type "double", used to allow clearance greater that 0 in where the robot is considered "at the setpoint".
     * @return Type "boolean", true if aligned with the target, false otherwise.
     */
    public boolean isRobotAlignedWithTarget(int id, Transform2d setpoints, double deadzone){
        Camera camera = getBestCameraFromTarget(id, 1);
        if (camera != null){
            double x = camera.getAlignmentToTarget(id).getX();
            double y = camera.getAlignmentToTarget(id).getY();
            double rot = camera.getAlignmentToTarget(id).getRotation().toRotation2d().getDegrees() - setpoints.getRotation().getDegrees();

            if(x <= setpoints.getX() + deadzone && x >= setpoints.getX() - deadzone){
                if(y <= setpoints.getY() + deadzone && y >= setpoints.getY() - deadzone){
                    if(rot <= setpoints.getRotation().getDegrees() + deadzone && rot >= setpoints.getRotation().getDegrees() - deadzone){
                        return true;
                    }
                }
            }
        }
        return false;
    }

    public Transform2d getTagAlignment(){
        return tagAllignment;
    }

    public void setWantedTarget(int id){
        wantedTarget = id;
    }
    public int getWantedTarget(){
        return wantedTarget;
    }

    public int getNearestReefTag(Pose2d pose){
        List<Pose2d> poseList = new ArrayList<>();
        for(int id : reefTags){
            if(frontCamera.apriltagMap.getTagPose(id).isPresent())
                poseList.add(frontCamera.apriltagMap.getTagPose(id).get().toPose2d());
        }    
        pose.nearest(poseList);
        for(int tag : reefTags){
            if(frontCamera.apriltagMap.getTagPose(frontCamera.apriltagMap.getTags().get(tag).ID).get().toPose2d().getTranslation().equals(pose.getTranslation())){
                return tag;
            }
        }
        return 0;
    }
}
