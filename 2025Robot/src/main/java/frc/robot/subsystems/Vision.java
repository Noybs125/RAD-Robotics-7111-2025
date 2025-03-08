package frc.robot.subsystems;

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

    // TODO: change variable names on actual robot
    public final Camera limelight = new Camera(
        new PhotonCamera("photonvision"), 
        Constants.kVision.cameraToRobotCenter1, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );
    public final Camera frontCamera = new Camera(
        new PhotonCamera("OV9281_1"), 
        Constants.kVision.cameraToRobotCenter2, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );
    public final Camera backCamera = new Camera(
        new PhotonCamera("OV9281_3"), 
        Constants.kVision.cameraToRobotCenter3, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );

    public Camera[] cameraList = new Camera[] {
        frontCamera,
        backCamera,
    };

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
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;

            case RightReef:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;

            case ReefL1:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;

            case FeederStation:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;

            case Processor:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
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
     * Checks if a specified camera can see a specified apriltag.
     * @param id -Type "int", apriltag id
     * @param camera -Type "Camera", specified camera object
     * @return -True if id seen by camera, false otherwise.
     */
    public boolean canSeeTarget(int id, Camera camera) {
        if (camera.latestResult != null) {
            List<PhotonTrackedTarget> latest = camera.latestResult.getTargets();
            if(latest != null){
                for(PhotonTrackedTarget target : latest){
                    if(target.getFiducialId() == id){
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * Calculates the speed required to rotate to a specified target.
     * @param id -Type "int", the specified apriltag.
     * @param camera -Type "Camera", the camera object to check for the target.
     * @param speed -Type "double", the speed to turn toward the target
     * @return -Speed if target is identified in the camera, 0 otherwise.
     */
    public double rotateToTarget(int id, Camera camera, double speed){
        if (camera.latestResult != null) {
            List<PhotonTrackedTarget> latest = camera.latestResult.getTargets();
                if(canSeeTarget(id, camera)){
                 return speed;
               } 
            }
        return 0;
    }

    /**
     * returns the translation needed to align to the aprilTag.
     * @param id -Type "int", the specified apriltag.
     * @param camera -Type "Camera", the camera object to check for the target
     * @return Type "Transform2d", If no apriltag found, returns null. if an apriltag is found, updates SmartDashboard and returns a transform2d to allign with the apriltag
     */
    public Transform2d getAlignmentToTarget(int id, Camera camera){
        Transform2d alignment;
        Pose3d pose;
        pose = camera.getCameraPose().transformBy(camera.getAlignmentToTarget(id));
        alignment = new Transform2d(new Pose2d(), pose.toPose2d());
        return alignment;
    }

    /**
     * Checks if the robot is aligned at the target or not.
     * @param id -Type "int", the specified apriltag.
     * @param camera -Type "Camera", the camera object to check for the target.
     * @param setpoints -Type "Transform2d", used as a destination for the robot to sit at
     * @param deadzone -Type "double", used to allow clearance greater that 0 in where the robot is considered "at the setpoint".
     * @return Type "boolean", true if aligned with the target, false otherwise.
     */
    public boolean isAtTarget(int id, Camera camera, Transform2d setpoints, double deadzone){
        if(canSeeTarget(id, camera)){
            double x = getAlignmentToTarget(id, camera).getX();
            double y = getAlignmentToTarget(id, camera).getY();
            double rot = getAlignmentToTarget(id, camera).getRotation().getDegrees() - setpoints.getRotation().getDegrees();

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
}
