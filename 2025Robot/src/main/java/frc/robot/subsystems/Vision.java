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
    public final Camera orangepi1 = new Camera(
        new PhotonCamera("OV9281_1"), 
        Constants.kVision.cameraToRobotCenter2, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );
    public final Camera orangepi2 = new Camera(
        new PhotonCamera("OV9281_3"), 
        Constants.kVision.cameraToRobotCenter3, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );

    public Camera[] cameraList = new Camera[] {
        orangepi1,
        //orangepi2,
    };

    public enum VisionState{
        LeftReef,
        RightReef,
        ReefL1,
        FeederStation,
        Processor,
        Climb,
    }

    public Vision(AHRS gyro){
        this.gyro = gyro;
    }

    public void setState(VisionState state){
        this.state = state;
    }

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

    public boolean canSeeTarget(int id, Camera camera) {
        List<PhotonTrackedTarget> latest = camera.latestResult.getTargets();
        if(latest != null){
            for(PhotonTrackedTarget target : latest){
                if(target.getFiducialId() == id){
                    return true;
                }
            }
        }
        return false;
    }

    public double rotateToTarget(int id, Camera camera, double speed){
        List<PhotonTrackedTarget> latest = camera.latestResult.getTargets();
            if(canSeeTarget(id, camera)){
                return speed;
            } 
        return 0;
    }

    public Transform2d getAlignmentToTarget(int id, Camera camera){
        List<PhotonTrackedTarget> latest = camera.latestResult.getTargets();
        if(latest != null){
            for(PhotonTrackedTarget target : latest){
                if(canSeeTarget(id, camera)){
                    Transform3d cameraToTarget = target.getBestCameraToTarget();

                    return new Transform2d(cameraToTarget.getY(), cameraToTarget.getX(), Rotation2d.fromDegrees(target.getYaw()));
                }
            }
        }
        return null;
    }
}
