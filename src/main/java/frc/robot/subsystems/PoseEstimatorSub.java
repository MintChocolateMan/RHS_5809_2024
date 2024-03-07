package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class PoseEstimatorSub extends SubsystemBase {

    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator poseEstimator;
    PhotonPoseEstimator photonPoseEstimator;
    SwerveSub swerveSub;

    PhotonCamera shooterCam = new PhotonCamera(Constants.PoseEstimatorSub.shooterCamName);

    int visionCount = 0;
    
    public PoseEstimatorSub() { 
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        shooterCam = new PhotonCamera(Constants.PoseEstimatorSub.shooterCamName);

        photonPoseEstimator = new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            shooterCam,
            Constants.PoseEstimatorSub.robotToShooterCam
        );
    }

    public void initialize(SwerveSub swerveSub) {
        this.swerveSub = swerveSub;

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            swerveSub.getModulePositions(),
            getStartingPose() //getStartingPose()
        );
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getModulePositions(), pose);
    }
    
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Pose2d getStartingPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return Constants.PoseEstimatorSub.redStartingPose;
            } else {
                return Constants.PoseEstimatorSub.blueStartingPose;
            }
        }
        else return Constants.PoseEstimatorSub.blueStartingPose;
    }

    public Pose2d getSpeakerTargetPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return Constants.PoseEstimatorSub.redSpeakerTarget;
            } else {
                return Constants.PoseEstimatorSub.blueSpeakerTarget;
            }
        }
        else return Constants.PoseEstimatorSub.blueSpeakerTarget;
    }
    
    public double getTargetYaw() {
        return (180 / Math.PI) * Math.atan(
            (getPose().getY() - getSpeakerTargetPose().getY()) / 
            (getPose().getX() - getSpeakerTargetPose().getX())
            
        );
        //return PhotonUtils.getYawToPose(getPose(), getSpeakerTargetPose()).getDegrees();
    }

    public double getSpeakerTargetPitch() {
        return Math.atan(
            Constants.PoseEstimatorSub.speakerTargetHeight / 
            PhotonUtils.getDistanceToPose(getPose(), getSpeakerTargetPose())
            ) * 180 / Math.PI;
    }

    public void update() {
        poseEstimator.update(getGyroYaw(), swerveSub.getModulePositions());

        Optional<EstimatedRobotPose> optionalEstimatedPose = photonPoseEstimator.update();
        if (optionalEstimatedPose.isPresent()) {
            EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();
            poseEstimator.addVisionMeasurement(
                estimatedPose.estimatedPose.toPose2d(),
                estimatedPose.timestampSeconds);
                visionCount += 1;
                SmartDashboard.putNumber("visionCount", visionCount);
        }
    }

    @Override //This method is called continuously
    public void periodic() {
        update();

        SmartDashboard.putNumber("targetDistance", PhotonUtils.getDistanceToPose(getPose(), getSpeakerTargetPose()));
        SmartDashboard.putNumber("targetPitch", getTargetPitch());
        SmartDashboard.putNumber("targetYaw", getTargetYaw());

        //SmartDashboard.putNumber("gyro heading", getGyroYaw().getDegrees());
        //SmartDashboard.putNumber("estimatorPositions", swerveSub.getRobotRelativeSpeeds().vxMetersPerSecond);

        SmartDashboard.putNumber("poseX", getPose().getTranslation().getX());
        SmartDashboard.putNumber("poseY", getPose().getTranslation().getY());
        SmartDashboard.putNumber("poseRotation", getPose().getRotation().getDegrees());
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
