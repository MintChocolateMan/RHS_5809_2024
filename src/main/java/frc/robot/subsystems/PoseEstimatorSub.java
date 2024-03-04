package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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

    //Declare motors and sensors
    Pigeon2 gyro;
    SwerveDrivePoseEstimator poseEstimator;
    PhotonCamera shooterCam;
    PhotonPoseEstimator photonPoseEstimator;
    Transform3d robotToShooterCam;
    SwerveSub swerveSub;
    
    public PoseEstimatorSub() { //Subsystem constructor
        //Initialize motors and sensors
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        shooterCam = new PhotonCamera(Constants.PoseEstimatorSub.shooterCamName);

        robotToShooterCam = new Transform3d(
            new Translation3d(
                Constants.PoseEstimatorSub.shooterCamForwardOffset,
                Constants.PoseEstimatorSub.shooterCamHorizontalOffset,
                Constants.PoseEstimatorSub.shooterCamVerticalOffset
            ), new Rotation3d(
                Constants.PoseEstimatorSub.shooterCamYaw,
                Constants.PoseEstimatorSub.shooterCamRoll,
                Constants.PoseEstimatorSub.shooterCamPitch
            )
        );

        photonPoseEstimator = new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            shooterCam,
            robotToShooterCam
        );
    }

    //Declare subsystem suppliers
    public void initialize(SwerveSub swerveSub) {
        this.swerveSub = swerveSub;

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            swerveSub.getModulePositions(),
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0))
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
    
    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Pose2d getSpeakerTargetPose() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return new Pose2d(
                new Translation2d(16.58, 5.5),
                new Rotation2d()
            );
        } else {
                return new Pose2d(
                    new Translation2d(-0.04, 5.55),
                    new Rotation2d()
                );
        }
    }
    
    public double getTargetYaw() {
        return PhotonUtils.getYawToPose(getPose(), getSpeakerTargetPose()).getDegrees();
    }

    public double getTargetPitch() {
        return Math.tanh(
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
        }
    }

    //Declare methods
    public void exampleMethod() {}

    //Declare inline Commands
    public Command ExampleInlineCommand() {
        return runOnce(() -> {
        });
    }

    @Override //This method is called continuously
    public void periodic() {
        poseEstimator.update(getGyroYaw(), swerveSub.getModulePositions());

        
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
