package frc.robot.subsystems;

import frc.lib.util.LimelightHelpers;
import frc.lib.util.LimelightHelpers.LimelightResults;
import frc.robot.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
*/

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class PoseEstimatorSub extends SubsystemBase {

    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator poseEstimator;
    //PhotonPoseEstimator photonPoseEstimator;
    SwerveSub swerveSub;

    Field2d field = new Field2d();

    //PhotonCamera shooterCam = new PhotonCamera(Constants.PoseEstimatorSub.shooterCamName);

    int visionCount = 0;
    
    public PoseEstimatorSub() { 
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
    }

    public void initialize(SwerveSub swerveSub) {
        this.swerveSub = swerveSub;

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            swerveSub.getModulePositions(),
            getCloseSpeakerPose()
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

    public void setPoseTranslation(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getModulePositions(), new Pose2d(pose.getTranslation(), getPose().getRotation()));
    }

    public void resetPoseToCloseSpeaker() {
        setPoseTranslation(getCloseSpeakerPose());
    }

    public void resetPoseToProtected() {
        setPoseTranslation(getProtectedPose());
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

    public Pose2d getCloseSpeakerPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return Constants.PoseEstimatorSub.redCloseSpeakerPose;
            } else {
                return Constants.PoseEstimatorSub.blueCloseSpeakerPose;
            }
        }
        else return Constants.PoseEstimatorSub.blueCloseSpeakerPose;
    }

    public Pose2d getProtectedPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return Constants.PoseEstimatorSub.redProtectedPose;
            } else {
                return Constants.PoseEstimatorSub.blueProtectedPose;
            }
        }
        else return Constants.PoseEstimatorSub.blueProtectedPose;
    }

    public Pose2d getSpeakerPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return Constants.PoseEstimatorSub.redSpeakerPose;
            } else {
                return Constants.PoseEstimatorSub.blueSpeakerPose;
            }
        }
        else return Constants.PoseEstimatorSub.blueSpeakerPose;
    }
    
    public double getTargetYaw() {
        double targetYaw = (180 / Math.PI) * Math.atan( 1.0 *
            (getPose().getY() - getSpeakerPose().getY()) / 
            (getPose().getX() - getSpeakerPose().getX())
            );
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            if (targetYaw > 0) targetYaw -= 180;
            else targetYaw += 180;
            return targetYaw;
        } else {
                return targetYaw;
        }
        /*var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                if (targetYaw >= 0) return targetYaw;
                else return targetYaw;
            } else {
                return targetYaw;
            }
        } else {
            return  targetYaw;
        }*/
    }
    
    public double getTargetPitch() {
        return (180 / Math.PI) * Math.atan(
            Constants.PoseEstimatorSub.speakerTargetHeight / 
            Math.sqrt(
                Math.pow(getPose().getX() - getSpeakerPose().getX(), 2) +
                Math.pow(getPose().getY() - getSpeakerPose().getY(), 2)
            )
        ) + Constants.PoseEstimatorSub.shootkG * Math.sqrt(
            Math.pow(getPose().getX() - getSpeakerPose().getX(), 2) +
            Math.pow(getPose().getY() - getSpeakerPose().getY(), 2)
        );
    }

    public boolean getValidNote() {
        return LimelightHelpers.getTV("intakeLimelight");
    }

    public double getNoteYaw() {
        return LimelightHelpers.getTX("intakeLimelight");
    }

    public void update() {
        poseEstimator.update(getGyroYaw(), swerveSub.getModulePositions());

        /* LimeLight Code Copied from website */
        LimelightHelpers.PoseEstimate limelightBotpose = LimelightHelpers.getBotPoseEstimate_wpiBlue("shooterLimelight");
        if(limelightBotpose.tagCount >= 2) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(12,12,9999999));
            poseEstimator.addVisionMeasurement(
                limelightBotpose.pose,
                limelightBotpose.timestampSeconds);
        } /*else if (LimelightHelpers.getTA("limelight") > 0.6) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            poseEstimator.addVisionMeasurement(
                limelightBotpose.pose,
                limelightBotpose.timestampSeconds);
        }*/

        field.setRobotPose(getPose());

        /*Optional<EstimatedRobotPose> optionalEstimatedPose = photonPoseEstimator.update();
        if (optionalEstimatedPose.isPresent()) {
            EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();
            poseEstimator.addVisionMeasurement(
                estimatedPose.estimatedPose.toPose2d(),
                estimatedPose.timestampSeconds);
                visionCount += 1;
        }*/
    }

    @Override //This method is called continuously
    public void periodic() {

        update();
        
        SmartDashboard.putData("Field", field);


        //SmartDashboard.putNumber("targetDistance", PhotonUtils.getDistanceToPose(getPose(), getSpeakerTargetPose()));
        SmartDashboard.putNumber("targetPitch", getTargetPitch());
        SmartDashboard.putNumber("targetYaw", getTargetYaw());
        //SmartDashboard.putNumber("heading", getHeading().getDegrees());
        //SmartDashboard.putNumber("gyro heading", getGyroYaw().getDegrees());
        //SmartDashboard.putNumber("estimatorPositions", swerveSub.getRobotRelativeSpeeds().vxMetersPerSecond);

        //SmartDashboard.putNumber("poseX", getPose().getTranslation().getX());
        //SmartDashboard.putNumber("poseY", getPose().getTranslation().getY());
        SmartDashboard.putNumber("poseRotation", getPose().getRotation().getDegrees());

        //SmartDashboard.putNumber("visionCount", visionCount);
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
