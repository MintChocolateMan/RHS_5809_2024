package frc.robot.subsystems;

import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class PoseEstimatorSub extends SubsystemBase {

    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator poseEstimator;
    SwerveSub swerveSub;

    double visionStdDevs;

    Field2d field = new Field2d();
    
    public PoseEstimatorSub() { 
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        visionStdDevs = 1;
    }

    public void initialize(SwerveSub swerveSub) {
        this.swerveSub = swerveSub;

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            swerveSub.getModulePositions(),
            getCloseSpeakerPose()//,
            //VecBuilder.fill(.1, .1, .1),
            //VecBuilder.fill(.5, .5, 9999999)
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

    public double getVisionStdDevs() {
        return visionStdDevs;
    }

    public void setVisionStdDevs(double visionStdDevs) {
        this.visionStdDevs = visionStdDevs;
    }

    public Rotation2d getHeadingFieldOriented() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() == false) return getPose().getRotation();
        if (alliance.get() == DriverStation.Alliance.Blue) {
            return getPose().getRotation();
        } else {
            if (getPose().getRotation().getDegrees() <= 0) {
                return new Rotation2d(getPose().getRotation().getRadians() + Math.PI);
            } else return new Rotation2d(getPose().getRotation().getRadians() - Math.PI);
        }
    }

    public void setHeading(Rotation2d heading) {
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() == true && alliance.get() == DriverStation.Alliance.Red) {
            poseEstimator.resetPosition(getGyroYaw(), swerveSub.getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d(Math.PI)));
        } else {
            poseEstimator.resetPosition(getGyroYaw(), swerveSub.getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        }
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

    public boolean getValidAmp() {
        if (LimelightHelpers.getTV("limelight-amp") == true) return true;
        else return false;
    }

    public double getAmpTY() {
        return LimelightHelpers.getTY("limelight-amp");
    }
    
    public double getAmpTX() {
        return LimelightHelpers.getTX("limelight-amp");
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

    /*public double getTargetPitch() {
        double currentValue;
        double bestValue = 99999999;
        for (double i = 30; i < 62; i += .25) {
            currentValue = ((((Constants.ShooterSub.initialVelocity * Math.sin(i * Math.PI / 180) *
                Math.sqrt(
                Math.pow(getPose().getX() - getSpeakerPose().getX(), 2) +
                Math.pow(getPose().getY() - getSpeakerPose().getY(), 2) + .2
                )) / (Constants.ShooterSub.initialVelocity * Math.cos(i * Math.PI / 180) )) + 
                (
                (-9.8 / 2) *Math.pow(((
                Math.pow(getPose().getX() - getSpeakerPose().getX(), 2) +
                Math.pow(getPose().getY() - getSpeakerPose().getY(), 2) + .2
                ) / (Constants.ShooterSub.initialVelocity * Math.cos(i * Math.PI / 180))), 2))
                ) - Constants.PoseEstimatorSub.speakerTargetHeight);
            if (currentValue < bestValue) bestValue = currentValue;
        }
        return bestValue;
    }*/
    
    public double getTargetPitch() {
        return (180 / Math.PI) * Math.atan(
            Constants.PoseEstimatorSub.speakerTargetHeight / 
            Math.sqrt(
                Math.pow(getPose().getX() - getSpeakerPose().getX(), 2) +
                Math.pow(getPose().getY() - getSpeakerPose().getY(), 2) + .2
            )
        //) + Constants.PoseEstimatorSub.shootkG * Math.sqrt(
           // Math.pow(getPose().getX() - getSpeakerPose().getX(), 2) +
           // Math.pow(getPose().getY() - getSpeakerPose().getY(), 2)
        );
    }

    public boolean getValidNote() {
        return LimelightHelpers.getTV("limelight-intake");
    }

    public double getNoteYaw() {
        return LimelightHelpers.getTX("limelight-intake");
    }

    public void update() {
        poseEstimator.update(getGyroYaw(), swerveSub.getModulePositions());

        /* LimeLight Code Copied from website */
        LimelightHelpers.PoseEstimate limelightBotpose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
        if(limelightBotpose.tagCount >= 2) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(getVisionStdDevs(),getVisionStdDevs(),9999999));
            poseEstimator.addVisionMeasurement(
                limelightBotpose.pose,
                Timer.getFPGATimestamp() -
                    (LimelightHelpers.getLatency_Pipeline("limelight-shooter") / 1000) -
                    (LimelightHelpers.getLatency_Capture("limelight-shooter") / 1000)
            );
                //limelightBotpose.timestampSeconds);
        }

        field.setRobotPose(getPose());
    }

    @Override //This method is called continuously
    public void periodic() {

        update();
        
        SmartDashboard.putData("Field", field);


        //SmartDashboard.putNumber("targetDistance", PhotonUtils.getDistanceToPose(getPose(), getSpeakerTargetPose()));
        //SmartDashboard.putNumber("targetPitch", getTargetPitch());
        //SmartDashboard.putNumber("targetYaw", getTargetYaw());
        SmartDashboard.putNumber("heading", getHeading().getDegrees());
        //SmartDashboard.putNumber("gyro heading", getGyroYaw().getDegrees());
        //SmartDashboard.putNumber("estimatorPositions", swerveSub.getRobotRelativeSpeeds().vxMetersPerSecond);

        //SmartDashboard.putNumber("poseX", getPose().getTranslation().getX());
        //SmartDashboard.putNumber("poseY", getPose().getTranslation().getY());
        //SmartDashboard.putNumber("poseRotation", getPose().getRotation().getDegrees());

        //SmartDashboard.putNumber("visionCount", visionCount);
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
