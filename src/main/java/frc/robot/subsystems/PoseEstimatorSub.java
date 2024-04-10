package frc.robot.subsystems;

import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

    private boolean autoNoteSeen;
    
    public PoseEstimatorSub() { 
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        visionStdDevs = Constants.PoseEstimatorSub.autoVisionStdDevs;

        autoNoteSeen = false;
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

    public void setPoseTranslation(Translation2d translation) {
        poseEstimator.resetPosition(getGyroYaw(), swerveSub.getModulePositions(), new Pose2d(translation, getPose().getRotation()));
    }

    public void resetPoseToCloseSpeaker() {
        setPoseTranslation(getCloseSpeakerPose());
    }
    
    public Rotation2d getHeading() {
        return getPose().getRotation();
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

    public double getVisionStdDevs() {
        return visionStdDevs;
    }

    public void setVisionStdDevs(double visionStdDevs) {
        this.visionStdDevs = visionStdDevs;
    }

    public void setStandardVisionStdDevs() {
        if (DriverStation.isAutonomousEnabled() == true) {
            setVisionStdDevs(Constants.PoseEstimatorSub.autoVisionStdDevs);
        } else {
            setVisionStdDevs(Constants.PoseEstimatorSub.teleopVisionStdDevs);
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

    public Pose2d getAmpPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Constants.PoseEstimatorSub.redAmpPose;
        } else {
            return Constants.PoseEstimatorSub.blueAmpPose;
        }
    }

    public Pose2d getSpeakerPoseYaw() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return Constants.PoseEstimatorSub.redSpeakerPoseYaw;
            } else {
                return Constants.PoseEstimatorSub.blueSpeakerPoseYaw;
            }
        }
        else return Constants.PoseEstimatorSub.blueSpeakerPoseYaw;
    }

    public Pose2d getSpeakerPoseDistance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                return Constants.PoseEstimatorSub.redSpeakerPoseDistance;
            } else {
                return Constants.PoseEstimatorSub.blueSpeakerPoseDistance;
            }
        }
        else return Constants.PoseEstimatorSub.blueSpeakerPoseYaw;
    }

    public boolean getValidAmp() {
        LimelightHelpers.setPipelineIndex("limelight-amp", 0);
        if (LimelightHelpers.getTV("limelight-amp") == true) return true;
        else return false;
    }

    public double getAmpTY() {
        LimelightHelpers.setPipelineIndex("limelight-amp", 0);
        return LimelightHelpers.getTY("limelight-amp");
    }
    
    public double getAmpTX() {
        LimelightHelpers.setPipelineIndex("limelight-amp", 0);
        return LimelightHelpers.getTX("limelight-amp");
    }

    public boolean getValidTrap() {
        LimelightHelpers.setPipelineIndex("limelight-amp", 1);
        if (LimelightHelpers.getTV("limelight-amp") == true) return true;
        else return false;
    }

    public double getTrapTY() {
        LimelightHelpers.setPipelineIndex("limelight-amp", 1);
        return LimelightHelpers.getTY("limelight-amp");
    }
    
    public double getTrapTX() {
        LimelightHelpers.setPipelineIndex("limelight-amp", 1);
        return LimelightHelpers.getTX("limelight-amp");
    }

    public boolean getValidNote() {
        return LimelightHelpers.getTV("limelight-intake");
    }

    public double getNoteYaw() {
        LimelightHelpers.setPipelineIndex("limelight-amp", 0);
        return LimelightHelpers.getTX("limelight-intake");
    }

    public double getNoteTY() {
        return LimelightHelpers.getTX("limelight-intake");
    }

    public boolean getAutoNoteSeen() {
        return autoNoteSeen;
    }

    public void setAutoNoteSeen(boolean autoNoteSeen) {
        this.autoNoteSeen = autoNoteSeen;
    }

    public boolean getValidSpeaker() {
        if (LimelightHelpers.getTV("limelight-Shooter") == true) return true;
        else return false;
    }

    public double getSpeakerTX() {
        return LimelightHelpers.getTX("limelight-shooter");
    }

    public double getSpeakerTY() {
        return LimelightHelpers.getTY("limelight-shooter");
    }

    public double getCameraSpeakerDistance() {
        return (
            (Constants.PoseEstimatorSub.speakerTagHeight - Constants.PoseEstimatorSub.cameraHeight) /
            Math.tan(Math.PI / 180 * (
                30 + getSpeakerTY()
            ))
        );
    }

    public double getSpeakerDistance() {
        return (
            Math.sqrt(
                Math.pow(Constants.PoseEstimatorSub.cameraOffset, 2) +
                Math.pow(getCameraSpeakerDistance(), 2) - (
                    2 * Constants.PoseEstimatorSub.cameraOffset * getCameraSpeakerDistance() * Math.cos(Math.PI / 180 * (
                        90 + getSpeakerTX()
                    ))
                )
            )
        );
    }

    public double getSpeakerPitch() {
        return (180 / Math.PI * Math.atan(
            (
                Constants.PoseEstimatorSub.speakerTargetHeight
            ) / 
            (
                getSpeakerDistance()
            )
        ));
    }

    public double getSpeakerYaw() {
        double yaw = (getPose().getRotation().getDegrees() - 90 + (180 / Math.PI * Math.acos(
            (
                Math.pow(getCameraSpeakerDistance(), 2) - 
                Math.pow(getSpeakerDistance(), 2) -
                Math.pow(Constants.PoseEstimatorSub.cameraOffset, 2)
            ) / (
                -2 * getCameraSpeakerDistance() * getSpeakerDistance()
            )
        )));

        yaw = getPose().getRotation().getDegrees() - LimelightHelpers.getTX("limelight-shooter");

        if (yaw > 180) return yaw - 360;
        else if (yaw <= -180) return yaw + 360;
        else return yaw;
    }  
    
    public double getTargetYaw() {
        double targetYaw = (180 / Math.PI) * Math.atan( 1.0 *
            (getPose().getY() - getSpeakerPoseYaw().getY()) / 
            (getPose().getX() - getSpeakerPoseYaw().getX())
            );
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            if (targetYaw > 0) targetYaw -= 180;
            else targetYaw += 180;
            return targetYaw;
        } else {
                return targetYaw;
        }
    }

    /*public double getTargetPitch() {
        double bestAngle = 28;
        double currentValue = 420;
        double bestValue = 420;
        for (double i = 28; i < 62; i += .25) {
            currentValue = ((((Constants.ShooterSub.initialVelocity * Math.sin(i * Math.PI / 180) *
                Math.sqrt(
                Math.pow(getPose().getX() - getSpeakerPose().getX(), 2) +
                Math.pow(getPose().getY() - getSpeakerPose().getY(), 2) + .2
                )) / (Constants.ShooterSub.initialVelocity * Math.cos(i * Math.PI / 180) )) + 
                ((-9.8 / 2) *Math.pow(((
                Math.pow(getPose().getX() - getSpeakerPose().getX(), 2) +
                Math.pow(getPose().getY() - getSpeakerPose().getY(), 2) + .2
                ) / (Constants.ShooterSub.initialVelocity * Math.cos(i * Math.PI / 180))), 2))
                ) - Constants.PoseEstimatorSub.speakerTargetHeight);
            if (currentValue < bestValue) {
                bestValue = currentValue;
                bestAngle = i;
            }
        }
        return bestAngle;
    }*/

    // NEW calculations for gravity shot
    /*public double getTargetPitch() {
        double bestAngle = 28;
        double bestDiff = 1000;
        double currDiff = 1000;
        for (int i = 28; i <= 62; i += 2) {
            currDiff = (
                (
                    Constants.ShooterSub.initialVelocity * Math.sin(Math.PI / 180 * i)
                ) / (
                    Constants.ShooterSub.initialVelocity * Math.cos(Math.PI / 180 * i)
                ) * Math.sqrt(
                Math.pow(getPose().getX() - getSpeakerPoseDistance().getX(), 2) +
                Math.pow(getPose().getY() - getSpeakerPoseDistance().getY(), 2) + .2
                ) - (
                    0.5 * 9.8 * Math.pow((
                        (
                            Math.sqrt(
                                Math.pow(getPose().getX() - getSpeakerPoseDistance().getX(), 2) +
                                Math.pow(getPose().getY() - getSpeakerPoseDistance().getY(), 2) + .2
                            )
                        ) / (
                            Constants.ShooterSub.initialVelocity * Math.cos(Math.PI / 180 * i)
                        )
                    ), 2)
                ) - Constants.PoseEstimatorSub.speakerTargetHeight
            );
            if (Math.abs(currDiff) < bestDiff) {
                bestDiff = currDiff;
                bestAngle = i;
            }
        }
        return bestAngle;
    }*/
    
    /*public double getTargetPitch() {
        return (180 / Math.PI) * Math.atan(
            Constants.PoseEstimatorSub.speakerTargetHeight / 
            Math.sqrt(
                Math.pow(getPose().getX() - getSpeakerPoseDistance().getX(), 2) +
                Math.pow(getPose().getY() - getSpeakerPoseDistance().getY(), 2) + .2
            )) + Constants.PoseEstimatorSub.shootkG * Math.sqrt(
                Math.pow(getPose().getX() - getSpeakerPoseDistance().getX(), 2) +
                Math.pow(getPose().getY() - getSpeakerPoseDistance().getY(), 2)
            );
    }*/

    public double getTargetPitch() {
        double speakerDistance = Math.sqrt(
            Math.pow(getPose().getX() - getSpeakerPoseDistance().getX(), 2) +
            Math.pow(getPose().getY() - getSpeakerPoseDistance().getY(), 2) + .2
        );

        double pitch = (180 / Math.PI) * Math.atan(
            Constants.PoseEstimatorSub.speakerTargetHeight / speakerDistance
        );

        if (speakerDistance < 1.5) return pitch + Math.pow(speakerDistance, 2) * Constants.PoseEstimatorSub.closeShootkG;
        else if (speakerDistance < 3) return pitch + Math.pow(speakerDistance, 2) * Constants.PoseEstimatorSub.mediumShootkG;
        else return pitch + Math.pow(speakerDistance, 2) * Constants.PoseEstimatorSub.farShootkG;

        

        /*return (180 / Math.PI) * Math.atan(
            Constants.PoseEstimatorSub.speakerTargetHeight / 
            Math.sqrt(
                Math.pow(getPose().getX() - getSpeakerPoseDistance().getX(), 2) +
                Math.pow(getPose().getY() - getSpeakerPoseDistance().getY(), 2) + .2
            )) + Constants.PoseEstimatorSub.shootkG * (
                Math.pow(getPose().getX() - getSpeakerPoseDistance().getX(), 2) +
                Math.pow(getPose().getY() - getSpeakerPoseDistance().getY(), 2)
            );*/
    }

    public void update() {
        poseEstimator.update(getGyroYaw(), swerveSub.getModulePositions());

        boolean rejectUpdate = false;
        LimelightHelpers.SetRobotOrientation(Constants.PoseEstimatorSub.shooterCamera, 
            getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.PoseEstimatorSub.shooterCamera);
        if (Math.abs(gyro.getRate()) > 720) rejectUpdate = true;
        if (botPose.tagCount == 0) rejectUpdate = true;
        if (rejectUpdate == false) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(getVisionStdDevs(), getVisionStdDevs(), 9999999));
            poseEstimator.addVisionMeasurement(botPose.pose, botPose.timestampSeconds);
        }
    
        /*         // Megatag 1 code RIP
        LimelightHelpers.PoseEstimate limelightBotpose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
        if(limelightBotpose.tagCount >= 2) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(getVisionStdDevs(), getVisionStdDevs(),9999999));
            poseEstimator.addVisionMeasurement(
                limelightBotpose.pose,
                Timer.getFPGATimestamp() -
                    (LimelightHelpers.getLatency_Pipeline("limelight-shooter") / 1000) -
                    (LimelightHelpers.getLatency_Capture("limelight-shooter") / 1000)
            );
                //limelightBotpose.timestampSeconds);
        }*/

        field.setRobotPose(getPose());
    }

    @Override 
    public void periodic() {

        update();
        
        SmartDashboard.putData("Field", field);

        SmartDashboard.putNumber("speaker yaw", getSpeakerYaw());
        SmartDashboard.putNumber("speaker pitch", getSpeakerPitch());
        SmartDashboard.putBoolean("valid speaker", getValidSpeaker());
        SmartDashboard.putBoolean("valid vision", LimelightHelpers.getTV("limelight-shooter"));
        SmartDashboard.putNumber("robotYaw", getPose().getRotation().getDegrees());
        

        //SmartDashboard.putNumber("capture latentcy", LimelightHelpers.getLatency_Capture("limelight-shooter"));
        //SmartDashboard.putNumber("pipeline latentcy", LimelightHelpers.getLatency_Pipeline("limelight-shooter"));


        //SmartDashboard.putNumber("targetDistance", PhotonUtils.getDistanceToPose(getPose(), getSpeakerTargetPose()));
        //SmartDashboard.putNumber("targetPitch", getTargetPitch());
        //SmartDashboard.putNumber("stdDevs", getVisionStdDevs());
        //SmartDashboard.putNumber("targetYaw", getTargetYaw());
        //SmartDashboard.putNumber("heading", getHeading().getDegrees());
        //SmartDashboard.putNumber("gyro heading", getGyroYaw().getDegrees());

        //SmartDashboard.putNumber("poseX", getPose().getTranslation().getX());
        //SmartDashboard.putNumber("poseY", getPose().getTranslation().getY());
        //SmartDashboard.putNumber("poseRotation", getPose().getRotation().getDegrees());
    }

    @Override 
    public void simulationPeriodic() {}
}
