package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.LimelightHelpers.LimelightResults;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class VisionShoot extends Command {
  
    private final PoseEstimatorSub poseEstimatorSub;
    private final SwerveSub swerveSub;
    private final ShooterSub shooterSub;
    private final ActuatorSub actuatorSub;
    private final IntakeSub intakeSub;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    Timer intakeTimer;
    Timer targetTimer;

    double pitch;
    double yaw;

    boolean speakerSeen = false;

    public VisionShoot(PoseEstimatorSub poseEstimatorSub, SwerveSub swerveSub, ShooterSub shooterSub, ActuatorSub actuatorSub, IntakeSub intakeSub, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;
        this.shooterSub = shooterSub;
        this.actuatorSub = actuatorSub;
        this.intakeSub = intakeSub;

        addRequirements(swerveSub, shooterSub, actuatorSub, intakeSub);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        intakeTimer = new Timer();
        intakeTimer.stop();
        intakeTimer.reset();
        targetTimer = new Timer();
        targetTimer.stop();
        targetTimer.reset();

        speakerSeen = false;
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsOn();
        poseEstimatorSub.setVisionStdDevs(Constants.PoseEstimatorSub.aimVisionStdDevs);
    }

    @Override 
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if (LimelightHelpers.getTV("limelight-shooter") == true) {
            pitch = poseEstimatorSub.getSpeakerPitch();
            yaw = poseEstimatorSub.getSpeakerYaw();
            yaw = LimelightHelpers.getTX("limelight-shooter") * .5;
            speakerSeen = true;
        }   
        
        actuatorSub.setDesiredAngle(pitch);


        if (speakerSeen == false) {
            swerveSub.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), rotationVal, true, false);
        } else if (
            /*swerveSub.driveWithRotationGoal(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                yaw
                ) == true && */
            swerveSub.driveAndAim(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), yaw
                ) == true &&
            actuatorSub.onTarget() == true &&
            poseEstimatorSub.getSpeakerPitch() > 35) {
                targetTimer.start();
        } else {
            targetTimer.stop();
            targetTimer.reset();
        }

        if (targetTimer.get() > .2) intakeTimer.start();

        if (intakeTimer.get() != 0) intakeSub.intakeMotorOn();
    }

    @Override 
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        shooterSub.shooterMotorsOff();
        intakeSub.intakeMotorOff();
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);
        intakeTimer.stop();
        intakeTimer.reset();
        poseEstimatorSub.setStandardVisionStdDevs();
        speakerSeen = false;
    }

    @Override 
    public boolean isFinished() {
        if (intakeTimer.hasElapsed(.4)) return true;
        return false;
    }
}
