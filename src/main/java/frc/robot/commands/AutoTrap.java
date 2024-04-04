package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoTrap extends Command {
  
    private final PoseEstimatorSub poseEstimatorSub;
    private final SwerveSub swerveSub;
    private final ShooterSub shooterSub;
    private final ActuatorSub actuatorSub;
    private final IntakeSub intakeSub;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    private double startYaw;
    private double targetYaw;

    private boolean trapSeen;

    Timer shooterTimer;
    Timer intakeTimer;
    Timer targetTimer;

    public AutoTrap(PoseEstimatorSub poseEstimatorSub, SwerveSub swerveSub, ShooterSub shooterSub, ActuatorSub actuatorSub, IntakeSub intakeSub, DoubleSupplier translationSup, DoubleSupplier strafeSup) { //Command constructor

        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;
        this.shooterSub = shooterSub;
        this.actuatorSub = actuatorSub;
        this.intakeSub = intakeSub;

        addRequirements(swerveSub, shooterSub, actuatorSub, intakeSub);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;

        trapSeen = false;

        shooterTimer = new Timer();
        shooterTimer.stop();
        shooterTimer.reset();
        intakeTimer = new Timer();
        intakeTimer.stop();
        intakeTimer.reset();
        targetTimer = new Timer();
        targetTimer.stop();
        targetTimer.reset();
    }

    @Override
    public void initialize() {
        actuatorSub.setDesiredAngle(62);
        shooterSub.shooterMotorsOn();
        shooterTimer.start();

        startYaw = poseEstimatorSub.getPose().getRotation().getDegrees();
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() == true && alliance.get() == DriverStation.Alliance.Blue) {
            if (startYaw >= -60 && startYaw <= 60) {
                targetYaw = 0;
            } else if (startYaw >= 60) {
                targetYaw = 120;
            } else if (startYaw <= -60) {
                targetYaw = -120;
            }
        } else {
            if (startYaw >= 0 && startYaw <= 120) {
                targetYaw = 60;
            } else if (startYaw <= 0 && startYaw >= -120) {
                targetYaw = -60;
            } else if (startYaw <= -120 || startYaw >= 120) {
                targetYaw = 180;
            }
        }
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        if (trapSeen == false) {
            swerveSub.driveWithRotationGoal(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), targetYaw);
        } else {
            if (actuatorSub.onTarget() == true &&
                swerveSub.ampDrive() == true &&
                shooterTimer.get() > .3
            ) targetTimer.start();
            else {
                targetTimer.stop();
                targetTimer.reset();
            }
        }

        if (poseEstimatorSub.getValidAmp() == true) trapSeen = true;

        if (targetTimer.get() > 0.3) intakeTimer.start();

        if (intakeTimer.get() != 0) intakeSub.intakeMotorOn();
        else if (intakeTimer.get() == 0) intakeSub.intakeMotorToPID();
    }

    @Override
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        shooterSub.shooterMotorsOff();
        intakeSub.intakeMotorOff();
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);

        if (interrupted == false) {
            poseEstimatorSub.setPoseTranslation(poseEstimatorSub.getAmpPose().getTranslation());
        }

        shooterTimer.stop();
        shooterTimer.reset();
        intakeTimer.stop();
        intakeTimer.reset();
        targetTimer.stop();
        targetTimer.reset();

        trapSeen = false;
    }

    @Override
    public boolean isFinished() {
        if (intakeTimer.hasElapsed(.5)) return true;
        return false;
    }
}
