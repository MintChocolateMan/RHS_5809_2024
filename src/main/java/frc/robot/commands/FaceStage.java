package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class FaceStage extends Command {
  
    private final SwerveSub swerveSub;
    private final PoseEstimatorSub poseEstimatorSub;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    private double startYaw;
    private double targetYaw;

    public FaceStage(SwerveSub swerveSub, PoseEstimatorSub poseEstimatorSub, DoubleSupplier translationSup, DoubleSupplier strafeSup) { 

        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;

        addRequirements(swerveSub);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
    }

    @Override
    public void initialize() {
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

        swerveSub.driveWithRotationGoal(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), targetYaw);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
