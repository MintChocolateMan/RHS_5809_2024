package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AimStage extends Command {
  
    private final SwerveSub swerveSub;
    private final ShooterSub shooterSub;
    private final ActuatorSub actuatorSub;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    public AimStage(SwerveSub swerveSub, ShooterSub shooterSub, ActuatorSub actuatorSub, DoubleSupplier translationSup, DoubleSupplier strafeSup) {

        this.swerveSub = swerveSub;
        this.shooterSub = shooterSub;
        this.actuatorSub = actuatorSub;

        addRequirements(swerveSub, shooterSub, actuatorSub);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
    }

    @Override
    public void initialize() {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.stageAngle);
        shooterSub.shooterMotorsOn();
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            swerveSub.driveWithRotationGoal(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), -155);
        } else {
            swerveSub.driveWithRotationGoal(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), -25);
        }
    }

    @Override
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        shooterSub.shooterMotorsOff();
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
