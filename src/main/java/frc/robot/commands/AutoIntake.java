package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoIntake extends Command {
  
    private final IntakeSub intakeSub;
    private final SwerveSub swerveSub;
    private final ActuatorSub actuatorSub;
    private final PoseEstimatorSub poseEstimatorSub;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    double translation;
    double rotation;
    double strafe;

    private boolean noteSeen;
    private double noteYaw;

    public AutoIntake(IntakeSub intakeSub, SwerveSub swerveSub, ActuatorSub actuatorSub, PoseEstimatorSub poseEstimatorSub, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) { //Command constructor

        this.intakeSub = intakeSub;
        this.swerveSub = swerveSub;
        this.actuatorSub = actuatorSub;
        this.poseEstimatorSub = poseEstimatorSub;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        noteSeen = false;
        noteYaw = 0;

        addRequirements(intakeSub, swerveSub);
    }

    @Override
    public void initialize() {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        intakeSub.intakeMotorOn();

        noteSeen = false;
    }

    @Override 
    public void execute() {
        if (poseEstimatorSub.getValidNote() == true) {
            noteSeen = true;
        }

        if (noteSeen == false) {
            double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
            double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

            swerveSub.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                true,
                true
            );
        } else {
            if (poseEstimatorSub.getValidNote() == true) {
                noteYaw = poseEstimatorSub.getNoteYaw();
                rotation = swerveSub.swerveRotationPID.calculate(poseEstimatorSub.getPose().getRotation().getDegrees(), 
                    poseEstimatorSub.getPose().getRotation().getDegrees() - noteYaw);
                strafe = swerveSub.swerveStrafePID.calculate(noteYaw / 4, 0);
            }
            if (Math.abs(noteYaw) <= Constants.IntakeSub.maxIntakeError) {
                translation = 2.5;
            } else if (Math.abs(noteYaw) > Constants.IntakeSub.maxIntakeError) {
                translation = 1;
            }
            
            swerveSub.intakeDrive(new Translation2d(translation, strafe), rotation);
        }
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);

        noteSeen = false;
        noteYaw = 0;
    }

    @Override 
    public boolean isFinished() {
        if (intakeSub.getNoteLoaded() == true) return true;
        else return false;
    }
}
