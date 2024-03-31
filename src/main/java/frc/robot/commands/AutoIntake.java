package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoIntake extends Command {
  
    //Declare subsystems
    private final IntakeSub intakeSub;
    private final SwerveSub swerveSub;
    private final PoseEstimatorSub poseEstimatorSub;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    double translation;
    double rotation;

    private boolean noteSeen;
    private double noteYaw;

    public AutoIntake(IntakeSub intakeSub, SwerveSub swerveSub, PoseEstimatorSub poseEstimatorSub, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) { //Command constructor

        this.intakeSub = intakeSub;
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        noteSeen = false;
        noteYaw = 0;

        addRequirements(intakeSub, swerveSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        intakeSub.intakeMotorOn();

        noteSeen = false;
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
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
            if (Math.abs(poseEstimatorSub.getPose().getRotation().getDegrees() - rotation) <= Constants.IntakeSub.maxIntakeError) {
                translation = 1.5;
            } else if (Math.abs(poseEstimatorSub.getPose().getRotation().getDegrees() - rotation) > Constants.IntakeSub.maxIntakeError) {
                translation = 0.5;
            }
            if (poseEstimatorSub.getValidNote() == true) {
                noteYaw = poseEstimatorSub.getNoteYaw();
                rotation = poseEstimatorSub.getPose().getRotation().getDegrees() - noteYaw;
            }

            swerveSub.intakeDrive(new Translation2d(translation, 0), rotation);
        }
    }

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);

        noteSeen = false;
        noteYaw = 0;
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        if (intakeSub.getNoteLoaded() == true) return true;
        else return false;
    }
}
