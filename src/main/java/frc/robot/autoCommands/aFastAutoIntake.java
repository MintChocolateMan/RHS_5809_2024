package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class aFastAutoIntake extends Command {
  
    private final IntakeSub intakeSub;
    private final SwerveSub swerveSub;
    private final PoseEstimatorSub poseEstimatorSub;

    double translation;
    double strafe;

    private double noteYaw;

    public aFastAutoIntake(IntakeSub intakeSub, SwerveSub swerveSub, PoseEstimatorSub poseEstimatorSub) { 

        this.intakeSub = intakeSub;
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;

        noteYaw = 0;

        addRequirements(intakeSub, swerveSub);
    }

    @Override
    public void initialize() {
        intakeSub.intakeMotorOn();
    }

    @Override 
    public void execute() {

        if (poseEstimatorSub.getValidNote() == true) {
            noteYaw = poseEstimatorSub.getNoteYaw();
            strafe = swerveSub.swerveStrafePID.calculate(noteYaw, 0);
        }
        if (Math.abs(noteYaw) <= Constants.IntakeSub.maxIntakeError) {
            translation = 2;
        } else if (Math.abs(noteYaw) > Constants.IntakeSub.maxIntakeError) {
            translation = 1;
        }
        
        swerveSub.intakeDrive(new Translation2d(translation, strafe), 0);
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();

        noteYaw = 0;
    }

    @Override 
    public boolean isFinished() {
        if (intakeSub.getNoteLoaded() == true) return true;
        else return false;
    }
}
