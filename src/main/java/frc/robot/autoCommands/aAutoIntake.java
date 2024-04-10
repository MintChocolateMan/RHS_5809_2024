package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class aAutoIntake extends Command {
  
    private final IntakeSub intakeSub;
    private final SwerveSub swerveSub;
    private final PoseEstimatorSub poseEstimatorSub;

    double translation;
    double strafe;

    private Timer timer;

    private double noteYaw;

    private boolean wasEmpty = true;

    public aAutoIntake(IntakeSub intakeSub, SwerveSub swerveSub, PoseEstimatorSub poseEstimatorSub) { 

        this.intakeSub = intakeSub;
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;

        timer = new Timer();
        timer.stop();
        timer.reset();

        noteYaw = 0;

        wasEmpty = false;

        addRequirements(intakeSub, swerveSub);
    }

    @Override
    public void initialize() {
        intakeSub.intakeMotorOn();
    }

    @Override 
    public void execute() {

        if (intakeSub.getNoteLoaded() == false) {
            wasEmpty = true;
        }

        if (poseEstimatorSub.getValidNote() == true) {
            noteYaw = poseEstimatorSub.getNoteYaw();
            strafe = swerveSub.swerveStrafePID.calculate(noteYaw, 0);
        }

        if (Math.abs(noteYaw) <= Constants.IntakeSub.maxIntakeError && poseEstimatorSub.getNoteTY() + 3 >= 0) {
            translation = 1.5; // THIS WAS 1.5
        } else if (Math.abs(noteYaw) > Constants.IntakeSub.maxIntakeError || poseEstimatorSub.getNoteTY() + 3 < 0) {
            translation = 1.5; //THIS WAS 0.75
        }

        if (intakeSub.getNoteLoaded() == true && wasEmpty == true) {
            timer.start();
        }

        if (timer.get() != 0) {
            translation = 1;
            strafe = 0;
        }
        
        swerveSub.intakeDrive(new Translation2d(translation, strafe), 0);
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSub.endIntakeOverride();
        intakeSub.intakeMotorOff();
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);

        timer.stop();
        timer.reset();

        noteYaw = 0;

        wasEmpty = false;
    }

    @Override 
    public boolean isFinished() {
        if (timer.get() > .2) return true;
        else return false;
    }
}
