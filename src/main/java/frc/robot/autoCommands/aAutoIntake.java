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

    public aAutoIntake(IntakeSub intakeSub, SwerveSub swerveSub, PoseEstimatorSub poseEstimatorSub) { 

        this.intakeSub = intakeSub;
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;

        timer = new Timer();
        timer.stop();
        timer.reset();

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
            translation = 1;
        } else if (Math.abs(noteYaw) > Constants.IntakeSub.maxIntakeError) {
            translation = 0.5;
        }

        if (intakeSub.getNoteLoaded() == true) {
            timer.start();
        }

        if (timer.get() != 0) {
            translation = 0;
            strafe = 0;
        }
        
        swerveSub.intakeDrive(new Translation2d(translation, strafe), 0);
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);

        timer.stop();
        timer.reset();

        noteYaw = 0;
    }

    @Override 
    public boolean isFinished() {
        if (timer.get() > .3) return true;
        else return false;
    }
}
