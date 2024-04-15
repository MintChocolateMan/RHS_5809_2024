package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aAutoIntakeShoot extends Command {
  
    private final IntakeSub intakeSub;
    private final SwerveSub swerveSub;
    private final PoseEstimatorSub poseEstimatorSub;

    double translation;
    double strafe;

    private double noteYaw;

    private boolean wasEmpty = true;

    private Timer intakeTimer;

    public aAutoIntakeShoot(IntakeSub intakeSub, SwerveSub swerveSub, PoseEstimatorSub poseEstimatorSub) { 

        this.intakeSub = intakeSub;
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;

        noteYaw = 0;

        wasEmpty = false;

        intakeTimer = new Timer();
        intakeTimer.stop();
        intakeTimer.reset();

        addRequirements(intakeSub, swerveSub);
    }

    @Override
    public void initialize() {
        intakeSub.intakeMotorOn();
    }

    @Override 
    public void execute() {

        if (intakeSub.getShooterLineBreaker() == false) {
            wasEmpty = true;
        }

        if (intakeSub.getIntakeLineBreaker() == true) intakeTimer.start();

        if (poseEstimatorSub.getValidNote() == true) {
            noteYaw = poseEstimatorSub.getNoteYaw();
            strafe = swerveSub.swerveStrafePID.calculate(noteYaw, 0);
        }

        if (intakeTimer.get() != 0) strafe = 0;
        
        swerveSub.intakeDrive(new Translation2d(1.5, strafe), 0);
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);

        noteYaw = 0;

        wasEmpty = false;

        intakeTimer.stop();
        intakeTimer.reset();
    }

    @Override 
    public boolean isFinished() {
        if (intakeTimer.get() > .4 && wasEmpty == true) return true;
        else return false;
    }
}
