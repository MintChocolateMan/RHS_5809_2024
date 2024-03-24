package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
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

    private boolean noteSeen = false;
    private double noteYaw = 0;
    private boolean startState;
    private Timer timer;

    public AutoIntake(IntakeSub intakeSub, SwerveSub swerveSub, PoseEstimatorSub poseEstimatorSub, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) { //Command constructor
        //Initialize subsystems
        this.intakeSub = intakeSub;
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        timer = new Timer();
        timer.stop();
        timer.reset();

        //Add subsystem requirements
        addRequirements(intakeSub, swerveSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        startState = intakeSub.getNoteLoaded();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        noteYaw = poseEstimatorSub.getNoteYaw();
        if (noteYaw != 0) noteSeen = true;

        if (noteSeen == false) {
            double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
            double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

            swerveSub.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                false,
                true
            );
        } else {
            swerveSub.intakeDrive(new Translation2d(0.2, 0), noteYaw);
        }
        
        if (intakeSub.getNoteLoaded() != startState && timer.get() == 0) timer.start();
        if (timer.get() > 0.3) intakeSub.intakeMotorReverse();
    }

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();

        timer.stop();
        timer.reset();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        if (timer.get() > 0.6) return true;
        else return false;
    }
}
