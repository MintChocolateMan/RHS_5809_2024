package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoAmp extends Command {
  
    //Declare subsystems
    private final PoseEstimatorSub poseEstimatorSub;
    private final SwerveSub swerveSub;
    private final ShooterSub shooterSub;
    private final ActuatorSub actuatorSub;
    private final IntakeSub intakeSub;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    Timer shooterTimer;
    Timer intakeTimer;

    public AutoAmp(PoseEstimatorSub poseEstimatorSub, SwerveSub swerveSub, ShooterSub shooterSub, ActuatorSub actuatorSub, IntakeSub intakeSub, DoubleSupplier translationSup, DoubleSupplier strafeSup) { //Command constructor
        //Initialize subsystems
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;
        this.shooterSub = shooterSub;
        this.actuatorSub = actuatorSub;
        this.intakeSub = intakeSub;

        //Add subsystem requirements
        addRequirements(swerveSub, shooterSub, actuatorSub, intakeSub);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;

        shooterTimer = new Timer();
        shooterTimer.stop();
        shooterTimer.reset();
        intakeTimer = new Timer();
        intakeTimer.stop();
        intakeTimer.reset();
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        actuatorSub.setDesiredAngle(62);
        shooterSub.shooterMotorsAmp();
        shooterTimer.start();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        if (poseEstimatorSub.getValidAmp() == true) {
            if (actuatorSub.onTarget() == true &&
                swerveSub.ampDrive() == true &&
                shooterTimer.get() > .3
            ) intakeTimer.start();
        } else swerveSub.driveWithRotationGoal(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 90
        );

        if (intakeTimer.get() != 0) intakeSub.intakeMotorOn();
    }

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        shooterSub.shooterMotorsOff();
        intakeSub.intakeMotorOff();
        shooterTimer.stop();
        shooterTimer.reset();
        intakeTimer.stop();
        intakeTimer.reset();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        if (intakeTimer.hasElapsed(.5)) return true;
        return false;
    }
}
