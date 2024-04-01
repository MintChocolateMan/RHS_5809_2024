package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoShoot extends Command {
  
    private final PoseEstimatorSub poseEstimatorSub;
    private final SwerveSub swerveSub;
    private final ShooterSub shooterSub;
    private final ActuatorSub actuatorSub;
    private final IntakeSub intakeSub;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    Timer shooterTimer;
    Timer intakeTimer;

    public AutoShoot(PoseEstimatorSub poseEstimatorSub, SwerveSub swerveSub, ShooterSub shooterSub, ActuatorSub actuatorSub, IntakeSub intakeSub, DoubleSupplier translationSup, DoubleSupplier strafeSup) { //Command constructor
    
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;
        this.shooterSub = shooterSub;
        this.actuatorSub = actuatorSub;
        this.intakeSub = intakeSub;

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

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsOn();
        shooterTimer.start();
    }

    @Override 
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        
        actuatorSub.setDesiredAngle(poseEstimatorSub.getTargetPitch());

        if (
            swerveSub.driveWithRotationGoal(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                poseEstimatorSub.getTargetYaw()
                ) == true && 
            actuatorSub.onTarget() == true &&
            shooterTimer.hasElapsed(.5)) {
                intakeTimer.start();
            }

        if (intakeTimer.get() != 0) intakeSub.intakeMotorOn();
    }

    @Override 
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        shooterSub.shooterMotorsOff();
        intakeSub.intakeMotorOff();
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);
        shooterTimer.stop();
        shooterTimer.reset();
        intakeTimer.stop();
        intakeTimer.reset();
    }

    @Override 
    public boolean isFinished() {
        if (intakeTimer.hasElapsed(.5)) return true;
        return false;
    }
}
