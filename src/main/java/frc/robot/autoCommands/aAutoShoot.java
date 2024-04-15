package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class aAutoShoot extends Command {
  
    private final PoseEstimatorSub poseEstimatorSub;
    private final SwerveSub swerveSub;
    private final ShooterSub shooterSub;
    private final ActuatorSub actuatorSub;
    private final IntakeSub intakeSub;

    Timer cutoffTimer;
    Timer targetTimer;
    Timer shooterTimer;
    Timer intakeTimer;

    boolean onTarget;
    boolean primed;

    public aAutoShoot(PoseEstimatorSub poseEstimatorSub, SwerveSub swerveSub, ShooterSub shooterSub, ActuatorSub actuatorSub, IntakeSub intakeSub) { //Command constructor
    
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;
        this.shooterSub = shooterSub;
        this.actuatorSub = actuatorSub;
        this.intakeSub = intakeSub;

        addRequirements(swerveSub, shooterSub, actuatorSub, intakeSub);

        targetTimer = new Timer();
        targetTimer.stop();
        targetTimer.reset();
        cutoffTimer = new Timer();
        cutoffTimer.stop();
        cutoffTimer.reset();
        shooterTimer = new Timer();
        shooterTimer.stop();
        shooterTimer.reset();
        intakeTimer = new Timer();
        intakeTimer.stop();
        intakeTimer.reset();

        onTarget = false;
        primed = false;
    }

    @Override 
    public void initialize() {
        poseEstimatorSub.setVisionStdDevs(Constants.PoseEstimatorSub.aimVisionStdDevs);
        cutoffTimer.start();
    }

    @Override 
    public void execute() {
        if (intakeSub.getSuckBacked() == true) primed = true;

        if (primed == true) {
            shooterSub.shooterMotorsOn();
            shooterTimer.start();
        }
        
        actuatorSub.setDesiredAngle(poseEstimatorSub.getTargetPitch());

        if (
            swerveSub.driveWithRotationGoal(
                new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
                poseEstimatorSub.getTargetYaw()
                ) == true && 
            actuatorSub.onTarget() == true &&
            poseEstimatorSub.getTargetPitch() > 35 && 
            shooterTimer.get() > .3) {
                targetTimer.start();
        } else {
            targetTimer.stop();
            targetTimer.reset();
        }

        if (targetTimer.get() > .3) onTarget = true;

        if (onTarget == true) intakeSub.intakeMotorOn();

        if (intakeSub.getShooterLineBreaker() == false && primed == true) intakeTimer.start();

        if (cutoffTimer.get() > 2) intakeSub.intakeMotorOn();
    }

    @Override 
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        shooterSub.shooterMotorsOff();
        intakeSub.intakeMotorOff();
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);
        targetTimer.stop();
        targetTimer.reset();
        cutoffTimer.stop();
        cutoffTimer.reset();
        shooterTimer.stop();
        shooterTimer.reset();
        intakeTimer.stop();
        intakeTimer.reset();
        primed = false;
        onTarget = false;
        poseEstimatorSub.setStandardVisionStdDevs();
    }

    @Override 
    public boolean isFinished() {
        if (intakeTimer.get() > .1) return true;
        else if (cutoffTimer.get() > 2.5) return true;
        return false;
    }
}
