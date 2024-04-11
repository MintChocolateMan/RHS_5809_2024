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

    boolean onTarget;

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

        onTarget = false;
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsOn();
        poseEstimatorSub.setVisionStdDevs(Constants.PoseEstimatorSub.aimVisionStdDevs);
        cutoffTimer.start();
    }

    @Override 
    public void execute() {
        
        actuatorSub.setDesiredAngle(poseEstimatorSub.getTargetPitch());

        if (
            swerveSub.driveWithRotationGoal(
                new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
                poseEstimatorSub.getTargetYaw()
                ) == true && 
            actuatorSub.onTarget() == true &&
            poseEstimatorSub.getTargetPitch() > 35) {
                targetTimer.start();
        } else {
            targetTimer.stop();
            targetTimer.reset();
        }

        if (targetTimer.get() > .3) onTarget = true;

        if (onTarget == true) intakeSub.intakeMotorOn();

        if (cutoffTimer.get() > 1.7) intakeSub.intakeMotorOn();
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
        poseEstimatorSub.setStandardVisionStdDevs();
    }

    @Override 
    public boolean isFinished() {
        if (intakeSub.getShooterLineBreaker() == false || cutoffTimer.get() > 2) return true;
        return false;
    }
}
