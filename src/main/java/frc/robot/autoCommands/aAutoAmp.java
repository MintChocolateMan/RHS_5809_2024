package frc.robot.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class aAutoAmp extends Command {
  
    private final PoseEstimatorSub poseEstimatorSub;
    private final SwerveSub swerveSub;
    private final ShooterSub shooterSub;
    private final ActuatorSub actuatorSub;
    private final IntakeSub intakeSub;

    Timer shooterTimer;
    Timer intakeTimer;
    Timer targetTimer;

    public aAutoAmp(PoseEstimatorSub poseEstimatorSub, SwerveSub swerveSub, ShooterSub shooterSub, ActuatorSub actuatorSub, IntakeSub intakeSub) {

        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;
        this.shooterSub = shooterSub;
        this.actuatorSub = actuatorSub;
        this.intakeSub = intakeSub;

        addRequirements(swerveSub, shooterSub, actuatorSub, intakeSub);

        shooterTimer = new Timer();
        shooterTimer.stop();
        shooterTimer.reset();
        intakeTimer = new Timer();
        intakeTimer.stop();
        intakeTimer.reset();
        targetTimer = new Timer();
        targetTimer.stop();
        targetTimer.reset();
    }

    @Override
    public void initialize() {
        actuatorSub.setDesiredAngle(62);
        shooterSub.shooterMotorsAmp();
        shooterTimer.start();
    }

    @Override
    public void execute() {
        if (actuatorSub.onTarget() == true &&
            swerveSub.ampDrive() == true &&
            shooterTimer.get() > .3
        ) targetTimer.start();
        else {
            targetTimer.stop();
            targetTimer.reset();
        }

        if (targetTimer.get() > 0.1) intakeTimer.start();

        if (intakeTimer.get() != 0) intakeSub.intakeMotorOn();
        else if (intakeTimer.get() == 0) intakeSub.intakeMotorToPID();
    }

    @Override
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        shooterSub.shooterMotorsOff();
        intakeSub.intakeMotorOff();
        swerveSub.drive(new Translation2d(0, 0), 0, false, false);

        if (interrupted == false) {
            poseEstimatorSub.setPoseTranslation(poseEstimatorSub.getAmpPose().getTranslation());
        }

        shooterTimer.stop();
        shooterTimer.reset();
        intakeTimer.stop();
        intakeTimer.reset();
        targetTimer.stop();
        targetTimer.reset();
    }

    @Override
    public boolean isFinished() {
        if (intakeTimer.hasElapsed(.5)) return true;
        return false;
    }
}
