package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aAimLow extends Command {
  
    private final ActuatorSub actuatorSub;
    private final ShooterSub shooterSub;
    private final PoseEstimatorSub poseEstimatorSub;

    public aAimLow(ActuatorSub actuatorSub, ShooterSub shooterSub, PoseEstimatorSub poseEstimatorSub) { 

        this.actuatorSub = actuatorSub;
        this.shooterSub = shooterSub;
        this.poseEstimatorSub = poseEstimatorSub;

        addRequirements(shooterSub);
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsOn();
    }

    @Override 
    public void execute() {
        actuatorSub.setDesiredAngle(poseEstimatorSub.getTargetPitch() - 1);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSub.shooterMotorsOff();
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
