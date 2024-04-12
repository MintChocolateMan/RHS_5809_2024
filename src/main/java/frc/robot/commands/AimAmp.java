package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AimAmp extends Command {
  
    private final ActuatorSub actuatorSub;
    private final ShooterSub shooterSub;

    public AimAmp(ActuatorSub actuatorSub, ShooterSub shooterSub) { 

        this.actuatorSub = actuatorSub;
        this.shooterSub = shooterSub;

        addRequirements(shooterSub);
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsAmp();
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.ampAngle);
    }

    @Override 
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        shooterSub.shooterMotorsOff();
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
