package frc.robot.backups.backupAutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aAimCenter extends Command {
  
    private final ActuatorSub actuatorSub;
    private final ShooterSub shooterSub;

    public aAimCenter(ActuatorSub actuatorSub, ShooterSub shooterSub) { 

        this.actuatorSub = actuatorSub;
        this.shooterSub = shooterSub;

        addRequirements(shooterSub);
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsOn();
    }

    @Override 
    public void execute() {
        actuatorSub.setDesiredAngle(41);
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
