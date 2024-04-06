package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aAimOutside extends Command {
  
    private final ActuatorSub actuatorSub;
    private final ShooterSub shooterSub;

    public aAimOutside(ActuatorSub actuatorSub, ShooterSub shooterSub) { 

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
        actuatorSub.setDesiredAngle(40);
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
