package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aShoot extends Command {
  
    private final ShooterSub shooterSub;

    public aShoot(ShooterSub shooterSub) { 

        this.shooterSub = shooterSub;

        addRequirements(shooterSub);
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsOn();
    }

    @Override 
    public void execute() {}

    @Override 
    public void end(boolean interrupted) {
        shooterSub.shooterMotorsOff();
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
