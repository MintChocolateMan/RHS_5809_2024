package frc.robot.backupAutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ShootAuto extends Command {
  
    private final ShooterSub shooterSub;

    public ShootAuto(ShooterSub shooterSub) {
        this.shooterSub = shooterSub;
        addRequirements(shooterSub);
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsOn();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSub.shooterMotorsOff();
    }
}
