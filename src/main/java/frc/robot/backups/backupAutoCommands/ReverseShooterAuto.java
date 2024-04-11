package frc.robot.backups.backupAutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ReverseShooterAuto extends Command {
  
    private final ShooterSub shooterSub;

    public ReverseShooterAuto(ShooterSub shooterSub) {
        this.shooterSub = shooterSub;
        addRequirements(shooterSub);
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsReverse();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSub.shooterMotorsOff();
    }
}
