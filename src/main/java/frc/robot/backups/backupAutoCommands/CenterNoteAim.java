package frc.robot.backups.backupAutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class CenterNoteAim extends Command {
  
    private final ActuatorSub actuatorSub;

    public CenterNoteAim(ActuatorSub actuatorSub) {
        this.actuatorSub = actuatorSub;
    }

    @Override 
    public void initialize() {
        actuatorSub.setDesiredAngle(42);
    }

    @Override
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(40);
    }
}
