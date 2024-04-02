package frc.robot.backupAutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class SourceAim extends Command {
  
    private final ActuatorSub actuatorSub;

    public SourceAim(ActuatorSub actuatorSub) {
        this.actuatorSub = actuatorSub;
    }

    @Override 
    public void initialize() {
        actuatorSub.setDesiredAngle(48);
    }

    @Override
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(45);
    }
}
