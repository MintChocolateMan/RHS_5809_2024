package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class LRNoteAim extends Command {
  
    private final ActuatorSub actuatorSub;

    public LRNoteAim(ActuatorSub actuatorSub) {
        this.actuatorSub = actuatorSub;
    }

    @Override 
    public void initialize() {
        actuatorSub.setDesiredAngle(40);
    }

    @Override
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(40);
    }
}
