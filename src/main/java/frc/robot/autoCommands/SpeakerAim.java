package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class SpeakerAim extends Command {
  
    private final ActuatorSub actuatorSub;

    public SpeakerAim(ActuatorSub actuatorSub) {
        this.actuatorSub = actuatorSub;
    }

    @Override 
    public void initialize() {
        actuatorSub.setDesiredAngle(58);
    }

    @Override
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(50);
    }
}
