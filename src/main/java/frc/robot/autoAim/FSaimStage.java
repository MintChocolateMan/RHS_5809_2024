package frc.robot.autoAim;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class FSaimStage extends Command {
  
    private final ActuatorSub actuatorSub;

    public FSaimStage(ActuatorSub actuatorSub) { 

        this.actuatorSub = actuatorSub;

        addRequirements();
    }

    @Override 
    public void initialize() {
    }

    @Override 
    public void execute() {
        actuatorSub.setDesiredAngle(45);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override 
    public boolean isFinished() {
        return true;
    }
}
