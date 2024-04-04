package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aInitActuator extends Command {
  
    private final ActuatorSub actuatorSub;

    public aInitActuator(ActuatorSub actuatorSub) { 
   
        this.actuatorSub = actuatorSub;

        addRequirements(actuatorSub);
    }

    @Override 
    public void initialize() {
        actuatorSub.initActuator();
        actuatorSub.resetActuatorController();
    }

    @Override 
    public void execute() {}

    @Override 
    public void end(boolean interrupted) {}

    @Override 
    public boolean isFinished() {
        return true;
    }
}
