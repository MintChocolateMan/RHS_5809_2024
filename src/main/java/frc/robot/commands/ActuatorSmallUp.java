package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ActuatorSmallUp extends Command {
  
    private final ActuatorSub actuatorSub;

    private double curAngle;

    public ActuatorSmallUp(ActuatorSub actuatorSub) { 

        this.actuatorSub = actuatorSub;

        addRequirements(actuatorSub);
    }

    @Override 
    public void initialize() {
        curAngle = actuatorSub.getAngleFromMotorPosition(actuatorSub.getMotorPosition());
        actuatorSub.setMotorPosition(actuatorSub.getMotorPositionFromAngle(curAngle - 1));
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
