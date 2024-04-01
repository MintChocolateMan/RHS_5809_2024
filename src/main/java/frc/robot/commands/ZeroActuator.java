package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ZeroActuator extends Command {
  
    private final ActuatorSub actuatorSub;

    public ZeroActuator(ActuatorSub actuatorSub) { //Command constructor
 
        this.actuatorSub = actuatorSub;

        addRequirements(actuatorSub);
    }

    @Override 
    public void initialize() {
        actuatorSub.setZeroing(true);
        actuatorSub.actuatorMotorDown();

    }

    @Override 
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        actuatorSub.actuatorMotorOff();
        actuatorSub.setMotorPosition(0);
        actuatorSub.setDesiredAngle(0);
        actuatorSub.setZeroing(false);
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
