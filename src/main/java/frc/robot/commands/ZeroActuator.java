package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ZeroActuator extends Command {
  
    //Declare subsystems
    private final ActuatorSub actuatorSub;

    public ZeroActuator(ActuatorSub actuatorSub) { //Command constructor
        //Initialize subsystems
        this.actuatorSub = actuatorSub;

        //Add subsystem requirements
        addRequirements(actuatorSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        actuatorSub.setZeroing(true);
        actuatorSub.actuatorMotorDown();

    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {}

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        actuatorSub.actuatorMotorOff();
        actuatorSub.setMotorPosition(0);
        actuatorSub.setDesiredAngle(0);
        actuatorSub.setZeroing(false);
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
