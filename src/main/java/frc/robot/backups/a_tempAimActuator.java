package frc.robot.backups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class a_tempAimActuator extends Command {
  
    //Declare subsystems
    private final ActuatorSub actuatorSub;

    public a_tempAimActuator(ActuatorSub actuatorSub) { //Command constructor
        //Initialize subsystems
        this.actuatorSub = actuatorSub;

        //Add subsystem requirements
        addRequirements(actuatorSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        actuatorSub.setDesiredAngle(60);
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {}

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(40);
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
