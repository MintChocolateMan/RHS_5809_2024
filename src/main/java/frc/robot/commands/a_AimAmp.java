package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class a_AimAmp extends Command {
  
    //Declare subsystems
    private final ActuatorSub actuatorSub;

    public a_AimAmp(ActuatorSub actuatorSub) { //Command constructor
        //Initialize subsystems
        this.actuatorSub = actuatorSub;

        //Add subsystem requirements
        addRequirements();
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {}

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {}

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.ampAngle);
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return true;
    }
}
