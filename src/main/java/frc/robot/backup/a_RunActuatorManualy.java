package frc.robot.backup;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class a_RunActuatorManualy extends Command {
  
    //Declare subsystems
    private final ActuatorSub ActuatorSub;

    public a_RunActuatorManualy(ActuatorSub ActuatorSub) { //Command constructor
        //Initialize subsystems
        this.ActuatorSub = ActuatorSub;

        //Add subsystem requirements
        addRequirements(ActuatorSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        ActuatorSub.actuatorMotorOn();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {}

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        ActuatorSub.actuatorMotorOff();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
