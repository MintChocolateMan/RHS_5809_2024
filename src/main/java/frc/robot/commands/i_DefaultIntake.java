package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class i_DefaultIntake extends Command {
  
    //Declare subsystems
    private final IntakeSub intakeSub;

    public i_DefaultIntake(IntakeSub intakeSub) { //Command constructor
        //Initialize subsystems
        this.intakeSub = intakeSub;

        //Add subsystem requirements
        addRequirements(intakeSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        intakeSub.resetIntakeMotorPosition();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        intakeSub.intakeMotorToPID();
    }

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {}

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
