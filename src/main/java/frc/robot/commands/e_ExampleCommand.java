package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class e_ExampleCommand extends Command {
  
    //Declare subsystems
    private final ExampleSub exampleSub;

    public e_ExampleCommand(ExampleSub exampleSub) { //Command constructor
        //Initialize subsystems
        this.exampleSub = exampleSub;

        //Add subsystem requirements
        addRequirements(exampleSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        exampleSub.exampleMethod();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {}

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {}

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
