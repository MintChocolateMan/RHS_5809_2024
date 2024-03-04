package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class p_ExtendClimbers extends Command {
  
    //Declare subsystems
    private final PneumaticSub pneumaticSub;

    public p_ExtendClimbers(PneumaticSub pneumaticSub) { //Command constructor
        //Initialize subsystems
        this.pneumaticSub = pneumaticSub;

        //Add subsystem requirements
        addRequirements(pneumaticSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        pneumaticSub.climbersUp();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {}

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        pneumaticSub.climbersDown();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}