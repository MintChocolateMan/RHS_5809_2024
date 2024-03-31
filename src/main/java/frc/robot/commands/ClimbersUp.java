package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ClimbersUp extends Command {
  
    private final PneumaticSub pneumaticSub;

    public ClimbersUp(PneumaticSub pneumaticSub) { 
      
        this.pneumaticSub = pneumaticSub;

        addRequirements(pneumaticSub);
    }

    @Override
    public void initialize() {
        pneumaticSub.climbersUp();
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
