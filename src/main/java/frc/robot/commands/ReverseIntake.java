package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ReverseIntake extends Command {
  
    private final IntakeSub intakeSub;

    public ReverseIntake(IntakeSub intakeSub) { 
       
        this.intakeSub = intakeSub;

        addRequirements(intakeSub);
    }

    @Override 
    public void initialize() {
        intakeSub.startIntakeOverride();
        intakeSub.intakeMotorReverse();
    }

    @Override 
    public void execute() {}

    @Override 
    public void end(boolean interrupted) {
        intakeSub.endIntakeOverride();
        intakeSub.intakeMotorOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
