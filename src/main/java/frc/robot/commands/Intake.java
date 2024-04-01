package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class Intake extends Command {
  
    private final IntakeSub intakeSub;

    public Intake(IntakeSub intakeSub) { 
       
        this.intakeSub = intakeSub;

        addRequirements(intakeSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        intakeSub.intakeMotorOn();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {}

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
