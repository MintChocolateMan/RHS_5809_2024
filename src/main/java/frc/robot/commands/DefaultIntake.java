package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class DefaultIntake extends Command {
  
    //Declare subsystems
    private final IntakeSub intakeSub;

    private boolean startState;
    private Timer timer;

    public DefaultIntake(IntakeSub intakeSub) { //Command constructor
        //Initialize subsystems
        this.intakeSub = intakeSub;

        //Add subsystem requirements
        addRequirements(intakeSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {}

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        if (intakeSub.getNoteLoaded() == true) {
            intakeSub.intakeMotorReverse();
        } else intakeSub.intakeMotorOff();
    }

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
