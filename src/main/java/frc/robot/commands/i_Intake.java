package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class i_Intake extends Command {
  
    //Declare subsystems
    private final IntakeSub intakeSub;

    private boolean startState;
    private Timer timer;

    public i_Intake(IntakeSub intakeSub) { //Command constructor
        //Initialize subsystems
        this.intakeSub = intakeSub;

        timer = new Timer();
        timer.stop();
        timer.reset();

        //Add subsystem requirements
        addRequirements(intakeSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        intakeSub.intakeMotorOn();
        startState = intakeSub.getNoteLoaded();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        if (intakeSub.getNoteLoaded() != startState && timer.get() == 0) timer.start();
        if (timer.get() > 0.3) intakeSub.intakeMotorReverse();
    }

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();

        timer.stop();
        timer.reset();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        if (timer.get() > 0.6) return true;
        else return false;
    }
}
