package frc.robot.backupAutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aIntake extends Command {
  
    private final IntakeSub intakeSub;

    private Timer timer;

    public aIntake(IntakeSub intakeSub) { 
       
        this.intakeSub = intakeSub;

        timer = new Timer();
        timer.stop();
        timer.reset();

        addRequirements(intakeSub);
    }

    @Override 
    public void initialize() {
        intakeSub.intakeMotorOn();
        timer.start();
    }

    @Override 
    public void execute() {}

    @Override 
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > .5) return true;
        else return false;
    }
}
