package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aDefaultIntake extends Command {
  
    private final IntakeSub intakeSub;

    private Timer timer;

    private boolean intaked;
    private boolean reset;

    public aDefaultIntake(IntakeSub intakeSub) { 

        this.intakeSub = intakeSub;

        timer = new Timer();
        timer.stop();
        timer.reset();

        intaked = false;
        reset = false;

        addRequirements(intakeSub);
    }

    @Override 
    public void initialize() {
        intakeSub.intakeMotorOn();
        timer.start();
    }

    @Override 
    public void execute() {
        if (timer.get() > .2) {
            intaked = true;
        }
        if (intaked == true && reset == false) {
            intakeSub.resetIntakeMotorPosition();
            reset = true;
        }
        if (intaked == true && reset == true) {
            intakeSub.intakeMotorToPID();
        }
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();

        timer.stop();
        timer.reset();

        intaked = false;
        reset = false;
    }

    @Override 
    public boolean isFinished() {
        if (timer.get() > 0.4) return true;
        else return false;
    }
}
