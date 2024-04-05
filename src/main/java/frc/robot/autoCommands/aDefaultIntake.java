package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aDefaultIntake extends Command {
  
    private final IntakeSub intakeSub;
    private final ShooterSub shooterSub;

    private Timer timer;

    private boolean intaked;
    private boolean reset;

    public aDefaultIntake(IntakeSub intakeSub, ShooterSub shooterSub) { 

        this.intakeSub = intakeSub;
        this.shooterSub = shooterSub;

        timer = new Timer();
        timer.stop();
        timer.reset();

        intaked = false;
        reset = false;

        addRequirements(intakeSub, shooterSub);
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsReverse();
        intakeSub.intakeMotorOn();
        timer.start();
    }

    @Override 
    public void execute() {
        if (timer.get() < .3) {
            intakeSub.intakeMotorOn();
        }
        else if (timer.get() > .3) {
            intakeSub.intakeMotorReverse();
            reset = true;
        }
    }

    @Override 
    public void end(boolean interrupted) {
        shooterSub.shooterMotorsOff();
        intakeSub.intakeMotorOff();

        timer.stop();
        timer.reset();

        intaked = false;
        reset = false;
    }

    @Override 
    public boolean isFinished() {
        if (timer.get() > 0.6) return true;
        else return false;
    }
}
