package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aShootPreload extends Command {
  
    private final IntakeSub intakeSub;
    private final ShooterSub shooterSub;

    private Timer timer;

    public aShootPreload(IntakeSub intakeSub, ShooterSub shooterSub) { 

        this.intakeSub = intakeSub;
        this.shooterSub = shooterSub;

        timer = new Timer();
        timer.stop();
        timer.reset();

        addRequirements(intakeSub);
    }

    @Override 
    public void initialize() {
        intakeSub.intakeMotorOff();
        intakeSub.startIntakeOverride();
        shooterSub.shooterMotorsOn();
        timer.start();
    }

    @Override 
    public void execute() {
        if (timer.get() > .3) intakeSub.intakeMotorOn();
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();
        shooterSub.shooterMotorsOff();

        timer.stop();
        timer.reset();
    }

    @Override 
    public boolean isFinished() {
        if (timer.get() > .5) return true;
        else return false;
    }
}
