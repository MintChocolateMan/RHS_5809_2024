package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ShootAmp extends Command {
  
    private final IntakeSub intakeSub;
    private final ActuatorSub actuatorSub;

    private Timer timer;

    public ShootAmp(IntakeSub intakeSub, ActuatorSub actuatorSub) { 
       
        this.intakeSub = intakeSub;
        this.actuatorSub = actuatorSub;

        addRequirements(intakeSub);

        timer = new Timer();
        timer.stop();
        timer.reset();
    }

    @Override 
    public void initialize() {
        intakeSub.intakeMotorOn();
    }

    @Override 
    public void execute() {
        if (actuatorSub.onTarget() == true) timer.start();

        if (timer.get() != 0) intakeSub.intakeMotorOn();
    }

    @Override 
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > .3) return true;
        else return false;
    }
}
