package frc.robot.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aShootClose extends Command {
  
    private final ActuatorSub actuatorSub;
    private final ShooterSub shooterSub;
    private final IntakeSub intakeSub;

    private Timer timer;

    public aShootClose(ActuatorSub actuatorSub, ShooterSub shooterSub, IntakeSub intakeSub) { 

        this.actuatorSub = actuatorSub;
        this.shooterSub = shooterSub;
        this.intakeSub = intakeSub;

        timer = new Timer();
        timer.stop();
        timer.reset();

        addRequirements(shooterSub, intakeSub);
    }

    @Override 
    public void initialize() {
        actuatorSub.setDesiredAngle(58);
        shooterSub.shooterMotorsOn();
        timer.start();
    }

    @Override 
    public void execute() {
        if (timer.get() > .3) intakeSub.intakeMotorOn();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSub.shooterMotorsOff();
        intakeSub.intakeMotorOff();

        timer.stop();
        timer.reset();
    }

    @Override 
    public boolean isFinished() {
        if (timer.get() > .6) return true;
        else return false;
    }
}
