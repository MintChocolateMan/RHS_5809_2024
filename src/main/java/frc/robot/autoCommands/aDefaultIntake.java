package frc.robot.autoCommands;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aDefaultIntake extends Command {
  
    private final IntakeSub intakeSub;
    private final ShooterSub shooterSub;
    private final ActuatorSub actuatorSub;

    private Timer timer;

    public aDefaultIntake(IntakeSub intakeSub, ShooterSub shooterSub, ActuatorSub actuatorSub) { 

        this.intakeSub = intakeSub;
        this.shooterSub = shooterSub;
        this.actuatorSub = actuatorSub;

        timer = new Timer();
        timer.stop();
        timer.reset();

        addRequirements(intakeSub, shooterSub, actuatorSub);
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsReverse();
        intakeSub.intakeMotorOn();
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        timer.start();
    }

    @Override 
    public void execute() {
        if (timer.get() < .3) {
            intakeSub.intakeMotorOn();
        }
        else if (timer.get() > .3) {
            intakeSub.intakeMotorReverse();
        }
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
        if (timer.get() > 0.7) return true;
        else return false;
    }
}
