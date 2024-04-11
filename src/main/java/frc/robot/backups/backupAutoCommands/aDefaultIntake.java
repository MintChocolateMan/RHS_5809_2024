package frc.robot.backups.backupAutoCommands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class aDefaultIntake extends Command {
  
    private final IntakeSub intakeSub;
    private final ShooterSub shooterSub;
    private final ActuatorSub actuatorSub;

    private Timer timer;
    private Timer suckTimer;

    public aDefaultIntake(IntakeSub intakeSub, ShooterSub shooterSub, ActuatorSub actuatorSub) { 

        this.intakeSub = intakeSub;
        this.shooterSub = shooterSub;
        this.actuatorSub = actuatorSub;

        timer = new Timer();
        timer.stop();
        timer.reset();
        suckTimer = new Timer();
        suckTimer.stop();
        suckTimer.reset();

        addRequirements(intakeSub, shooterSub, actuatorSub);
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsOff();
        intakeSub.intakeMotorOn();
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        timer.start();
    }

    @Override 
    public void execute() {
        if (timer.get() <= .3) {
            intakeSub.intakeMotorOn();
        } else if (timer.get() > 0.3 && timer.get() < .5) {
            intakeSub.intakeMotorOff();
        } else if (intakeSub.getNoteLoaded() == true && timer.get() > .5 && suckTimer.get() == 0) {
            intakeSub.intakeMotorReverse();
        } else if (intakeSub.getNoteLoaded() == false && timer.get() > .3 && suckTimer.get() == 0) {
            suckTimer.start();
        } else if (suckTimer.get() > 0 && suckTimer.get() < .1) {
            intakeSub.intakeMotorSlow();
        } else {
            intakeSub.intakeMotorOff();
        }
        
    }

    @Override 
    public void end(boolean interrupted) {
        shooterSub.shooterMotorsOff();
        intakeSub.intakeMotorOff();

        timer.stop();
        timer.reset();
        suckTimer.stop();
        suckTimer.reset();
    }

    @Override 
    public boolean isFinished() {
        if (timer.get() > 1) return true;
        else return false;
    }
}
