package frc.robot.backups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class SuckBack2 extends Command {
  
    //Declare subsystems
    private final IntakeSub intakeSub;
    private final ShooterSub shooterSub;

    private Timer timer;

    public SuckBack2(IntakeSub intakeSub, ShooterSub shooterSub) { //Command constructor
        //Initialize subsystems
        this.intakeSub = intakeSub;
        this.shooterSub = shooterSub;

        timer = new Timer();
        timer.reset();
        timer.stop();

        //Add subsystem requirements
        addRequirements(intakeSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        intakeSub.intakeMotorReverse();
        shooterSub.shooterMotorsReverse();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        if (intakeSub.getNoteLoaded() == false) {
            timer.start();
        }
    }

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();
        shooterSub.shooterMotorsOff();

        timer.stop();
        timer.reset();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        if (timer.get() > .5) return true;
        return false;
    }
}
