package frc.robot.backups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class i_DefaultIntake extends Command {
  
    //Declare subsystems
    private final IntakeSub intakeSub;

    private Timer timer;

    private boolean intaked;
    private boolean reset;

    public i_DefaultIntake(IntakeSub intakeSub) { //Command constructor
        //Initialize subsystems
        this.intakeSub = intakeSub;

        timer = new Timer();
        timer.stop();
        timer.reset();

        intaked = false;
        reset = false;

        //Add subsystem requirements
        addRequirements(intakeSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        intakeSub.intakeMotorOn();
        timer.start();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        if (timer.get() > .3) {
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

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        intakeSub.intakeMotorOff();

        timer.stop();
        timer.reset();

        intaked = false;
        reset = false;
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
