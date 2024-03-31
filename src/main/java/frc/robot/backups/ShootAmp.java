package frc.robot.backups;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ShootAmp extends Command {
  
    //Declare subsystems
    private final ActuatorSub actuatorSub;
    private final ShooterSub shooterSub;
    private final IntakeSub intakeSub;

    Timer timer;

    public ShootAmp(ActuatorSub actuatorSub, ShooterSub shooterSub, IntakeSub intakeSub) { //Command constructor
        //Initialize subsystems
        this.actuatorSub = actuatorSub;
        this.shooterSub = shooterSub;
        this.intakeSub = intakeSub;

        timer = new Timer();
        timer.reset();

        //Add subsystem requirements
        addRequirements(shooterSub, intakeSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        shooterSub.shooterMotorsAmp();
        timer.start();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        if (timer.get() > 0.75) {
            intakeSub.intakeMotorOn();
        }
    }

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
        shooterSub.shooterMotorsOff();
        intakeSub.intakeMotorOff();
        timer.reset();
        timer.stop();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        if (timer.get() > 1.25) {
            return true;
        }
        return false;
    }
}
