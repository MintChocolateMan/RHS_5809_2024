package frc.robot.backups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class s_ShooterShoot extends Command {
  
    //Declare subsystems
    private final ShooterSub shooterSub;

    public s_ShooterShoot(ShooterSub shooterSub) { //Command constructor
        //Initialize subsystems
        this.shooterSub = shooterSub;

        //Add subsystem requirements
        addRequirements(shooterSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        shooterSub.shooterShoot();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {}

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        shooterSub.stopMotors();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
