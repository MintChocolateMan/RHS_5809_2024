package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AimClose extends Command {
  
    //Declare subsystems
    private final ActuatorSub actuatorSub;
    private final ShooterSub shooterSub;

    public AimClose(ActuatorSub actuatorSub, ShooterSub shooterSub) { //Command constructor
        //Initialize subsystems
        this.actuatorSub = actuatorSub;
        this.shooterSub = shooterSub;

        //Add subsystem requirements
        addRequirements(shooterSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        shooterSub.shooterMotorsOn();
        actuatorSub.setDesiredAngle(64);
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {}

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        shooterSub.shooterMotorsOff();
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
