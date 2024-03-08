package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class CenterNoteAim extends Command {
  
    private final ActuatorSub actuatorSub;
    private final ShooterSub shooterSub;

    public CenterNoteAim(ActuatorSub actuatorSub, ShooterSub shooterSub) {
        this.actuatorSub = actuatorSub;
        this.shooterSub = shooterSub;
        addRequirements(shooterSub);
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsOn();
        actuatorSub.setDesiredAngle(44);
    }

    @Override
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(36);
        shooterSub.shooterMotorsOff();
    }
}
