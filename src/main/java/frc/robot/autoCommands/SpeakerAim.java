package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class SpeakerAim extends Command {
  
    private final ActuatorSub actuatorSub;
    private final ShooterSub shooterSub;

    public SpeakerAim(ActuatorSub actuatorSub, ShooterSub shooterSub) {
        this.actuatorSub = actuatorSub;
        this.shooterSub = shooterSub;
        addRequirements(shooterSub);
    }

    @Override 
    public void initialize() {
        shooterSub.shooterMotorsOn();
        actuatorSub.setDesiredAngle(64);
    }

    @Override
    public void end(boolean interrupted) {
        actuatorSub.setDesiredAngle(40);
        shooterSub.shooterMotorsOff();
    }
}
