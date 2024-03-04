package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class TeleopAutoAim extends Command {
  
    //Declare subsystems
    private final PoseEstimatorSub poseEstimatorSub;
    private final SwerveSub swerveSub;
    private final ShooterSub shooterSub;
    private final ActuatorSub actuatorSub;

    public TeleopAutoAim(PoseEstimatorSub poseEstimatorSub, SwerveSub swerveSub, ShooterSub shooterSub, ActuatorSub actuatorSub) { //Command constructor
        //Initialize subsystems
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;
        this.shooterSub = shooterSub;
        this.actuatorSub = actuatorSub;

        //Add subsystem requirements
        addRequirements(swerveSub, poseEstimatorSub, shooterSub, actuatorSub);
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {}

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        swerveSub.drive(new Translation2d(0, 0), poseEstimatorSub.robotToSpeaker(), true, true);
    }

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {}

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
