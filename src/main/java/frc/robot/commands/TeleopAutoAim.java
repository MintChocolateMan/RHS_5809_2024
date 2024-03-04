package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class TeleopAutoAim extends Command {
  
    //Declare subsystems
    private final PoseEstimatorSub poseEstimatorSub;
    private final SwerveSub swerveSub;
    private final ShooterSub shooterSub;
    private final ActuatorSub actuatorSub;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;

    public TeleopAutoAim(PoseEstimatorSub poseEstimatorSub, SwerveSub swerveSub, ShooterSub shooterSub, ActuatorSub actuatorSub, DoubleSupplier translationSup, DoubleSupplier strafeSup) { //Command constructor
        //Initialize subsystems
        this.swerveSub = swerveSub;
        this.poseEstimatorSub = poseEstimatorSub;
        this.shooterSub = shooterSub;
        this.actuatorSub = actuatorSub;

        //Add subsystem requirements
        addRequirements(swerveSub, shooterSub, actuatorSub);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
    }

    @Override //Called when the command is initially scheduled.
    public void initialize() {
        shooterSub.shooterShoot();
    }

    @Override // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        swerveSub.driveWithRotation(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            poseEstimatorSub.getTargetYaw()
        );

        actuatorSub.setDesiredAngle(poseEstimatorSub.getTargetPitch() + 180);
    }

    @Override // Called once the command ends or is interrupted.
    public void end(boolean interrupted) {
        shooterSub.stopMotors();
        actuatorSub.setDesiredAngle(Constants.ActuatorSub.defaultAngle);
    }

    @Override // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}
