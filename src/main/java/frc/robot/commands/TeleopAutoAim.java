package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.ShooterCameraSub;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.CameraSub;


public class TeleopAutoAim extends Command {    
    private CameraSub CameraSub;
    private SwerveSub SwerveSub;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    PIDController rotationController = new PIDController(.01, 0, 0);

    public TeleopAutoAim(CameraSub CameraSub, SwerveSub SwerveSub, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.CameraSub = CameraSub;
        this.SwerveSub = SwerveSub;
        addRequirements(SwerveSub);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal;

        if(CameraSub.shooterCamHasTarget()) {
            rotationVal = rotationController.calculate(CameraSub.shooterCamGetYaw());
        } else {
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        }

        SwerveSub.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            true,
            true
        );
    }
}