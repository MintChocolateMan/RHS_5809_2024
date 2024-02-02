package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ShooterCameraSub;
import edu.wpi.first.math.controller.PIDController;


public class TeleopAutoAim extends Command {    
    private ShooterCameraSub c_ShooterCameraSub;
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private double rotationVal;

    PIDController rotationController = new PIDController(.01, 0, 0);

    public TeleopAutoAim(ShooterCameraSub c_ShooterCameraSub, Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.c_ShooterCameraSub = c_ShooterCameraSub;
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve, c_ShooterCameraSub);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        

        if(c_ShooterCameraSub.driverCameraHasTarget()) {
            rotationVal = rotationController.calculate(c_ShooterCameraSub.driverCameraGetYaw());
        } else {
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        }

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            true,
            true
        );
    }
}