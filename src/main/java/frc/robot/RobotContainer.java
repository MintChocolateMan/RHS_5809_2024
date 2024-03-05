package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.backups.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser;
    
    /* Controllers */
    private final XboxController driver = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Operator Buttons */
    private final JoystickButton intake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton autoShoot = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton extendClimbers = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final JoystickButton aimClose = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton resetActuator = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Subsystems */
    private final PoseEstimatorSub poseEstimatorSub = new PoseEstimatorSub();
    private final SwerveSub swerveSub = new SwerveSub(poseEstimatorSub);
    private final IntakeSub intakeSub = new IntakeSub();
    private final ShooterSub shooterSub = new ShooterSub();
    private final ActuatorSub actuatorSub = new ActuatorSub();
    private final PneumaticSub pneumaticSub = new PneumaticSub();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        //Set Default Commands
        swerveSub.setDefaultCommand(
            new d_Swerve(
                swerveSub, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                robotCentric
            )
        );
        intakeSub.setDefaultCommand(new i_DefaultIntake(intakeSub));

        /* Register Commands with PathPlanner */
        NamedCommands.registerCommand("i_Intake", new i_Intake(intakeSub));
        NamedCommands.registerCommand("AutoAim", new AutoShoot(
            poseEstimatorSub, swerveSub, shooterSub, actuatorSub, intakeSub, 
            () -> 0, 
            () -> 0
        ));
        autoChooser = AutoBuilder.buildAutoChooser("Close to Speaker"); 
        SmartDashboard.putData("Auto Chooser:", autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> poseEstimatorSub.zeroHeading()));

        /* Operator Buttons */
        intake.whileTrue(new i_Intake(intakeSub));
        autoShoot.whileTrue(new AutoShoot(poseEstimatorSub, swerveSub, shooterSub, actuatorSub, intakeSub,
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis)
        ));
        extendClimbers.whileTrue(new p_ExtendClimbers(pneumaticSub));
        aimClose.whileTrue(new a_tempAimActuator(actuatorSub));
        resetActuator.whileTrue(new a_ResetActuator(actuatorSub));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}