package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
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
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton TeleopShooterAutoAim = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Operator Buttons */
    private final JoystickButton intakeOn = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton shooterShoot = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton climbersExtend = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private final SwerveSub SwerveSub = new SwerveSub();
    private final IntakeSub IntakeSub = new IntakeSub();
    private final ShooterSub ShooterSub = new ShooterSub();
    private final PneumaticSub PneumaticSub = new PneumaticSub();
    //private final CameraSub CameraSub = new CameraSub();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        SwerveSub.setDefaultCommand(
            new d_TeleopSwerve(
                SwerveSub, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        /* Register Commands with PathPlanner */
        NamedCommands.registerCommand("IntakeOn", IntakeSub.IntakeOn());
        NamedCommands.registerCommand("IntakeOff", IntakeSub.IntakeOff());

        // Build an auto chooser. This will use Commands.none() as the default option.
        // Optionally use .buildAutoChooser("Default Auto") to specify a default auto
        autoChooser = AutoBuilder.buildAutoChooser(); 
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
        zeroGyro.onTrue(new InstantCommand(() -> SwerveSub.zeroHeading()));
        
        /*TeleopShooterAutoAim.onTrue(new TeleopAutoAim(CameraSub, SwerveSub,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)
        ));*/

        /* Operator Buttons */
        intakeOn.whileTrue(new i_TeleopIntake(IntakeSub));
        shooterShoot.whileTrue(new s_ShooterShoot(ShooterSub));
        climbersExtend.whileTrue(new p_ClimbersExtend(PneumaticSub));
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