package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.backups.*;
//import frc.robot.autoCommands.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser;

    
    
    // XBOX Controller Buttons
    private final XboxController driver = new XboxController(0);
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton zeroActuator = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final JoystickButton autoIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton autoShoot = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton autoAmp = new JoystickButton(driver, XboxController.Button.kA.value);
    //private final JoystickButton scoreAmp = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton toggleClimbers = new JoystickButton(driver, XboxController.Button.kX.value);
    //private final JoystickButton aimClose = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton aimStage = new JoystickButton(driver, XboxController.Button.kX.value);
    //private final JoystickButton setPoseCloseSpeaker = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    //private final JoystickButton setPoseProtected = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    
    /*
    // JOYSTICK Buttons
    private final Joystick driver = new Joystick(0);
    private final int translationAxis = Joystick.AxisType.kY.value;
    private final int strafeAxis = Joystick.AxisType.kX.value;
    private final int rotationAxis = Joystick.AxisType.kZ.value;
    private final JoystickButton zeroGyro = new JoystickButton(driver, 4);
    private final JoystickButton robotCentric = new JoystickButton(driver, 6);
    private final JoystickButton intake = new JoystickButton(driver, 1);
    private final JoystickButton autoShoot = new JoystickButton(driver, 2);
    private final JoystickButton extendClimbers = new JoystickButton(driver, 8);
    private final JoystickButton aimClose = new JoystickButton(driver, 3);
    private final JoystickButton zeroActuator = new JoystickButton(driver, 5);
    */

    /* Subsystems */
    private final PoseEstimatorSub poseEstimatorSub = new PoseEstimatorSub();
    private final CandleSub candleSub = new CandleSub();
    private final SwerveSub swerveSub = new SwerveSub(poseEstimatorSub);
    private final IntakeSub intakeSub = new IntakeSub(candleSub);
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
        //intakeSub.setDefualtCommand(new SuckBack(intakeSub, shooterSub));

        /* Register Commands with PathPlanner */
        NamedCommands.registerCommand("AutoIntake", new AutoIntake(
            intakeSub, swerveSub, poseEstimatorSub, 
            () -> 0,
            () -> 0,
            () -> 0
        ));
        NamedCommands.registerCommand("AutoShoot", new AutoShoot(
            poseEstimatorSub, swerveSub, shooterSub, actuatorSub, intakeSub, 
            () -> 0, 
            () -> 0
        ));
        NamedCommands.registerCommand("SuckBack2", new SuckBack(intakeSub, shooterSub));

        NamedCommands.registerCommand("i_Intake", new i_Intake(intakeSub));

        /* Register Manual Commands with PathPlanner */
        /*NamedCommands.registerCommand("SpeakerAim", new SpeakerAim(actuatorSub));
        NamedCommands.registerCommand("CenterNoteAim", new CenterNoteAim(actuatorSub));
        NamedCommands.registerCommand("LRNoteAim", new LRNoteAim(actuatorSub));
        NamedCommands.registerCommand("ShootAuto", new ShootAuto(shooterSub));
        NamedCommands.registerCommand("SuckBack", new SuckBack(intakeSub));
        NamedCommands.registerCommand("SourceAim", new SourceAim(actuatorSub));
        NamedCommands.registerCommand("ReverseShooter", new ReverseShooterAuto(shooterSub));
        NamedCommands.registerCommand("HailMaryAim", new HailMaryAim(actuatorSub));
        */
       
        

        autoChooser = AutoBuilder.buildAutoChooser("Close Speaker Manual"); 
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
        zeroActuator.whileTrue(new a_ZeroActuator(actuatorSub));
        /*autoIntake.whileTrue(new AutoIntake(intakeSub, swerveSub, poseEstimatorSub, 
            () -> translationAxis, 
            () -> strafeAxis, 
            () -> rotationAxis
        ));*/
        //autoIntake.whileTrue(new i_Intake(intakeSub));
        autoIntake.whileTrue(new AutoIntake(
            intakeSub, swerveSub, poseEstimatorSub, 
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)
        ));
        autoShoot.whileTrue(new AutoShoot(poseEstimatorSub, swerveSub, shooterSub, actuatorSub, intakeSub,
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis)
        ));
        autoAmp.whileTrue(new AutoAmp(poseEstimatorSub, swerveSub, shooterSub, actuatorSub, intakeSub, 
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis)
        ));
        /*scoreAmp.whileTrue(new SequentialCommandGroup(
            new a_AimAmp(actuatorSub), 
            AutoBuilder.pathfindThenFollowPath(
                swerveSub.scoreAmp,
                swerveSub.scoreAmpConstraints,
                0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
            ), 
            new ShootAmp(actuatorSub, shooterSub, intakeSub)
        ));*/
        toggleClimbers.onTrue(new p_ToggleClimbers(pneumaticSub));
        //aimClose.whileTrue(new AimClose(actuatorSub, shooterSub));
        //aimStage.whileTrue(new AimStage(actuatorSub, shooterSub));

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