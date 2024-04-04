package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.backups.*;
import frc.robot.autoCommands.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Auto Chooser */
    private final SendableChooser<Command> autoChooser;
    
    /* Driver Keybinds */
    private final XboxController driver = new XboxController(0);
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton faceStage = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final POVButton zeroGyro = new POVButton(driver, 270);
    private final POVButton zeroActuator = new POVButton(driver, 90);
    private final JoystickButton autoIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton autoShoot = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final Trigger aimClose = new Trigger(() -> { if(driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.5) return true; else return false; });
    private final Trigger intake = new Trigger(() -> { if(driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5) return true; else return false; });
    private final JoystickButton autoAmp = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton aimStage = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton aimDiagonalFerry = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton aimStraightFerry = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Operator Keybinds */
    private final XboxController operator = new XboxController(1);
    private final POVButton climbersUp = new POVButton(operator, 0);
    private final POVButton climbersDown = new POVButton(operator, 180);
    private final JoystickButton actuatorSmallUp = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton actuatorSmallDown = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton reverseIntake = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton autoTrap = new JoystickButton(operator, XboxController.Button.kB.value);    
    
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
            new TeleopSwerve(
                swerveSub, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                robotCentric
            )
        );
        intakeSub.setDefaultCommand(new DefaultIntake(intakeSub));

        //Register Commands with PathPlanner
        NamedCommands.registerCommand("AutoIntake", new AutoIntake(
            intakeSub, swerveSub, actuatorSub, poseEstimatorSub, 
            () -> 0,
            () -> 0,
            () -> 0
        ));
        NamedCommands.registerCommand("AutoShoot", new AutoShoot(
            poseEstimatorSub, swerveSub, shooterSub, actuatorSub, intakeSub, 
            () -> 0, 
            () -> 0
        ));
        NamedCommands.registerCommand("AutoAmp", new AutoAmp(poseEstimatorSub, swerveSub, shooterSub, actuatorSub, intakeSub,
            () -> 0,
            () -> 0 
        ));

        NamedCommands.registerCommand("aAutoIntake", new aAutoIntake(intakeSub, swerveSub, poseEstimatorSub));
        NamedCommands.registerCommand("aFastAutoIntake", new aFastAutoIntake(intakeSub, swerveSub, poseEstimatorSub));
        NamedCommands.registerCommand("aAimLow", new aAimLow(actuatorSub, shooterSub, poseEstimatorSub));
        NamedCommands.registerCommand("aAimHigh", new aAimHigh(actuatorSub, shooterSub, poseEstimatorSub));
        NamedCommands.registerCommand("aInitActuator", new aInitActuator(actuatorSub));

        NamedCommands.registerCommand("aDefaultIntake", new aDefaultIntake(intakeSub));
        NamedCommands.registerCommand("aIntake", new aIntake(intakeSub));

        NamedCommands.registerCommand("OFSsetPoseStageNote", new OFSsetPoseStageNote(poseEstimatorSub));
        NamedCommands.registerCommand("OFSsetPoseCenterNote", new OFSsetPoseCenterNote(poseEstimatorSub));
        NamedCommands.registerCommand("OFSsetPoseAmpNote", new OFSsetPoseAmpNote(poseEstimatorSub));

        NamedCommands.registerCommand("OASsetPoseLeftNote", new OASsetPoseLeftNote(poseEstimatorSub));
        NamedCommands.registerCommand("OASsetPoseRightNote", new OASsetPoseRightNote(poseEstimatorSub));

        NamedCommands.registerCommand("OSsetPoseRightNote", new OSsetPoseRightNote(poseEstimatorSub));
        NamedCommands.registerCommand("OSsetPoseLeftNote", new OSsetPoseLeftNote(poseEstimatorSub));

        NamedCommands.registerCommand("FSsetPoseStageNote", new FSsetPoseStageNote(poseEstimatorSub));
        NamedCommands.registerCommand("FSsetPoseCenterNote", new FSsetPoseCenterNote(poseEstimatorSub));
        NamedCommands.registerCommand("FSsetPoseAmpNote", new FSsetPoseAmpNote(poseEstimatorSub));
        NamedCommands.registerCommand("FSsetPoseFarAmpNote", new FSsetPoseFarAmpNote(poseEstimatorSub));
        NamedCommands.registerCommand("FSsetPoseFarCenterNote", new FSsetPoseFarCenterNote(poseEstimatorSub));
       
        autoChooser = AutoBuilder.buildAutoChooser("Front Speaker Auto"); 
        SmartDashboard.putData("Auto Chooser:", autoChooser);
        

        // Configure the button bindings
        configureButtonBindings();

        /* Register Manual Commands with PathPlanner */
        /*NamedCommands.registerCommand("SpeakerAim", new SpeakerAim(actuatorSub));
        NamedCommands.registerCommand("CenterNoteAim", new CenterNoteAim(actuatorSub));
        NamedCommands.registerCommand("LRNoteAim", new LRNoteAim(actuatorSub));
        NamedCommands.registerCommand("ShootAuto", new ShootAuto(shooterSub));
        NamedCommands.registerCommand("SuckBack", new SuckBack(intakeSub));
        NamedCommands.registerCommand("SourceAim", new SourceAim(actuatorSub));
        NamedCommands.registerCommand("ReverseShooter", new ReverseShooterAuto(shooterSub));
        NamedCommands.registerCommand("HailMaryAim", new HailMaryAim(actuatorSub));
        NamedCommands.registerCommand("SuckBack2", new SuckBack2(intakeSub, shooterSub));
        NamedCommands.registerCommand("i_Intake", new Intake(intakeSub));
        */
    }

    private void configureButtonBindings() {
        
        /* Driver Functions */
        zeroGyro.onTrue(new InstantCommand(() -> poseEstimatorSub.zeroHeading()));
        zeroActuator.whileTrue(new ZeroActuator(actuatorSub));
        autoIntake.whileTrue(new AutoIntake(
            intakeSub, swerveSub, actuatorSub, poseEstimatorSub, 
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)
        ));
        autoShoot.whileTrue(new AutoShoot(poseEstimatorSub, swerveSub, shooterSub, actuatorSub, intakeSub,
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis)
        ));
        aimClose.whileTrue(new AimClose(actuatorSub, shooterSub));
        intake.whileTrue(new Intake(intakeSub));
        autoAmp.whileTrue(new AutoAmp(poseEstimatorSub, swerveSub, shooterSub, actuatorSub, intakeSub, 
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis)
        ));
        aimStage.whileTrue(new AimStage(swerveSub, shooterSub, actuatorSub,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis)
        ));
        aimDiagonalFerry.whileTrue(new AimDiagonalFerry(swerveSub, shooterSub, actuatorSub,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis)
        ));
        aimStraightFerry.whileTrue(new AimStraightFerry(swerveSub, shooterSub, actuatorSub,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis)
        ));
        faceStage.whileTrue(new FaceStage(swerveSub, poseEstimatorSub, 
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis)
        ));

        /* Operator Functions */
        climbersUp.onTrue(new ClimbersUp(pneumaticSub));
        climbersDown.onTrue(new ClimbersDown(pneumaticSub));
        actuatorSmallDown.onTrue(new ActuatorSmallDown(actuatorSub));
        actuatorSmallUp.onTrue(new ActuatorSmallUp(actuatorSub));
        reverseIntake.whileTrue(new ReverseIntake(intakeSub));
        autoTrap.whileTrue(new AutoTrap(poseEstimatorSub, swerveSub, shooterSub, actuatorSub, intakeSub, 
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis)
        ));

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