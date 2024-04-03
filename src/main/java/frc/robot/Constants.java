package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    
    public static final double stickDeadband = 0.1;

    public static final class IntakeSub {
        public static final int intakeMotorID = 10;
        public static final double intakeMotorSpeed = .8;
        public static final double intakeMotorReverseSpeed = .2;
        public static final boolean intakeMotorReversed = true;
        public static final int lineBreakerID = 1;

        public static final PIDController intakePID = new PIDController(
            0.05,
            0,
            0
        );
        public static final double intakePIDGoal = -1.8;

        public static final double maxIntakeError = 15;

        public static final int intakeCurrentLimit = 35;
        public static final int intakeCurrentThreshold = 50;
        public static final double intakeCurrentThresholdTime = 0.1;
        public static final boolean intakeEnableCurrentLimit = true;
    }

    public static final class ShooterSub {
        public static final int topMotorID = 11; // AKA Left Motor
        public static final int bottomMotorID = 12; // AKA Right Motor
        public static final boolean topMotorInverted = true;
        public static final boolean bottomMotorInverted = true;
        public static final double shootVoltage = 11;
        public static final double ampVoltage = 2.5;
        public static final double reverseVoltage = 3;
        public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Brake;

        public static final double initialVelocity = 10;

        public static final int shooterCurrentLimit = 30;
        public static final int shooterCurrentThreshold = 50;
        public static final double shooterCurrentThresholdTime = 0.1;
        public static final boolean shooterEnableCurrentLimit = true;
    }

    public static final class PneumaticSub {
        public static final int pneumaticHubID = 18;
        public static final int leftClimberForwardID = 5;
        public static final int leftClimberReverseID = 12;
        public static final int rightClimberForwardID = 3;
        public static final int rightClimberReverseID = 10;
    }

    public static final class ActuatorSub {
        public static final int actuatorMotorID = 19;
        public static final boolean actuatorMotorInverted = false;

        public static final double maxV = 8;
        public static final double maxA = 15;
        public static final double kP = 1.2;
        public static final double kI = .05;
        public static final double kD = 0.01;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 0.45;
        public static final double kIZone = 5;
        public static final double kIMax = .3;

        public static final double shooterLength = 6.5;
        public static final double bottomLength = 17.1;
        public static final double actuatorMinLength = 10.8;
        public static final double shooterMinAngle = 22.8;
        public static final double bottomAngle = 6.9;
        public static final double shooterMass = 3.5;
        public static final double shooterCenterOfMass = .25;
        public static final double actuatorRate = .3846;

        public static final double maxError = 2;

        public static final double maxDesiredAngle = 62;
        public static final double minDesiredAngle = 28;

        public static final double defaultAngle = 42;

        public static final double closeAngle = 62;
        public static final double stageAngle = 38;
        public static final double ampAngle = 62;
        public static final double ferryAngle = 55;

        public static final double actuatorDownSpeed = .03;

        public static final int actuatorCurrentLimit = 30;
        public static final int actuatorCurrentThreshold = 50;
        public static final double actuatorCurrentThresholdTime = 0.1;
        public static final boolean actuatorEnableCurrentLimit = true;

        //Old method of control - Currently not in use
        public static final PIDController actuatorPID = new PIDController(
            0.017,
            0,
            0.0003
        );
        public static final double actuatorIZone = 5;
        public static final double actuatorUpkF = 0.05;
        public static final double actuatorDownkF = 0;
        public static final double actuatorkG = 0.01;
        public static final double actuatorpG = 1;
    }

    public static final class PoseEstimatorSub {

        public static final double driveVisionStdDevs = 15;
        public static final double aimVisionStdDevs = .02;

        public static final double speakerTargetHeight = 1.9;
        public static final double shootkG = 1.2;

        public static final Pose2d redCloseSpeakerPose = new Pose2d(
            new Translation2d(15, 5.55),
            new Rotation2d(Math.PI)
        );
        public static final Pose2d blueCloseSpeakerPose = new Pose2d(
            new Translation2d(1.5, 5.55),
            new Rotation2d()
        );
        public static final Pose2d redAmpPose = new Pose2d(
            new Translation2d(14.67, 7.65),
            new Rotation2d()
        );
        public static final Pose2d blueAmpPose = new Pose2d(
            new Translation2d(1.83, 7.65),
            new Rotation2d()
        );
        public static final Pose2d redSpeakerPoseYaw = new Pose2d(
            new Translation2d(16.3, 5.55),
            new Rotation2d()
        );
        public static final Pose2d blueSpeakerPoseYaw = new Pose2d(
            new Translation2d(.2, 5.55),
            new Rotation2d()
        );
        public static final Pose2d redSpeakerPoseDistance = new Pose2d(
            new Translation2d(16.5, 5.55),
            new Rotation2d()
        );
        public static final Pose2d blueSpeakerPoseDistance = new Pose2d(
            new Translation2d(0, 5.55),
            new Rotation2d()
        );
    }

    public static final class CandleSub {
        public static final int candleID = 22;
        public static final double brightness = 1;
    }

    public static final class PathPlanner {
        public static final HolonomicPathFollowerConfig pathPlannerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants( // Translation PID constants
                5.0, 
                0.0, 
                0.0), 
            new PIDConstants( // Rotation PID constants
                5.0, 
                0.0, 
                0.0
            ),
            4.5,
            .41,
            new ReplanningConfig()
        );
    }

    public static final class Swerve {
        public static final int pigeonID = 13;

        public static final double translationSensitivity = 1;
        public static final double rotationSensitivity = 1;
        public static final double maxError = 5;

        public static final double maxIntakeError = 5;
        
        public static final PIDController swerveRotationPID = new PIDController(
            0.1,
            0,
            .015
        );

        public static final PIDController swerveTranslationPID = new PIDController(
            0.05,
            0, 
            0.1
        );

        public static final PIDController swerveStrafePID = new PIDController(
            0.05,
            0, 
            0
        );

        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.75); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(22.75); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.5; // 4.6//normally 5.5 TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 9; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3; 
            public static final int angleMotorID = 2; 
            public static final int canCoderID = 14; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-96.5);
            public static final boolean driveInverted = false; //TODO change for which motors are upside down
            public static final boolean angleInverted = false;  // currently reverse both front left motors and drive on front right
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveInverted, angleInverted);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5; 
            public static final int angleMotorID = 4; 
            public static final int canCoderID = 15; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(31);
            public static final boolean driveInverted = false;
            public static final boolean angleInverted = false;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveInverted, angleInverted);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7; //9
            public static final int angleMotorID = 6; //8
            public static final int canCoderID = 16; //17
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(89.6);
            public static final boolean driveInverted = false;
            public static final boolean angleInverted = true;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveInverted, angleInverted);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 9; 
            public static final int angleMotorID = 8; 
            public static final int canCoderID = 17; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-94.2);
            public static final boolean driveInverted = false;
            public static final boolean angleInverted = true;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveInverted, angleInverted);
        }
    }

    //ALL AUTOCONSTANTS UNUSED
    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 5.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}