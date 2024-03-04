package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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
        public static final double intakeMotorReverseSpeed = .1;
        public static final boolean intakeMotorReversed = true;
        public static final int lineBreakerID = 1;

        public static final double intakekP = 0.1;
        public static final double intakekI = 0;
        public static final double intakekD = 0;
        public static final double intakePIDGoal = -1;
    }

    public static final class ShooterSub {
        public static final int topMotorID = 11; // AKA Left Motor
        public static final int bottomMotorID = 12; // AKA Right Motor
        public static final boolean topMotorInverted = true;
        public static final boolean bottomMotorInverted = true;
        public static final double shootSpeed = 1;
        public static final double intakeSpeed = .2;
        public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Brake;
    }

    public static final class PneumaticSub {
        public static final int pneumaticHubID = 18;
        public static final int leftClimberForwardID = 5;
        public static final int leftClimberReverseID = 12;
        public static final int rightClimberForwardID = 3;
        public static final int rightClimberReverseID = 15;
    }

    public static final class ActuatorSub {
        public static final int actuatorMotorID = 19;
        public static final boolean actuatorMotorInverted = false;

        public static final double actuatorkP = 0.038; //.035
        public static final double actuatorkI = 0.003; //.02
        public static final double actuatorkD = 2; //.01

        public static final double actuatorMaxError = 0.15;

        public static final double shooterLength = 6.5;
        public static final double bottomLength = 17.1;
        public static final double actuatorMinLength = 10.8;
        public static final double actuaterMaxLength = 13.7;
        public static final double maxRotations = 8.44;
        public static final double shooterMinAngle = 22.8;
        public static final double bottomAngle = 6.9;
        
        public static final double actuatorRate = .0584;

        public static final double maxDesiredAngle = 60;
        public static final double minDesiredAngle = 30;

        public static final double defaultAngle = 55;
    }

    public static final class PoseEstimatorSub {
        public static final String shooterCamName = "shooterCam";

        public static final double shooterCamForwardOffset = -0.35; //TODO find lol
        public static final double shooterCamHorizontalOffset = .1;
        public static final double shooterCamVerticalOffset = 0.2;
        public static final double shooterCamRoll = 0;
        public static final double shooterCamPitch = 30;
        public static final double shooterCamYaw = 0;

        public static final double speakerTargetHeight = 1.7;
    }

    public static final class PathPlanner {
        public static final double kPTranslation = 5.0;
        public static final double kITranslation = 0.0;
        public static final double kDTranslation = 0.0;
        public static final double kPRotation = 5.0;
        public static final double kIRotation = 0.0;
        public static final double kDRotation = 0.0;

        public static final double maxModuleSpeed = 5.5;
        public static final double driveBaseRadius = 0.41;
    }

    public static final class Swerve {
        public static final int pigeonID = 13;

        public static final double translationSensitivity = .5;
        public static final double rotationSensitivity = 1;

        public static final double rotationkP = 0.1;
        public static final double rotationkI = 0;
        public static final double rotationkD = 0;

        public static final COTSTalonFXSwerveConstants chosenModule = 
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.875); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(22.875); //TODO: This must be tuned to specific robot
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
        public static final double maxSpeed = 5.5; //normally 5.5 TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 6.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3; 
            public static final int angleMotorID = 2; 
            public static final int canCoderID = 14; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(38);
            public static final boolean driveInverted = true; //TODO change for which motors are upside down
            public static final boolean angleInverted = false;  // currently reverse both front left motors and drive on front right
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveInverted, angleInverted);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5; 
            public static final int angleMotorID = 4; 
            public static final int canCoderID = 15; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(92);
            public static final boolean driveInverted = true;
            public static final boolean angleInverted = false;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, driveInverted, angleInverted);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7; //9
            public static final int angleMotorID = 6; //8
            public static final int canCoderID = 16; //17
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-90);
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-94);
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