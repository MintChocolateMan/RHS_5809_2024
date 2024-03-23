package frc.lib.util;

import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public TalonFXConfiguration actuatorFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration shooterFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration intakeFXConfig = new TalonFXConfiguration();
    public CANdleConfiguration candleConfig = new CANdleConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        swerveCANcoderConfig.MagnetSensor.MagnetOffset = 0;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        
        //Actuator motor current limiting
        actuatorFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ActuatorSub.actuatorEnableCurrentLimit;
        actuatorFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.ActuatorSub.actuatorCurrentLimit;
        actuatorFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.ActuatorSub.actuatorCurrentThreshold;
        actuatorFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.ActuatorSub.actuatorCurrentThresholdTime;

        //shooter motors current limiting
        shooterFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ShooterSub.shooterEnableCurrentLimit;
        shooterFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.ShooterSub.shooterCurrentLimit;
        shooterFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.ShooterSub.shooterCurrentThreshold;
        shooterFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.ShooterSub.shooterCurrentThresholdTime;

        //intake motor current limiting shooterFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ShooterSub.shooterEnableCurrentLimit;
        intakeFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.IntakeSub.intakeEnableCurrentLimit;
        intakeFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.IntakeSub.intakeCurrentLimit;
        intakeFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.IntakeSub.intakeCurrentThreshold;
        intakeFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.IntakeSub.intakeCurrentThresholdTime;

        //CANdle configs
        candleConfig.brightnessScalar = Constants.CandleSub.brightness;
        candleConfig.statusLedOffWhenActive = true;
        candleConfig.stripType = LEDStripType.RGB;
    }
}