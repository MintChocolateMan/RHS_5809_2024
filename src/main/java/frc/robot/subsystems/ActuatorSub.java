package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ActuatorSub extends SubsystemBase {

    private TalonFX actuatorMotor;

    private TrapezoidProfile.Constraints actuatorControllerConstraints = new TrapezoidProfile.Constraints(Constants.ActuatorSub.maxV, Constants.ActuatorSub.maxA);
    private ProfiledPIDController actuatorController = new ProfiledPIDController(Constants.ActuatorSub.kP, Constants.ActuatorSub.kI, Constants.ActuatorSub.kD, actuatorControllerConstraints, 0.02);
    private SimpleMotorFeedforward actuatorFeedforward = new SimpleMotorFeedforward(Constants.ActuatorSub.kS, Constants.ActuatorSub.kV, Constants.ActuatorSub.kA);
    

    private double desiredAngle = 0;
    private boolean zeroing = false;

    public ActuatorSub() {
        actuatorMotor = new TalonFX(Constants.ActuatorSub.actuatorMotorID);
        actuatorMotor.setInverted(Constants.ActuatorSub.actuatorMotorInverted);
        actuatorMotor.setNeutralMode(NeutralModeValue.Brake);
        actuatorMotor.getConfigurator().apply(Robot.ctreConfigs.actuatorFXConfig);
        actuatorMotor.setPosition(0);

        actuatorController.setIZone(Constants.ActuatorSub.kIZone);
        actuatorController.setIntegratorRange(-Constants.ActuatorSub.kIMax, Constants.ActuatorSub.kIMax);
    }

    public void setDesiredAngle(double desiredAngle) {
        this.desiredAngle = desiredAngle;
    }

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public void setZeroing(boolean zeroing) {
        this.zeroing = zeroing;
    }

    public boolean getZeroing() {
        return zeroing;
    }

    public void resetActuatorController() {
        actuatorController.reset(getMotorPosition());
    }

    public void setMotorPosition(double position) {
        actuatorMotor.setPosition(position);
    }

    public double getMotorPosition() {
        return actuatorMotor.getRotorPosition().getValueAsDouble();
    }

    public void actuatorMotorDown() {
        actuatorMotor.setVoltage(-Constants.ActuatorSub.actuatorDownSpeed * 12);
    }

    public void actuatorMotorOff() {
        actuatorMotor.set(0);
    }

    public boolean onTarget() {
        if (Math.abs(getAngleFromMotorPosition(getMotorPosition()) - getDesiredAngle()) < Constants.ActuatorSub.maxError) return true;
        else return false;
    }

    public void initActuator() {
        setDesiredAngle(getAngleFromMotorPosition(getMotorPosition()));
    }

    public void regulateDesiredAngle() {
        if (getDesiredAngle() > Constants.ActuatorSub.maxDesiredAngle) {
            setDesiredAngle(Constants.ActuatorSub.maxDesiredAngle);
        } else if (getDesiredAngle() < Constants.ActuatorSub.minDesiredAngle) {
            setDesiredAngle(Constants.ActuatorSub.minDesiredAngle);
        }
    }

    public double getMotorPositionFromAngle(double angle) {
        return (
            (
                Math.sqrt(
                    Math.pow(Constants.ActuatorSub.bottomLength, 2) + Math.pow(Constants.ActuatorSub.shooterLength, 2) -
                    2 * Constants.ActuatorSub.bottomLength * Constants.ActuatorSub.shooterLength * Math.cos(Math.PI / 180 * (
                        angle - Constants.ActuatorSub.shooterMinAngle + Constants.ActuatorSub.bottomAngle
                    ))
                ) - Constants.ActuatorSub.actuatorMinLength
            ) / (
                Constants.ActuatorSub.actuatorRate
            )
        );
    }

    public double getAngleFromMotorPosition(double motorPosition) {
        return ( 
            Constants.ActuatorSub.shooterMinAngle - Constants.ActuatorSub.bottomAngle + (180 / Math.PI * Math.acos(
                (
                    Math.pow((
                        Constants.ActuatorSub.actuatorMinLength + Constants.ActuatorSub.actuatorRate * motorPosition
                    ), 2) - Math.pow(Constants.ActuatorSub.shooterLength, 2) - Math.pow(Constants.ActuatorSub.bottomLength, 2)
                ) / (
                    -2 * Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.bottomLength
                )
            ))
        );
    }

    public double getGravityFeedforward(double angle) {
        return Constants.ActuatorSub.kG / (
            (
                Constants.ActuatorSub.shooterMass * Constants.ActuatorSub.shooterCenterOfMass * 9.8 * Math.sin(
                    Math.PI / 180 * (angle - Constants.ActuatorSub.shooterMinAngle) 
                )
            ) / (
                Constants.ActuatorSub.shooterLength * Math.sin(Math.PI / 180 * (
                    180 - (180 / Math.PI * Math.asin(
                        Constants.ActuatorSub.bottomLength / (Constants.ActuatorSub.actuatorMinLength + Constants.ActuatorSub.actuatorRate * getMotorPosition()) * Math.sin( Math.PI / 180 * (
                            angle - Constants.ActuatorSub.shooterMinAngle + Constants.ActuatorSub.bottomAngle
                        ))
                    ))
                ))
            )
        );
    }

    public void actuateToGoalAngle() {
        regulateDesiredAngle();
        actuatorController.setGoal(getMotorPositionFromAngle(getDesiredAngle()));
        
        actuatorMotor.setVoltage(
            actuatorController.calculate(getMotorPosition()) +
            actuatorFeedforward.calculate(actuatorController.getSetpoint().velocity) +
            getGravityFeedforward(getAngleFromMotorPosition(actuatorController.getSetpoint().position))
        );
    }

    @Override 
    public void periodic() {
        if (!getZeroing()) {
            actuateToGoalAngle();
        }

        SmartDashboard.putNumber("desiredAngle", getDesiredAngle());
        SmartDashboard.putNumber("GFeedforward", getGravityFeedforward(getAngleFromMotorPosition(actuatorController.getSetpoint().position)));
        SmartDashboard.putNumber("currentAngle", getAngleFromMotorPosition(getMotorPosition()));
    }
}
