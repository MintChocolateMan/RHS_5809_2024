package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ActuatorSub extends SubsystemBase {

    private TalonFX actuatorMotor;

    /*
     * HOW TO TUNE:
     * 1. Set everything to 0, then increase kG until arm can hold its position anywhere
     * 2. start tuning kP until it gets to position well as well as adjust maxV and maxA so that it doesn't overshoot
     * 3. 
     */
    private final TrapezoidProfile.Constraints actuatorControllerConstraints = new TrapezoidProfile.Constraints(10, 5);
    private final ProfiledPIDController actuatorController = new ProfiledPIDController(0, 0, 0, actuatorControllerConstraints, 0.02);
    private final ArmFeedforward actuatorFeedforward = new ArmFeedforward(0, 0, 0, 0);
    

    private double desiredAngle = 0;
    private boolean zeroing = false;

    public ActuatorSub() {
        actuatorMotor = new TalonFX(Constants.ActuatorSub.actuatorMotorID);
        actuatorMotor.setInverted(Constants.ActuatorSub.actuatorMotorInverted);
        actuatorMotor.setNeutralMode(NeutralModeValue.Brake);
        actuatorMotor.getConfigurator().apply(Robot.ctreConfigs.actuatorFXConfig);
        actuatorMotor.setPosition(0);
    }

    //Declare subsystem methods
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

    public void setMotorPosition(double position) {
        actuatorMotor.setPosition(position);
    }

    public double getMotorPosition() {
        return actuatorMotor.getRotorPosition().getValueAsDouble();
    }

    public boolean onTarget() {
        if (Math.abs(getActuatorAngle() - getDesiredAngle()) < Constants.ActuatorSub.maxError) return true;
        else return false;
    }

    public void actuatorMotorDown() {
        actuatorMotor.set(-Constants.ActuatorSub.actuatorDownSpeed);
    }

    public void actuatorMotorOff() {
        actuatorMotor.set(0);
    }

    public void regulateDesiredAngle() {
        if (getDesiredAngle() > Constants.ActuatorSub.maxDesiredAngle) {
            setDesiredAngle(Constants.ActuatorSub.maxDesiredAngle);
        } else if (getDesiredAngle() < Constants.ActuatorSub.minDesiredAngle) {
            setDesiredAngle(Constants.ActuatorSub.minDesiredAngle);
        }
    }

    public double getActuatorAngle() {
        return (((180 / Math.PI) * 
            Math.acos((((Constants.ActuatorSub.actuatorMinLength + Constants.ActuatorSub.actuatorRate * getMotorPosition())) * 
            ((Constants.ActuatorSub.actuatorMinLength + Constants.ActuatorSub.actuatorRate * getMotorPosition())) - 
            (Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.shooterLength) - 
            (Constants.ActuatorSub.bottomLength * Constants.ActuatorSub.bottomLength)) / 
            (-2.0 * Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.bottomLength))) - 
            Constants.ActuatorSub.bottomAngle + Constants.ActuatorSub.shooterMinAngle);
    }

    public double setpointToFeedforwardAngle(double angle) {
        return Math.asin(Constants.ActuatorSub.bottomLength * Math.sin((
            angle - Constants.ActuatorSub.shooterMinAngle + Constants.ActuatorSub.bottomAngle) * Math.PI / 180.0) / 
            (Constants.ActuatorSub.actuatorMinLength + Constants.ActuatorSub.actuatorRate * getMotorPosition())) - 
            (Math.PI / 180.0 * (90.0 - angle + Constants.ActuatorSub.shooterMinAngle));
    }

    public void actuateToGoalAngle() {
        regulateDesiredAngle();
        actuatorController.setGoal(getDesiredAngle());
        
        actuatorMotor.setVoltage(
            actuatorController.calculate(getActuatorAngle()) +
            actuatorFeedforward.calculate(
                setpointToFeedforwardAngle(actuatorController.getSetpoint().position), 
                actuatorController.getSetpoint().velocity)
        );
    }

    @Override 
    public void periodic() {
        if (!getZeroing()) {
            actuateToGoalAngle();
        }

        SmartDashboard.putNumber("desiredAngle", getDesiredAngle());
        SmartDashboard.putNumber("currentAngle", getActuatorAngle());
    }
}
