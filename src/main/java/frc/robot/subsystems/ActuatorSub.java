package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ActuatorSub extends SubsystemBase {

    //Declare motors, sensors, and variables
    TalonFX actuatorMotor;
    double desiredAngle = Constants.ActuatorSub.defaultAngle;
    
    public ActuatorSub() {
        actuatorMotor = new TalonFX(Constants.ActuatorSub.actuatorMotorID);
        actuatorMotor.setInverted(Constants.ActuatorSub.actuatorMotorInverted);
        actuatorMotor.setNeutralMode(NeutralModeValue.Brake);
        actuatorMotor.setPosition(0);
    }

    //Declare subsystem methods
    public void setDesiredAngle(double desiredAngle) {
        this.desiredAngle = desiredAngle;
    }

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public void actuatorMotorDown() {
        actuatorMotor.set(Constants.ActuatorSub.actuatorDownSpeed);
    }

    public void actuatorMotorOff() {
        actuatorMotor.set(0);
    }

    public void setMotorPosition(double position) {
        actuatorMotor.setPosition(position);
    }

    public double getMotorPosition() {
        return actuatorMotor.getRotorPosition().getValueAsDouble();
    }

    public double desiredAngleToMotorPosition() {
        return ( Math.sqrt(
            Math.pow(Constants.ActuatorSub.shooterLength, 2) + Math.pow(Constants.ActuatorSub.bottomLength, 2) +
            -2.0 * Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.bottomLength *
            Math.cos(getDesiredAngle() - Constants.ActuatorSub.shooterMinAngle + Constants.ActuatorSub.bottomAngle * 
            Math.PI / 180.0)) - Constants.ActuatorSub.actuatorMinLength ) / 
            (2.0 * Math.PI * Constants.ActuatorSub.actuatorRate);
    } 

    public double motorPositionToCurrentAngle() {
        return (Math.PI / 180 * Math.cosh(
            (Math.pow((2 * Math.PI * Constants.ActuatorSub.actuatorRate * getMotorPosition()), 2) - 
            Math.pow(Constants.ActuatorSub.shooterLength, 2) - Math.pow(Constants.ActuatorSub.bottomLength, 2)) /
            (-2.0 * Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.bottomLength)
        )) + Constants.ActuatorSub.shooterMinAngle - Constants.ActuatorSub.bottomAngle;
    }

    public boolean onTarget() {
        if (Math.abs(motorPositionToCurrentAngle() - desiredAngle) < Constants.ActuatorSub.maxError) return true;
        else return false;
    }

    public void regulateDesiredAngle() {
        if (getDesiredAngle() > Constants.ActuatorSub.maxDesiredAngle) {
            setDesiredAngle(Constants.ActuatorSub.maxDesiredAngle);
        } else if (getDesiredAngle() < Constants.ActuatorSub.minDesiredAngle) {
            setDesiredAngle(Constants.ActuatorSub.minDesiredAngle);
        }
    }

    public void actuateToGoalAngle() {
        regulateDesiredAngle();
        if (onTarget()) {
            actuatorMotor.stopMotor();
        } else {
            actuatorMotor.set(Constants.ActuatorSub.actuatorPID.calculate(motorPositionToCurrentAngle(), getDesiredAngle()));
            //actuatorMotor.set(Constants.ActuatorSub.actuatorPID.calculate(getMotorPosition(), goalAngleToPIDRotations()));
        }
    }
    
    //Declare inline Commands
    public Command a_setDesiredAngle(double desiredAngle) {
        return runOnce(() -> {
            setDesiredAngle(desiredAngle);
        });
    }

    @Override //This method is called continuously
    public void periodic() {

        actuateToGoalAngle();

        //SmartDashboard.putNumber("actuatorMotorPosition", getMotorPosition());
        SmartDashboard.putNumber("desiredAngle", getDesiredAngle());

        //SmartDashboard.putNumber("UpPID", actuatorPID.calculate(getMotorPosition(), goalAngleToPIDRotations()));
        //SmartDashboard.putNumber("positionError", actuatorPID.getPositionError());
        //SmartDashboard.putNumber("velocityError", actuatorPID.getVelocityError());
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
