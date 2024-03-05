package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ActuatorSub extends SubsystemBase {

    //Declare motors and sensors
    TalonFX actuatorMotor;

    double desiredAngle = 0;//Constants.ActuatorSub.defaultAngle;
    
    public ActuatorSub() { //Subsystem constructor
        //Initialize motors and sensors
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

    public void actuatorMotorOn() {
        actuatorMotor.set(.5);
    }

    public void actuatorMotorOff() {
        actuatorMotor.set(0);
    }

    public double getMotorPosition() {
        return actuatorMotor.getRotorPosition().getValueAsDouble();
    }

    public double desiredAngleToGoalAngle() {
        return getDesiredAngle() - Constants.ActuatorSub.shooterMinAngle + Constants.ActuatorSub.bottomAngle;
    }

    public double goalAngleToPIDRotations() {
        return ( Math.sqrt(
            Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.shooterLength +
            Constants.ActuatorSub.bottomLength * Constants.ActuatorSub.bottomLength +
            -2.0 * Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.bottomLength *
            Math.cos(desiredAngleToGoalAngle() * Math.PI / 180.0)
        ) - Constants.ActuatorSub.actuatorMinLength ) / 
        (2.0 * Math.PI * Constants.ActuatorSub.actuatorRate);
    } 

    public double currentAngle() {
        return (Math.PI / 180 * Math.cosh(
            (Math.pow((2 * Math.PI * Constants.ActuatorSub.actuatorRate * getMotorPosition()), 2) - 
            Math.pow(Constants.ActuatorSub.shooterLength, 2) - Math.pow(Constants.ActuatorSub.bottomLength, 2)) /
            (-2.0 * Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.bottomLength)
        )) + Constants.ActuatorSub.shooterMinAngle - Constants.ActuatorSub.bottomAngle;
    }

    public boolean onTarget() {
        if (Math.abs(currentAngle() - desiredAngle) < Constants.ActuatorSub.maxError) return true;
        else return false;
    }

    public void actuateToGoalAngle() {
        if (onTarget()) {
            actuatorMotor.stopMotor();
        } else {
            //actuatorMotor.set(Constants.ActuatorSub.actuatorPID.calculate(currentAngle(), getDesiredAngle()));
            actuatorMotor.set(Constants.ActuatorSub.actuatorPID.calculate(getMotorPosition(), goalAngleToPIDRotations()));
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
        if (getDesiredAngle() > Constants.ActuatorSub.maxDesiredAngle) {
            setDesiredAngle(Constants.ActuatorSub.maxDesiredAngle);
        }
        if (getDesiredAngle() < Constants.ActuatorSub.minDesiredAngle) {
            setDesiredAngle(Constants.ActuatorSub.minDesiredAngle);
        }

        actuateToGoalAngle();

        //SmartDashboard.putNumber("actuatorMotorPosition", getMotorPosition());
        //SmartDashboard.putNumber("desiredPosition", getDesiredAngle());

        //SmartDashboard.putNumber("UpPID", actuatorPID.calculate(getMotorPosition(), goalAngleToPIDRotations()));
        //SmartDashboard.putNumber("positionError", actuatorPID.getPositionError());
        //SmartDashboard.putNumber("velocityError", actuatorPID.getVelocityError());
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
