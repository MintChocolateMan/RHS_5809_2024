package frc.robot.subsystems;

import frc.robot.Constants;

import java.math.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix6.hardware.TalonFX;

public class ActuatorSub extends SubsystemBase {

    //Declare motors and sensors
    TalonFX actuatorMotor;
    PIDController actuatorPID;

    double desiredAngle = 0;
    
    public ActuatorSub() { //Subsystem constructor
        //Initialize motors and sensors
        actuatorMotor = new TalonFX(Constants.ActuatorSub.actuatorMotorID);
        actuatorPID = new PIDController(
            Constants.ActuatorSub.actuatorkP,
            Constants.ActuatorSub.actuatorkI,
            Constants.ActuatorSub.actuatorkD
        );
        actuatorMotor.setPosition(-1);
    }

    //Declare subsystem methods
    public void setDesiredAngle(double desiredAngle) {
        this.desiredAngle = desiredAngle;
    }

    public double getMotorPosition() {
        return actuatorMotor.getPosition().getValueAsDouble();
    }

    public double desiredAngleToGoalAngle() {
        return desiredAngle - Constants.ActuatorSub.shooterMinAngle + Constants.ActuatorSub.bottomAngle;
    }

    public double goalAngleToPIDRotations() {
        return ( Math.sqrt(
            Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.shooterLength +
            Constants.ActuatorSub.bottomLength * Constants.ActuatorSub.bottomLength +
            -2.0 * Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.bottomLength *
            Math.cos(desiredAngleToGoalAngle() * Math.PI / 180.0)
        ) - Constants.ActuatorSub.actuatorMinLength) / (2.0 * Math.PI * Constants.ActuatorSub.actuatorRate);
    } 

    public void actuateToGoalAngle() {
        actuatorMotor.set(actuatorPID.calculate(getMotorPosition(), goalAngleToPIDRotations()));
    }
    
    //Declare inline Commands
    public Command a_setDesiredAngle(double desiredAngle) {
        return runOnce(() -> {
            setDesiredAngle(desiredAngle);
        });
    }

    @Override //This method is called continuously
    public void periodic() {
        if (desiredAngle > Constants.ActuatorSub.maxDesiredAngle) {
            desiredAngle = Constants.ActuatorSub.maxDesiredAngle;
        }
        if (desiredAngle < Constants.ActuatorSub.minDesiredAngle) {
            desiredAngle = Constants.ActuatorSub.minDesiredAngle;
        }

        //actuateToDesiredAngle();
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
