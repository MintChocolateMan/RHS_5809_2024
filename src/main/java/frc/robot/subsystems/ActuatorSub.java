package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public double getMotorPosition() {
        return actuatorMotor.getPosition().getValueAsDouble();
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

    public void actuateToGoalAngle() {
        actuatorMotor.set(-actuatorPID.calculate(getMotorPosition(), goalAngleToPIDRotations()));
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

        SmartDashboard.putNumber("actuatorMotorPosition", getMotorPosition());
        SmartDashboard.putNumber("desiredPosition", getDesiredAngle());
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
