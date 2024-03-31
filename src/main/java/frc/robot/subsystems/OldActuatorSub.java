package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class OldActuatorSub extends SubsystemBase {

    //Declare motors, sensors, and variables
    TalonFX actuatorMotor;

    PIDController actuatorPID;
    

    double desiredAngle = 0; //Constants.ActuatorSub.defaultAngle;
    boolean zeroing = false;

    public OldActuatorSub() {
        actuatorMotor = new TalonFX(Constants.ActuatorSub.actuatorMotorID);
        actuatorMotor.setInverted(Constants.ActuatorSub.actuatorMotorInverted);
        actuatorMotor.setNeutralMode(NeutralModeValue.Brake);
        actuatorMotor.getConfigurator().apply(Robot.ctreConfigs.actuatorFXConfig);
        actuatorMotor.setPosition(0);

        actuatorPID = Constants.ActuatorSub.actuatorPID;
        actuatorPID.setIZone(Constants.ActuatorSub.actuatorIZone);
        actuatorPID.setIntegratorRange(-.1, .1);
    }

    //Declare subsystem methods
    public void setDesiredAngle(double desiredAngle) {
        this.desiredAngle = desiredAngle;
    }

    public double getDesiredAngle() {
        return desiredAngle;
    }

    public void actuatorMotorDown() {
        actuatorMotor.set(-Constants.ActuatorSub.actuatorDownSpeed);
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

    //currently unused
    public double desiredAngleToMotorPosition() {
        return ( Math.sqrt(
            Math.pow(Constants.ActuatorSub.shooterLength, 2) + Math.pow(Constants.ActuatorSub.bottomLength, 2) +
            -2.0 * Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.bottomLength *
            Math.cos(getDesiredAngle() - Constants.ActuatorSub.shooterMinAngle + Constants.ActuatorSub.bottomAngle * 
            Math.PI / 180.0)) - Constants.ActuatorSub.actuatorMinLength ) / 
            (2.0 * Math.PI * Constants.ActuatorSub.actuatorRate);
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

    public double getGravityFeedForward() {
        return Math.pow(
            Math.sqrt(Constants.ActuatorSub.actuatorkG) *
            (Math.cos(Math.PI / 180 * 
            (90.0 - (Constants.ActuatorSub.bottomAngle + (180.0 * Math.PI * Math.asin(
            (Constants.ActuatorSub.shooterLength / Math.sqrt(
            Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.shooterLength + 
            Constants.ActuatorSub.bottomLength * Constants.ActuatorSub.bottomLength - 2.0 *
            Constants.ActuatorSub.shooterLength * Constants.ActuatorSub.bottomLength * Math.cos(Math.PI / 180.0 * (
            getActuatorAngle() - Constants.ActuatorSub.shooterMinAngle + Constants.ActuatorSub.bottomAngle
            )))) * Math.sin(Math.PI / 180.0 * (
            getActuatorAngle() - Constants.ActuatorSub.shooterMinAngle + Constants.ActuatorSub.bottomAngle
            )))))))),
        Constants.ActuatorSub.actuatorpG);
    }

    public boolean onTarget() {
        if (Math.abs(getActuatorAngle() - getDesiredAngle()) < Constants.ActuatorSub.maxError) return true;
        else return false;
    }

    public void setZeroing(boolean zeroing) {
        this.zeroing = zeroing;
    }

    public boolean getZeroing() {
        return zeroing;
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
        if (!getZeroing()) {
            if (Constants.ActuatorSub.actuatorPID.calculate(getActuatorAngle(), getDesiredAngle()) > 0) {
                actuatorMotor.set(
                    actuatorPID.calculate(getActuatorAngle(), getDesiredAngle()) + 
                    getGravityFeedForward());
            } else {
                actuatorMotor.set(
                    actuatorPID.calculate(getActuatorAngle(), getDesiredAngle()) + 
                    getGravityFeedForward() - Constants.ActuatorSub.actuatorDownkF);
            }
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

        //SmartDashboard.putNumber("actuatorFF", getFeedForward());

        //SmartDashboard.putNumber("actuatorMotorPosition", getMotorPosition());
        SmartDashboard.putNumber("desiredAngle", getDesiredAngle());
        SmartDashboard.putNumber("currentAngle", getActuatorAngle());
        //SmartDashboard.putNumber("desiredToMotor", desiredAngleToMotorPosition());
        //SmartDashboard.putNumber("motorPosition", getMotorPosition());
        

        //SmartDashboard.putNumber("UpPID", actuatorPID.calculate(getMotorPosition(), goalAngleToPIDRotations()));
        //SmartDashboard.putNumber("positionError", actuatorPID.getPositionError());
        //SmartDashboard.putNumber("velocityError", actuatorPID.getVelocityError());
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
