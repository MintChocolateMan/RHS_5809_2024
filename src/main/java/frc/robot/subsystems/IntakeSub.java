package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSub extends SubsystemBase {

    //Declare motors and sensors
    private TalonFX intakeMotor;
    private DigitalInput lineBreaker;
    private PIDController intakePID;
    
    public IntakeSub() { //Subsystem constructor
        //Initialize motors and sensors
        intakeMotor = new TalonFX(Constants.IntakeSub.intakeMotorID);
        intakeMotor.setInverted(Constants.IntakeSub.intakeMotorReversed);
        intakeMotor.setPosition(Constants.IntakeSub.intakePIDGoal);
        lineBreaker = new DigitalInput(Constants.IntakeSub.lineBreakerID);
        intakePID = new PIDController(
            Constants.IntakeSub.intakekP,
            Constants.IntakeSub.intakekI,
            Constants.IntakeSub.intakekD
        );
    }

    //Declare subsystem suppliers
    public boolean getNoteLoaded() {
        return lineBreaker.get();
    }

    //Declare methods
    public void intakeMotorOn() {
        intakeMotor.set(Constants.IntakeSub.intakeMotorSpeed);
    }

    public void intakeMotorReverse() {
        intakeMotor.set(-Constants.IntakeSub.intakeMotorReverseSpeed);
    }

    public void intakeMotorOff() {
        intakeMotor.set(0);
    }

    public double getIntakeMotorPosition() {
        return intakeMotor.getPosition().getValueAsDouble();
    }

    public void resetIntakeMotorPosition() {
        intakeMotor.setPosition(0);
    }

    public void intakeMotorToPID() {
        intakeMotor.set(intakePID.calculate(getIntakeMotorPosition(), Constants.IntakeSub.intakePIDGoal));
    }

    //Declare inline Commands
    public Command IntakeOn() {
        return runOnce(() -> {
            intakeMotorOn();
        });
    }

    public Command IntakeOff() {
        return runOnce(() -> {
            intakeMotorOff();
        });
    }

    @Override //This method is called continuously
    public void periodic() {
        SmartDashboard.putBoolean("NOTE LOADED", getNoteLoaded());
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
