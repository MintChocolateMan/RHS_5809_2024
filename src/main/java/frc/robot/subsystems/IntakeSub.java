package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSub extends SubsystemBase {

    //Declare motors and sensors
    private TalonFX intakeMotor;
    private DigitalInput lineBreaker;
    
    public IntakeSub() { //Subsystem constructor
        //Initialize motors and sensors
        intakeMotor = new TalonFX(Constants.IntakeSub.intakeMotorID);
        intakeMotor.setInverted(Constants.IntakeSub.intakeMotorReversed);
        lineBreaker = new DigitalInput(Constants.IntakeSub.lineBreakerID);
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
        intakeMotor.set(-Constants.IntakeSub.intakeMotorSpeed);
    }

    public void intakeMotorOff() {
        intakeMotor.set(0);
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
