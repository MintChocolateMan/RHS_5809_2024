package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSub extends SubsystemBase {

    //Declare motors and sensors
    TalonFX topMotor;
    TalonFX bottomMotor;
    
    public ShooterSub() { //Subsystem constructor
        //Initialize motors and sensors
        TalonFX topMotor = new TalonFX(Constants.ShooterSub.topMotorID);
        TalonFX bottomMotor = new TalonFX(Constants.ShooterSub.bottomMotorID);
        topMotor.setInverted(Constants.ShooterSub.topMotorInverted);
        bottomMotor.setInverted(Constants.ShooterSub.bottomMotorInverted);
    }

    //Declare subsystem suppliers

    //Declare methods
    public void motorsForward() {
        topMotor.set(Constants.ShooterSub.shooterSpeed);
        bottomMotor.set(Constants.ShooterSub.shooterSpeed);
    }
    
    public void stopMotors() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    //Declare inline Commands
    public Command ExampleInlineCommand() {
        return runOnce(() -> {
        });
    }

    @Override //This method is called continuously
    public void periodic() {}

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
