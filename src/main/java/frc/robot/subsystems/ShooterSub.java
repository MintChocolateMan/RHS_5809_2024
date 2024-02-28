package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSub extends SubsystemBase {

    //Declare motors and sensors
    private TalonFX topMotor;
    private TalonFX bottomMotor;
    
    public ShooterSub() { //Subsystem constructor
        //Initialize motors and sensors
        topMotor = new TalonFX(Constants.ShooterSub.topMotorID);
        bottomMotor = new TalonFX(Constants.ShooterSub.bottomMotorID);
        topMotor.getConfigurator().apply(Robot.ctreConfigs.shooterFXConfig);
        bottomMotor.getConfigurator().apply(Robot.ctreConfigs.shooterFXConfig);
        topMotor.setInverted(Constants.ShooterSub.topMotorInverted);
        bottomMotor.setInverted(Constants.ShooterSub.bottomMotorInverted);
    }

    //Declare subsystem suppliers

    //Declare methods
    public void shooterShoot() {
        topMotor.set(Constants.ShooterSub.shootSpeed);
        bottomMotor.set(Constants.ShooterSub.shootSpeed);
    }

    public void shooterIntake() {
        topMotor.set(-Constants.ShooterSub.intakeSpeed);
        bottomMotor.set(-Constants.ShooterSub.intakeSpeed);
    }
    
    public void stopMotors() {
        topMotor.set(0);
        bottomMotor.set(0);
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
