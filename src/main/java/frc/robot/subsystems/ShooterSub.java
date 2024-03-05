package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterSub extends SubsystemBase {

    private TalonFX topMotor;
    private TalonFX bottomMotor;
    
    public ShooterSub() {
        topMotor = new TalonFX(Constants.ShooterSub.topMotorID);
        bottomMotor = new TalonFX(Constants.ShooterSub.bottomMotorID);
        topMotor.setNeutralMode(NeutralModeValue.Brake);
        bottomMotor.setNeutralMode(NeutralModeValue.Brake);
        topMotor.setInverted(Constants.ShooterSub.topMotorInverted);
        bottomMotor.setInverted(Constants.ShooterSub.bottomMotorInverted);
    }

    //Declare methods
    public void shooterMotorsOn() {
        topMotor.set(Constants.ShooterSub.shootSpeed);
        bottomMotor.set(Constants.ShooterSub.shootSpeed);
    }

    public void shooterMotorsReverse() {
        topMotor.set(-Constants.ShooterSub.reverseSpeed);
        bottomMotor.set(-Constants.ShooterSub.reverseSpeed);
    }
    
    public void shooterMotorsOff() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}
}
