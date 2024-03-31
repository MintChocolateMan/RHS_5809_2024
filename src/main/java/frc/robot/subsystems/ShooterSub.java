package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterSub extends SubsystemBase {

    private TalonFX topMotor;
    private TalonFX bottomMotor;
    
    public ShooterSub() {
        topMotor = new TalonFX(Constants.ShooterSub.topMotorID);
        bottomMotor = new TalonFX(Constants.ShooterSub.bottomMotorID);
        topMotor.getConfigurator().apply(Robot.ctreConfigs.shooterFXConfig);
        topMotor.getConfigurator().apply(Robot.ctreConfigs.shooterFXConfig);
        topMotor.setNeutralMode(NeutralModeValue.Brake);
        bottomMotor.setNeutralMode(NeutralModeValue.Brake);
        topMotor.setInverted(Constants.ShooterSub.topMotorInverted);
        bottomMotor.setInverted(Constants.ShooterSub.bottomMotorInverted);
    }

    //Declare methods
    public void shooterMotorsOn() {
        topMotor.setVoltage(Constants.ShooterSub.shootVoltage);
        bottomMotor.setVoltage(Constants.ShooterSub.shootVoltage);
    }

    public void shooterMotorsAmp() {
        topMotor.setVoltage(Constants.ShooterSub.ampVoltage);
        bottomMotor.setVoltage(Constants.ShooterSub.ampVoltage);
    }

    public void shooterMotorsReverse() {
        topMotor.setVoltage(-Constants.ShooterSub.reverseVoltage);
        bottomMotor.setVoltage(-Constants.ShooterSub.reverseVoltage);
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
