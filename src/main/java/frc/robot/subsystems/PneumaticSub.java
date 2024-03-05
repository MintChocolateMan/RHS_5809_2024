package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PneumaticSub extends SubsystemBase {

    private PneumaticHub pneumaticHub;
    private DoubleSolenoid leftClimber;
    private DoubleSolenoid rightClimber;
    
    public PneumaticSub() { 
        pneumaticHub = new PneumaticHub(Constants.PneumaticSub.pneumaticHubID);
        
        leftClimber = pneumaticHub.makeDoubleSolenoid(Constants.PneumaticSub.leftClimberForwardID, Constants.PneumaticSub.leftClimberReverseID);
        rightClimber = pneumaticHub.makeDoubleSolenoid(Constants.PneumaticSub.rightClimberForwardID, Constants.PneumaticSub.rightClimberReverseID);

        leftClimber.set(Value.kOff);
        rightClimber.set(Value.kOff);
    }

    public void climbersUp() {
        leftClimber.set(Value.kForward);
        rightClimber.set(Value.kForward);
    }

    public void climbersDown() {
        leftClimber.set(Value.kReverse);
        rightClimber.set(Value.kReverse);
    }

    public void climbersOff() {
        leftClimber.set(Value.kOff);
        rightClimber.set(Value.kOff);
    }

    @Override 
    public void periodic() {}

    @Override 
    public void simulationPeriodic() {}
}
