package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticSub extends SubsystemBase {

    //Declare motors and sensors
    private PneumaticHub pneumaticHub;
    private Solenoid leftClimberForward;
    private Solenoid leftClimberReverse;
    private Solenoid rightClimberForward;
    private Solenoid rightClimberReverse;
    
    public PneumaticSub() { //Subsystem constructor
        //Initialize motors and sensors
        pneumaticHub = new PneumaticHub(Constants.PneumaticSub.pneumaticHubID);
        
        leftClimberForward = pneumaticHub.makeSolenoid(Constants.PneumaticSub.leftClimberReverseID);
        leftClimberReverse = pneumaticHub.makeSolenoid(Constants.PneumaticSub.leftClimberReverseID);
        rightClimberForward = pneumaticHub.makeSolenoid(Constants.PneumaticSub.rightClimberForwardID);
        rightClimberReverse = pneumaticHub.makeSolenoid(Constants.PneumaticSub.rightClimberReverseID);

        leftClimberForward.setPulseDuration(0.5);
        leftClimberReverse.setPulseDuration(0.5);
        rightClimberForward.setPulseDuration(0.5);
        rightClimberReverse.setPulseDuration(0.5);
    }

    //Declare subsystem suppliers

    //Declare methods
    public void climbersUp() {
        leftClimberForward.startPulse();
        rightClimberForward.startPulse();
    }

    public void climbersDown() {
        leftClimberReverse.startPulse();
        rightClimberReverse.startPulse();
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
