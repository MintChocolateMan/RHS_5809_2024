package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;

public class CameraSub extends SubsystemBase {

    //Declare motors and sensors
    PhotonCamera shooterCam = new PhotonCamera("shooterCamera");
    
    public CameraSub() { //Subsystem constructor
        //Initialize motors and sensors
        PhotonCamera shooterCam = new PhotonCamera("shooterCamera");
    }

    //Declare subsystem suppliers
    public boolean shooterCamHasTarget() {
        return shooterCam.getLatestResult().hasTargets();
    }

    public double shooterCamGetYaw() {
        return shooterCam.getLatestResult().getBestTarget().getYaw();
    }

    //Declare methods
    public void exampleMethod() {}

    //Declare inline Commands
    public Command ExampleInlineCommand() {
        return runOnce(() -> {
        });
    }

    @Override //This method is called continuously
    public void periodic() {
        if (shooterCamHasTarget()) {
            SmartDashboard.putNumber("Target Yaw:", shooterCamGetYaw());
        }
        
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
