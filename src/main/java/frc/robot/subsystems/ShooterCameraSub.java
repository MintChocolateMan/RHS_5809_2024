package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;

public class ShooterCameraSub extends SubsystemBase {
  PhotonCamera driverCamera = new PhotonCamera("driverCamera");// = new PhotonCamera("driverCamera");

  public ShooterCameraSub() {

    PhotonCamera driverCamera = new PhotonCamera("driverCamera");

  }

  public boolean driverCameraHasTarget() {
    return driverCamera.getLatestResult().hasTargets();
  }

  public double driverCameraGetYaw() {
    return driverCamera.getLatestResult().getBestTarget().getYaw();
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("TargetYaw ", driverCameraGetYaw());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
