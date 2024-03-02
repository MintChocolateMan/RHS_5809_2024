package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class PoseEstimatorSub extends SubsystemBase {

    //Declare motors and sensors
    Pigeon2 gyro;
    SwerveDrivePoseEstimator poseEstimator;
    PhotonCamera shooterCam;
    Transform3d robotToShooterCam;
    SwerveSub swerveSub;
    
    public PoseEstimatorSub() { //Subsystem constructor
        //Initialize motors and sensors
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        robotToShooterCam = new Transform3d(
            new Translation3d(
                Constants.CameraSub.shooterCamForwardOffset,
                Constants.CameraSub.shooterCamHorizontalOffset,
                Constants.CameraSub.shooterCamVerticalOffset
            ), new Rotation3d(
                Constants.CameraSub.shooterCamYaw,
                Constants.CameraSub.shooterCamRoll,
                Constants.CameraSub.shooterCamPitch
            )
        );

    }

    //Declare subsystem suppliers
    public void initialize(SwerveSub swerveSub) {
        this.swerveSub = swerveSub;

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            swerveSub.getModulePositions(),
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0))
        );
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
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
        poseEstimator.update(getGyroYaw(), swerveSub.getModulePositions());
    }

    @Override //This method is called continuously during simulation
    public void simulationPeriodic() {}
}
