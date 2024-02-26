/*package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimatorSub extends SubsystemBase {

    //Declare motors and sensors
    private SwerveDrivePoseEstimator poseEstimator;
    private Pigeon2 gyro;
    
    public PoseEstimatorSub(SwerveSub SwerveSub) { //Subsystem constructor
        //Initialize motors and sensors
        gyro = new Pigeon2(Constants.PoseEstimatorSub.gyroID);
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, gyro.getYaw(), 
            SwerveSub.getModulePositions(), new Pose2d(0.0, 0.0, new Rotation2d(0)));
    }

    //Declare subsystem suppliers
    public boolean exampleSupplier() {
        return false;
    }

    //Declare methods
    public void exampleMethod() {}

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
*/