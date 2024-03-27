package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveModule;
import frc.robot.Constants;

public class SwerveSub extends SubsystemBase {;
    public SwerveModule[] mSwerveMods;
    PoseEstimatorSub poseEstimatorSub;

    public PathPlannerPath scoreAmp;
    public PathConstraints scoreAmpConstraints;

    public PIDController swerveRotationPID;
    public PIDController swerveTranslationPID;

    public SwerveSub(PoseEstimatorSub poseEstimatorSub) {
        this.poseEstimatorSub = poseEstimatorSub;

        swerveRotationPID = Constants.Swerve.swerveRotationPID;
        swerveRotationPID.enableContinuousInput(-180, 180);

        swerveTranslationPID = Constants.Swerve.swerveTranslationPID;

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        poseEstimatorSub.initialize(this);

        AutoBuilder.configureHolonomic(
                poseEstimatorSub::getPose, //Robot pose supplier
                poseEstimatorSub::setPose, //Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, //ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::pathPlannerDrive, //Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                Constants.PathPlanner.pathPlannerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        scoreAmp = PathPlannerPath.fromPathFile("Score Amp");
        scoreAmpConstraints = new PathConstraints(
            2.0, 2.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)
        );

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() * Constants.Swerve.translationSensitivity, 
                                    translation.getY() * Constants.Swerve.translationSensitivity, 
                                    rotation * Constants.Swerve.rotationSensitivity, 
                                    poseEstimatorSub.getHeadingFieldOriented()
                                )
                                : new ChassisSpeeds(
                                    translation.getX() * Constants.Swerve.translationSensitivity, 
                                    translation.getY() * Constants.Swerve.translationSensitivity, 
                                    rotation * Constants.Swerve.rotationSensitivity)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public boolean intakeDrive(Translation2d translation, double rotation) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(
                    translation.getX() * Constants.Swerve.translationSensitivity, 
                    translation.getY() * Constants.Swerve.translationSensitivity, 
                    swerveRotationPID.calculate(poseEstimatorSub.getPose().getRotation().getDegrees(), rotation)
                ));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }

        if (Math.abs(poseEstimatorSub.getPose().getRotation().getDegrees() - rotation) < Constants.Swerve.maxIntakeError) return true;
        else return false;
    }

    public boolean driveWithRotationGoal(Translation2d translation, double rotation) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() * Constants.Swerve.translationSensitivity, 
                                    translation.getY() * Constants.Swerve.translationSensitivity, 
                                    swerveRotationPID.calculate(poseEstimatorSub.getPose().getRotation().getDegrees(), rotation), 
                                    poseEstimatorSub.getHeadingFieldOriented()
                                ));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }

        if (Math.abs(poseEstimatorSub.getPose().getRotation().getDegrees() - poseEstimatorSub.getTargetYaw()) < Constants.Swerve.maxError) return true;
        else return false;
    }

    public boolean ampDrive() {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(
                    swerveTranslationPID.calculate(poseEstimatorSub.getAmpTY(), 0), 
                    swerveTranslationPID.calculate(poseEstimatorSub.getAmpTX(), 0), 
                    swerveRotationPID.calculate(poseEstimatorSub.getPose().getRotation().getDegrees(), 90)
                ));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }

        if (Math.abs(poseEstimatorSub.getPose().getRotation().getDegrees() - 90) < 5 &&
            Math.abs(poseEstimatorSub.getAmpTX()) < 4 &&
            Math.abs(poseEstimatorSub.getAmpTY()) < 4
        ) return true;
        else return false;
    }

    public void pathPlannerDrive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = 
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    //Get ChassisSpeeds robot relative, used by PathPlanner
    public ChassisSpeeds getRobotRelativeSpeeds(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        ChassisSpeeds chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(states[0], states[1], states[2], states[3]);
        return chassisSpeeds;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        /*for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }*/
    }
}