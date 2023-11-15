package frc.robot.subsystems.odometry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;

// TODO: This class uses both SwerveDrivePoseEstimator and SwerveDriveOdometry though they both do the same thing. 

public abstract class Odometry {

    // Instantialize the gyroscope and odometry which will be actually set up in the build function. 
    private static final AHRS gyroscope = new AHRS();
    private static SwerveDriveOdometry swerveOdometry;
    private static SwerveDrivePoseEstimator swervePoseEstimator = null;

    /**
     * Should be used while setting up the odometry system and resets their position to an intial position. 
     * @param swerveModulePositions
     * @param pose
     */
    public static void reset(SwerveModulePosition[] swerveModulePositions, Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), swerveModulePositions, pose);
    }

    public static void zeroGyro(){
        gyroscope.reset();
    }

    public static void build(SwerveModulePosition[] swerveModulePositions)
    {
        zeroGyro();
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), swerveModulePositions);
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), swerveModulePositions, getPose());
    }

    public static void build(SwerveModulePosition[] swerveModulePositions, Pose2d initalPosition)
    {
        Odometry.build(swerveModulePositions);
        Odometry.reset(swerveModulePositions, initalPosition);
        // If this method is run we need to recreate the object using the correct, updated pose. 
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), swerveModulePositions, getPose());
    }

    public static Rotation2d getYaw() {
        // If we've choosen for the gyroscope to be inverted then we must adjust through rotating the yaw given by the gyroscope. 
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyroscope.getYaw()) : Rotation2d.fromDegrees(gyroscope.getYaw());
    }

    public static Pose2d getPose() {
        if (swervePoseEstimator == null) {
            return swerveOdometry.getPoseMeters();
        }
        else 
        {
            return swervePoseEstimator.getEstimatedPosition();
        }
    }

    /**
     * Should be used <strong>while the robot is operating</strong> and updates the position of the swerve modules for odometry calculations. 
     * @param swerveModulePositions 
     */
    public static void update(SwerveModulePosition[] swerveModulePositions) {
        swervePoseEstimator = Vision.getVisionEstimatedRobotPose(swervePoseEstimator);
        swervePoseEstimator.update(getYaw(), swerveModulePositions);
        swerveOdometry.update(getYaw(), swerveModulePositions);
    }
}