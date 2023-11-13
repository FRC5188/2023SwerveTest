package frc.robot.subsystems.odometry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

public class Odometry {

    // Instantialize the gyroscope and encoders. 
    private static final AHRS gyroscope = new AHRS();
    private static SwerveDriveOdometry swerveOdometry;


    /**
     * When given module position creates the system for odometry.  
     * @param modulePositions
     */
    public Odometry(SwerveModulePosition[] modulePositions)
    {
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), modulePositions);
    }

    /**
     * When given the module positions and the initial position it'll both create the odometry system and then update it to the new position. 
     * @param modulePositions
     * @param initalPosition
     */
    public Odometry(SwerveModulePosition[] modulePositions, Pose2d initalPosition)
    {
        this(modulePositions);
        this.reset(modulePositions, initalPosition);
    }

    /**
     * @return The yaw of the robot. 
     */
    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyroscope.getYaw()) : Rotation2d.fromDegrees(gyroscope.getYaw());
    }

    /**
     * Resets the yaw axis of the NavX. Since the gyroscope isn't dependent of the instance of the object we can make this static, accessing if before we create the object. 
     */
    public static void zeroGyro(){
        gyroscope.reset();
    }

    /**
     * The position of the robot on the field relative to where it was last zeroed. 
     * @return Pose2d of robot's position.
     */
    public static Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Updates the position of the robot based on it's module positions and it's global position. 
     * @param positions List of module positions. 
     * @param pose Pose2d representing the robot's position on the field. 
     */
    public void reset(SwerveModulePosition[] positions, Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), positions, pose);
    }

    /**
     * Updates the position of the robot but <STRONG> should be used while robot is operating </STRONG> as it uses the feedfowards of the robot along with other variables to account for
     * things that just resetting position doesn't.
     * @param positions
     */
    public void update(SwerveModulePosition[] positions) {
        swerveOdometry.update(getYaw(), positions);
    }
}