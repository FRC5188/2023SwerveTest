package frc.robot.subsystems.vision;
import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.output.Output;
import frc.robot.subsystems.odometry.Odometry;

public class Vision {
    private static final String CAMERA_NAME = "photoncamera";
    /**
     * Assume that you're looking at the robot from above it. In our code we treat
     * the robot like a single point in an XY-Plane. Where the front of the robot is
     * the
     * positive X, and where the left side of the robot is the negative Y.
     */
    // How far foward/backward the camera is from robot center.
    private static final double CAMERA_X_FROM_ROBOT_CENTER = 0.193;
    // How far left/right the camera is from robot center.
    private static final double CAMERA_Y_FROM_ROBOT_CENTER = 0.2794;
    // How far up/down the camera is from center if we look at robot from side in 3D
    // space.
    private static final double CAMERA_Z_FROM_ROBOT_CENTER = 0.375;

    private static final double CAMERA_ROLL = 0;
    private static final double CAMERA_PITCH = 0;
    private static final double CAMERA_YAW = Math.toRadians(-10.5);

    private static final Transform3d CAMERA_POSITION = new Transform3d(
        new Translation3d(CAMERA_X_FROM_ROBOT_CENTER, CAMERA_Y_FROM_ROBOT_CENTER, CAMERA_Z_FROM_ROBOT_CENTER),
        new Rotation3d(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW));
    
    private static final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);

    private static Optional<AprilTagFieldLayout> fieldLayout;
    static {
        try {fieldLayout = Optional.of(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField());
        } catch (IOException exception) {Output.warning("Cannot Load Field. Vision Results Cannot Be Used.");}
    };

    private static Optional<PhotonPoseEstimator> poseEstimator = Optional.of(
        new PhotonPoseEstimator(
            fieldLayout.get(),
            PoseStrategy.LOWEST_AMBIGUITY,
            camera,
            CAMERA_POSITION
        )
    );

    /**
     * Based on whether you'd like to use a field oriented approach, or the less reliable robot oriented approach, you're creating the vision object. 
     * @param isFieldOriented Whether we should be using an apriltag field layout or other methods of calculating position.
     */
    public Vision(boolean isFieldOriented){
        if(!isFieldOriented)
        {
            // If we don't want to use these we'll just set them to empty so we know not to use them. 
            poseEstimator = Optional.empty();
            fieldLayout = Optional.empty();
        }
        // In the other case then we know that they do in fact work and we can use them. Instead of having a file-wide true or false
        // indicating whether we can use them we can just check if they exist when we use them which is safer anyway. 
    }

    /**
     * When creating the vision object without parameters it'll check to see if the pose estimation function loaded correctly. If it did then 
     * it'll use field oriented vision, otherwise it'll use robot oriented vision.
     */
    public Vision(){ this(!(poseEstimator.isEmpty() || poseEstimator.equals(null))); }

    public static double getAngleToApriltag(short id) throws NullPointerException {
        if(poseEstimator.isEmpty()) // Then we know that we're not field-oriented. 
        {
            return camera.getLatestResult().getBestTarget().getYaw();
        }
        else // We're field-oriented and just use odometry to get the angle. 
        {
            Pose2d robotPose = Odometry.getPose();
            Pose2d apriltagPose = fieldLayout.get().getTagPose(id).isPresent()
                    ? ( fieldLayout.get().getTagPose(id).get().toPose2d())
                    : null;
            if (apriltagPose == null) {
                Output.warning("Tag Couldn't Be Located. Ensure Apriltag ID is correct. ");
                throw new NullPointerException("Tag couldn't be found. Please ensure that ID for apriltag is correct.");
            }
            return robotPose.relativeTo(apriltagPose).getRotation().getDegrees();
        }
    }

    public static double getDistanceToApriltag(short id, double targetHeight) throws NullPointerException {
        if(poseEstimator.isEmpty()) // Then we know that we're not field-oriented. 
        {
            return PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_Y_FROM_ROBOT_CENTER,
                                targetHeight,
                                Units.degreesToRadians(CAMERA_PITCH),
                                Units.degreesToRadians(camera.getLatestResult().getBestTarget().getPitch()));
        }
        else // We're field-oriented and just use odometry to get the distance. 
        {
            Pose2d robotPose = Odometry.getPose();
            Pose2d apriltagPose = fieldLayout.get().getTagPose(id).isPresent()
                    ? ( fieldLayout.get().getTagPose(id).get().toPose2d())
                    : null;
                  // Null check to ensure that we aren't working with a null value.
            if (apriltagPose == null) {
                Output.warning("Tag couldn't be found. Please ensure that ID for apriltag is correct.");
                throw new NullPointerException("Tag couldn't be found. Please ensure that ID for apriltag is correct.");
            }
            // Pretty sure this returns the distance betweeen the robot and the apriltag.
            return robotPose.relativeTo(apriltagPose).getTranslation().getDistance(apriltagPose.getTranslation());
        }
    }
}