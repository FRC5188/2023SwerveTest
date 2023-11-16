package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.output.Output;
import frc.robot.subsystems.odometry.Odometry;

public abstract class Vision {
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
    private static boolean isFieldOriented = false;
    private static boolean hasTarget = false;

    private static Optional<AprilTagFieldLayout> fieldLayout;
    static {
        try {
            fieldLayout = Optional.of(AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField());
        } catch (IOException exception) {
            fieldLayout = Optional.empty(); // If we can't load the map we just set it to be null.
            isFieldOriented = false; // Can't be field oreie
            Output.warning("Cannot Load Field. Vision Results Cannot Be Used.");
        }
    };

    // If the field layout couldn't be loaded then we just set this to empty and
    // move on. Else we set it up. This assure that
    // everything is working order meaning a null-check later in the program isn't
    // needed since it's checked here.
    private static Optional<PhotonPoseEstimator> poseEstimator = (!fieldLayout.isEmpty()) ? (Optional.of(
            new PhotonPoseEstimator(
                    fieldLayout.get(),
                    PoseStrategy.LOWEST_AMBIGUITY,
                    camera,
                    CAMERA_POSITION)))
            : (Optional.empty());

    public static void build() {
        // If we can't load in the field map then automatically assume we aren't in
        // field oriented mode.
        Vision.build(!fieldLayout.isEmpty());
    }

    public static void build(boolean isFieldOriented) {
        Vision.isFieldOriented = isFieldOriented;
    }

    public static double getAngleToApriltag(short id) throws NullPointerException {
        if (!isFieldOriented) // Then we know that we're not field-oriented.
        {
            return camera.getLatestResult().getBestTarget().getYaw();
        } else // We're field-oriented and just use odometry to get the angle.
        {
            // Is that tag even on the field?
            Pose2d apriltagPose = fieldLayout.get().getTagPose(id).isPresent()
                    ? (fieldLayout.get().getTagPose(id).get().toPose2d())
                    : null;
            // If we can't find the tag then throw an exception that should be handled in
            // the calling function.
            if (apriltagPose == null) {
                Output.error("Tag Couldn't Be Located. Ensure Apriltag ID is correct. ");
                throw new NullPointerException("Tag couldn't be found. Please ensure that ID for apriltag is correct.");
            }
            return Odometry.getPose().relativeTo(apriltagPose).getRotation().getDegrees();
        }
    }

    public static double getDistanceToApriltag(short id, double targetHeight) throws NullPointerException {
        if (!isFieldOriented) // Then we know that we're not field-oriented.
        {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_Z_FROM_ROBOT_CENTER,
                    targetHeight,
                    Units.degreesToRadians(CAMERA_PITCH),
                    Units.degreesToRadians(camera.getLatestResult().getBestTarget().getPitch()));
        } else // We're field-oriented and just use odometry to get the distance.
        {
            Pose2d apriltagPose = fieldLayout.get().getTagPose(id).isPresent()
                    ? (fieldLayout.get().getTagPose(id).get().toPose2d())
                    : null;
            // Null check to ensure that we aren't working with a null value.
            if (apriltagPose == null) {
                Output.error("Tag couldn't be found. Please ensure that ID for apriltag is correct.");
                throw new NullPointerException("Tag couldn't be found. Please ensure that ID for apriltag is correct.");
            }
            // Pretty sure this returns the distance betweeen the robot and the apriltag.
            return Odometry.getPose().relativeTo(apriltagPose).getTranslation()
                    .getDistance(apriltagPose.getTranslation());
        }
    }

    public static SwerveDrivePoseEstimator getVisionEstimatedRobotPose(SwerveDrivePoseEstimator poseEstimator) {
        PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
        if (target != null) {
            hasTarget = true;
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                    fieldLayout.get().getTagPose(target.getFiducialId()).get(), CAMERA_POSITION);
            poseEstimator.addVisionMeasurement(robotPose.toPose2d(), Timer.getFPGATimestamp());
            Output.print("Robot Pose Updated From Vision " + robotPose);
            return poseEstimator;
        } else {
            hasTarget = false;
            return poseEstimator;
        }
    }

    public static Pose3d getRobotInitialPose() throws NullPointerException {
        try {
            PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                    fieldLayout.get().getTagPose(target.getFiducialId()).get(), CAMERA_POSITION);
            return robotPose;
        } catch (Exception exception) {
            throw new NullPointerException("[ERROR] Please Start Robot Facing An Apriltag. Pose Cannot Be Determined.");
        }
    }

    private static Optional<EstimatedRobotPose> getEstimatedRobotGlobalPose(Pose2d previousEstimatedRobotPose) {
        poseEstimator.get().setReferencePose(previousEstimatedRobotPose);
        return poseEstimator.get().update();
    }

    public static boolean hasTarget() {
        return hasTarget;
    }
}