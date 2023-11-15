package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.autonomous.Autonomous;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.vision.Vision;

public class DriveToApriltag extends SequentialCommandGroup {
    private static final double TARGET_APRILTAG_HEIGHT = 0;

    private Trajectory generatedTrajectory;

    DriveToApriltag(Swerve swerve, short id)
    {
        double distance = Vision.getDistanceToApriltag(id, TARGET_APRILTAG_HEIGHT);
        double angle = Vision.getAngleToApriltag(id);
        
        double horizontialDistance = ((0 > angle) ? (-1) : (1))*(distance)*(Math.sin(angle));
        double verticalDistance = ((0 > angle) ? (-1) : (1))*(distance)*(Math.cos(angle));
        generatedTrajectory = Autonomous.buildLinearTrajectory(
            new Pose2d(
                horizontialDistance, // TODO: This might need to account for the initial pose of the robot as well. I'm not sure if this will mess up the odometry. 
                verticalDistance,
                new Rotation2d(angle)
            )
        );

        addCommands(
            new InstantCommand(
                () -> Odometry.reset(swerve.getModulePositions(), generatedTrajectory.getInitialPose())),
            swerve.buildAutonomousSwerveCommnad(generatedTrajectory)
        );
    }
}
