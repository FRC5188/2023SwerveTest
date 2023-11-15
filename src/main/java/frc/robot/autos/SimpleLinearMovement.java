package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.autonomous.Autonomous;
import frc.robot.subsystems.odometry.Odometry;

public class SimpleLinearMovement extends SequentialCommandGroup {
    private final double MOVE_X_METERS = 5;
    private final double MOVE_Y_METERS = 0;

    private final Trajectory GENERATED_TRAJECTORY = Autonomous.buildLinearTrajectory(new Pose2d(MOVE_X_METERS, MOVE_Y_METERS, new Rotation2d(0)));

    public SimpleLinearMovement(Swerve swerve)
    {
        addCommands(
            new InstantCommand(
                () -> Odometry.reset(swerve.getModulePositions(), GENERATED_TRAJECTORY.getInitialPose())),
            swerve.buildAutonomousSwerveCommnad(GENERATED_TRAJECTORY)
        );
    }
}
