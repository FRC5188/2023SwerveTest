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

public class TurningTowardApriltag extends SequentialCommandGroup {

    private Trajectory GENERATED_TRAJECTORY;

    public TurningTowardApriltag(Swerve swerve, short id)
    {
        GENERATED_TRAJECTORY = Autonomous.buildLinearTrajectory(
        new Pose2d(0, 0, new Rotation2d(Vision.getAngleToApriltag(id)))
        );

        addCommands(
            new InstantCommand(
                () -> Odometry.reset(swerve.getModulePositions(), GENERATED_TRAJECTORY.getInitialPose())),
            swerve.buildAutonomousSwerveCommnad(GENERATED_TRAJECTORY)
        );
    }
}
