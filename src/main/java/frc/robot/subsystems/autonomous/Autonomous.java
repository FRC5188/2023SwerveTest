package frc.robot.subsystems.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public abstract class Autonomous {
    private final static TrajectoryConfig trajectoryConfiguration = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

    public static Trajectory buildLinearTrajectory(Pose2d end)
    {
        return buildLinearTrajectory(new Pose2d(0, 0, new Rotation2d(0)), end);
    }

    public static Trajectory buildLinearTrajectory(Pose2d beginning, Pose2d end)
    {
        return TrajectoryGenerator.generateTrajectory(
            beginning,
            // We'll just make these some random point half-way through the path resulting in a linear path. 
            List.of(new Translation2d(end.getX()/2, end.getY()/2)), // Since we assume starting at zero this math works out. 
            end,
            trajectoryConfiguration);
    }

    public static Trajectory buildTrajectory(List<Translation2d> middle, Pose2d end)
    {
        return buildTrajectory(new Pose2d(0, 0, new Rotation2d(0)), middle, end);
    }

    public static Trajectory buildTrajectory(Pose2d start, List<Translation2d> middle, Pose2d end)
    {
        return TrajectoryGenerator.generateTrajectory(
            start,
            middle,
            end,
            trajectoryConfiguration
        );
    }
}
