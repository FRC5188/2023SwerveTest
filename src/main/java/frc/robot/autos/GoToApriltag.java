package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.vision.Vision;

public class GoToApriltag extends SequentialCommandGroup {
    private static final double TARGET_HEIGHT = 0;

    public GoToApriltag(Swerve s_Swerve, short id) {
        Vision vision = new Vision();
        Odometry _swerveOdometry = s_Swerve.getSwerveOdometry(); 

        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
    
        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(0, 0), new Translation2d(0, 0)), // TODO: See if this can be null, right now just set to zero. 

                new Pose2d(
                    Odometry.getPose().getX() + (Vision.getDistanceToApriltag(id, TARGET_HEIGHT) * Math.sin(Vision.getAngleToApriltag(id))), 
                    (Vision.getAngleToApriltag(id) > 0) ? (Odometry.getPose().getX() + (Vision.getDistanceToApriltag(id, TARGET_HEIGHT) * Math.cos(Vision.getAngleToApriltag(id)))) : (Odometry.getPose().getX() - (Vision.getDistanceToApriltag(id, TARGET_HEIGHT) * Math.cos(Vision.getAngleToApriltag(id)))), 
                    new Rotation2d(Vision.getAngleToApriltag(id))), // TODO: Assume yaw left is negative and right is positive. 
                    config
                );
                // TODO: Make sure this is in rads. 
    
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                Odometry::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        
        addCommands(
            new InstantCommand(() -> _swerveOdometry.reset(s_Swerve.getModulePositions(), exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}
