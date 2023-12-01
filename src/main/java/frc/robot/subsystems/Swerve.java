package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.odometry.Odometry;
import frc.robot.subsystems.vision.Vision;

public class Swerve extends SubsystemBase {
    private Field2d _field = new Field2d();

    private static final SwerveModule[] _swerveModules = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    // Makes the controller for rotational motion of the robot. 
    private static final ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, 
        Constants.AutoConstants.kThetaControllerConstraints);
    
    // Some setup for the robot before we use the subsystem. 
    static {
        Timer.delay(1.0);
        // Sets the module back to absolute before they are given to the odometry system so we're accurate. 
        for (SwerveModule module : _swerveModules) {
            module.resetToAbsolute();
        }

        thetaController.enableContinuousInput(-Math.PI, Math.PI); // Sets up the rotational controller for building commands. 
    }

    public Swerve() {
        Odometry.build(getModulePositions()); // We must build the depending systems. Since no pose is passed in we're assuming that we start at zero. 
        Vision.build();
    }

    private Field2d getField()
    {
        return _field;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    Odometry.getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : _swerveModules){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public SwerveControllerCommand buildAutonomousSwerveCommnad(Trajectory trajectory)
    {
        return new SwerveControllerCommand(
            trajectory, 
            Odometry::getPose, 
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), 
            thetaController,
            this::setModuleStates, 
            this);
    }

    @Override
    public void periodic(){
        Odometry.update(getModulePositions());  

        for(SwerveModule mod : _swerveModules){
            SmartDashboard.putNumber("Module " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Module " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Module " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Distance Apriltag", Vision.getDistanceToApriltag((short) 2, 1.26));
            SmartDashboard.putString("Pose", Odometry.getPose().toString());
        }
        SmartDashboard.putNumber("Gyroscope Angle", Odometry.getYaw().getDegrees());

        _field.setRobotPose(Odometry.getPose());
        SmartDashboard.putData("Field", _field);
    }

    private void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : _swerveModules){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    private SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : _swerveModules){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : _swerveModules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }
}