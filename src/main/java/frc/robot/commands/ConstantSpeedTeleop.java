package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
//import java.util.function.DoubleSupplier;

//import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ConstantSpeedTeleop extends CommandBase {    
    private Swerve s_Swerve;    
    /*private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;*/
    private BooleanSupplier robotCentricSup;

    public ConstantSpeedTeleop (Swerve s_Swerve, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        /*this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;*/
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = 1;
        double strafeVal = 0;
        double rotationVal = 0;

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(1), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            false
        );
    }
    
}