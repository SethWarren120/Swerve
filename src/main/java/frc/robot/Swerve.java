package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

    public static Swerve instance;
    
    SwerveMod[] modules;

    // A utility class that calculates physical values and properties of the swerve module 
    SwerveDriveKinematics kinematics;

    /**
     * Move translationally and rotate based on percentage inputs, with the translation being relative to the field / gyroscope heading
     * @param yPercent Percentage of Y (Forward - Backward) translation
     * @param xPercent Percentage of X (Left - Right) translation
     * @param turnPercent Percentage of rotation
     */
    public void drivePercentFieldRelative(double yPercent, double xPercent, double turnPercent) {        
        // Limit drive percentages to have a combined maximum percentage of 1 (100%)
        Translation2d translation = new Translation2d(xPercent, yPercent);
        translation = translation.div(Math.min(1 / translation.getNorm(), 1));

        // Generate the target states for each module based on the percentage movements
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                translation.getX() * UnitUtil.feetToMeters(SwerveConstants.maxSpeedFeetPerSecond), 
                translation.getY() * UnitUtil.feetToMeters(SwerveConstants.maxSpeedFeetPerSecond), 
                turnPercent * SwerveConstants.percentTurnRateDegreesPerSecond * 0.02), 
            new Rotation2d(Gyro.getHeading())));
            
        for (int i = 0; i < 4; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    /**
     * Move translationally and rotate based on percentage inputs, with the translation being relative to the direction of the robot
     * @param yPercent Percentage of Y (Forward - Backward) translation
     * @param xPercent Percentage of X (Left - Right) translation
     * @param turnPercent Percentage of rotation
     */
    public void drivePercentRobotRelative(double yPercent, double xPercent, double turnPercent) {
        // Limit drive percentages to have a combined maximum percentage of 1 (100%)
        Translation2d translation = new Translation2d(xPercent, yPercent);
        translation = translation.div(Math.min(1 / translation.getNorm(), 1));

        // Generate the target states for each module based on the percentage movements
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(translation.getX() * UnitUtil.feetToMeters(SwerveConstants.maxSpeedFeetPerSecond), 
            translation.getY() * UnitUtil.feetToMeters(SwerveConstants.maxSpeedFeetPerSecond), turnPercent * SwerveConstants.percentTurnRateDegreesPerSecond * 0.02));       
            
        for (int i = 0; i < 4; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    public Swerve() {
        // Set up the Kinematics class with the positions of each module wheel
        // Goes in counter-clockwise order starting from the front left module
        kinematics = new SwerveDriveKinematics(
            new Translation2d( ((SwerveConstants.frameX / 2) - SwerveConstants.wheelInset),  ((SwerveConstants.frameY / 2) - SwerveConstants.wheelInset)),
            new Translation2d( ((SwerveConstants.frameX / 2) - SwerveConstants.wheelInset), -((SwerveConstants.frameY / 2) - SwerveConstants.wheelInset)),
            new Translation2d(-((SwerveConstants.frameX / 2) - SwerveConstants.wheelInset), -((SwerveConstants.frameY / 2) - SwerveConstants.wheelInset)),
            new Translation2d(-((SwerveConstants.frameX / 2) - SwerveConstants.wheelInset),  ((SwerveConstants.frameY / 2) - SwerveConstants.wheelInset))
        );

        modules = new SwerveMod[4];

        // Set up each swerve module with the correct CAN ids
        modules[0] = new SwerveMod(SwerveConstants.driveMotorIds[0], SwerveConstants.angleMotorIds[0], SwerveConstants.canCoderIds[0]);
        modules[1] = new SwerveMod(SwerveConstants.driveMotorIds[1], SwerveConstants.angleMotorIds[1], SwerveConstants.canCoderIds[1]);
        modules[2] = new SwerveMod(SwerveConstants.driveMotorIds[2], SwerveConstants.angleMotorIds[2], SwerveConstants.canCoderIds[2]);
        modules[3] = new SwerveMod(SwerveConstants.driveMotorIds[3], SwerveConstants.angleMotorIds[3], SwerveConstants.canCoderIds[3]);

        // Getting the swerve section hardware configuration shuffleboard tab
        // ShuffleboardLayout swerveConfig = Shuffleboard.getTab("Hardware Config").getLayout("Swerve");

        // // Getting the sections for each modules within the general swerve section
        // ShuffleboardLayout flConfig = swerveConfig.getLayout("Front Left" , BuiltInLayouts.kList);
        // ShuffleboardLayout frConfig = swerveConfig.getLayout("Front Right", BuiltInLayouts.kList);
        // ShuffleboardLayout blConfig = swerveConfig.getLayout("Back Left"  , BuiltInLayouts.kList);
        // ShuffleboardLayout brConfig = swerveConfig.getLayout("Back Right" , BuiltInLayouts.kList);

        // // Adding the hardware configuration commands from each module
        // flConfig.add((Sendable) modules[0].zeroAngleEncoderCommand()   );
        // flConfig.add((Sendable) modules[0].checkAngleInversionCommand());
        // flConfig.add((Sendable) modules[0].checkDriveInversionCommand());

        // blConfig.add((Sendable) modules[1].zeroAngleEncoderCommand()   );
        // blConfig.add((Sendable) modules[1].checkAngleInversionCommand());
        // blConfig.add((Sendable) modules[1].checkDriveInversionCommand());

        // brConfig.add((Sendable) modules[2].zeroAngleEncoderCommand()   );
        // brConfig.add((Sendable) modules[2].checkAngleInversionCommand());
        // brConfig.add((Sendable) modules[2].checkDriveInversionCommand());

        // frConfig.add((Sendable) modules[3].zeroAngleEncoderCommand()   );
        // frConfig.add((Sendable) modules[3].checkAngleInversionCommand());
        // frConfig.add((Sendable) modules[3].checkDriveInversionCommand());

        if (instance == null)
            instance = this;
    }
}
