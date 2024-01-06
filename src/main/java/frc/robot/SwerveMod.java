package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveMod extends SubsystemBase {
    
    /** The SparkMax controller for the drive motor */
    CANSparkMax drive;
    /** The SparkMax controller for the angle motor */
    CANSparkMax angle;

    /** The relative encoder on the drive motor */
    RelativeEncoder driveEncoder;
    /** The relative encoder on the angle motor */
    RelativeEncoder angleEncoder;
    /** The absolute encoder on the angle axis */
    CANCoder angleEncoderAbsolute;

    // Using the SparkMAX closed-loop control mode
    /** The closed-loop control mode PID controller for the drive motor */
    SparkMaxPIDController drivePid;
    /** The closed-loop control mode PID controller for the angle motor */
    SparkMaxPIDController anglePid;

    /** The target state for the swerve module */
    SwerveModuleState targetState = new SwerveModuleState(0, new Rotation2d(0));

    /**
     * Sets the target state for the swerve module
     * @param state The target state for the swerve module
     */
    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, new Rotation2d(angleEncoder.getPosition()));
    }
    
    /**
     * Gets the current target state of the module
     * @return The target state last set by {@link #setTargetState(SwerveModuleState)}
     */
    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Gets the current physical state of the swerve module
     * @return The current physical state of the swerve module as measured by the relative encoders in the drive and angle motors
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d((angleEncoder.getPosition() / 360) * 2 * Math.PI));
    }

    @Override
    public void periodic() {
        // Pass both motor controllers the desired states and the closed-loop control mode of those states
        drivePid.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity);
        anglePid.setReference(targetState.angle.getDegrees(), ControlType.kPosition);
    }

    public SwerveMod(int driveMotorId, int angleMotorId, int angleEncoderId) {
        drive = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        angle = new CANSparkMax(angleMotorId, MotorType.kBrushless);

        driveEncoder = drive.getEncoder();
        angleEncoder = angle.getEncoder();

        angleEncoderAbsolute = new CANCoder(angleEncoderId);
        
        // Configure encoders to use the correct range
        angleEncoderAbsolute.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        // TODO add relative encoder config once figured out

        // Configure angle encoders to read wheel rotation from 0 - 360 degrees
        angleEncoder.setPositionConversionFactor((1 / SwerveConstants.angleGearRatio) * 360);
        angleEncoder.setVelocityConversionFactor((1 / SwerveConstants.angleGearRatio) * 360);

        // Zero the relative angle encoder based on the absolute encoder
        // Assumes that the absolute encoder is set to read back 0 - 360
        angleEncoder.setPosition(angleEncoderAbsolute.getPosition());

        // Configure drive encoders to read wheel position in meters
        // Conversion to meters here is (Pi * Diameter), not (2 * Pi * Radius)
        driveEncoder.setPositionConversionFactor((1 / SwerveConstants.driveGearRatio) * Math.PI * UnitUtil.inchesToMeters(SwerveConstants.wheelDiameterInch));
        driveEncoder.setVelocityConversionFactor((1 / SwerveConstants.driveGearRatio) * Math.PI * UnitUtil.inchesToMeters(SwerveConstants.wheelDiameterInch));

        // Set encoder update rates to match the tick rate of the robot code
        driveEncoder.setMeasurementPeriod(20);
        angleEncoder.setMeasurementPeriod(20);
        angleEncoderAbsolute.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);

        // Setting up closed-loop control
        // Closed-loop control only requires getting the PID
        drivePid = drive.getPIDController();
        anglePid = angle.getPIDController();

        // Set up the PID controllers for the closed loop control modes
        drivePid.setP(SwerveConstants.DriveVelocityPid.p);
        drivePid.setI(SwerveConstants.DriveVelocityPid.i);
        drivePid.setD(SwerveConstants.DriveVelocityPid.d);
        drivePid.setOutputRange(-1, 1);

        anglePid.setP(SwerveConstants.AnglePositionPid.p);
        anglePid.setP(SwerveConstants.AnglePositionPid.i);
        anglePid.setP(SwerveConstants.AnglePositionPid.d);
        anglePid.setOutputRange(-1, 1);
    }

    /**
     * A command that will set the zero position for the angle of the swerve module to the current physical position of the module
     * @return A command to zero the encoder
     */
    public Command zeroAngleEncoderCommand() {
        return Commands.runOnce(() -> {
            angleEncoderAbsolute.configMagnetOffset(angleEncoderAbsolute.configGetMagnetOffset() - angleEncoderAbsolute.getPosition());
        });
    }

    /**
     * A command that will detect inversions in the angle encoders on the module and invert where necessary
     * @return A command to check and correct inversions on the angle encoders
     */
    public Command checkAngleInversionCommand() {
        return new AngleInversionCheck(angle, angleEncoderAbsolute);
    }

    /**
     * A command that will detect inversion in the drive motor on the module and invert if necessary
     * @return A command to check and correct inversion on the drive motor
     */
    public Command checkDriveInversionCommand() {
        return new DriveInversionCheck(drive);
    }
}
