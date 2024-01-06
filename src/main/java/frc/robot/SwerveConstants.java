package frc.robot;

public class SwerveConstants {

    // CAN Ids go around the swerve drive counter-clockwise starting from the front left module
    /** CAN bus ids for the drive motors, in counter-clockwise order from the front left module */
    public static final int[] driveMotorIds = { 2, 3, 5, 7 };
    /** CAN bus ids for the angle motors, in counter-clockwise order from the front left module */
    public static final int[] angleMotorIds = { 1, 4, 6, 8 };
    /** CAN bus ids for the CANCoders, in counter-clockwise order from the front left module */
    public static final int[] canCoderIds = { 21, 22, 23, 24 };

    /** The length of the robot frame along the forward-backward axis */
    public static final double frameY = 26;
    /** The length of the robot frame along the left-right axis */
    public static final double frameX = 26;

    /** How far inside the frame the wheel touches the ground */
    public static final double wheelInset = 2.625;

    /** The diameter of the swerve module wheels */
    public static final double wheelDiameterInch = 4;

    /** The gear reduction ratio from the motor to the drive wheel */
    public static final double driveGearRatio = 6.75;
    /** The gear reduction ratio from the motor to the wheel heading */
    public static final double angleGearRatio = 150 / 7;

    /** The maximumn attainable speed of the swerve drive */
    public static final double maxSpeedFeetPerSecond = 15.1;


    /** The maximumn rotational speed when turning based on a percent input */
    public static final double percentTurnRateDegreesPerSecond = 180;


    /** The Drive Motor PID Controller values for velocity closed-loop control */
    public static final class DriveVelocityPid {
        
        public static final double p = 0.1;
        public static final double i = 0;
        public static final double d = 0;
    }

    /** The Angle Motor PID Controller values for position closed-loop control */
    public static final class AnglePositionPid {
        
        public static final double p = 0.1;
        public static final double i = 0;
        public static final double d = 0;
    }
}
