package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_ControlFrame;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

public class Gyro {
    
    static AHRS navX;

    static Pigeon2 pigeon;

    public static double getHeading() {
        // Return the heading from 0 - 360
        
        return pigeon.getYaw();

        // TODO implement pigeon failure detection, and switch to NavX on failure. If both fail, use estimation from swerve odometry
        // return (navX.getYaw() + 360) % 360;
    }

    public Gyro() {
        navX = new AHRS(Port.kOnboard, (byte) 50);
        pigeon = new Pigeon2(31);

        pigeon.setControlFramePeriod(PigeonIMU_ControlFrame.Control_1, 20);
    }
}
