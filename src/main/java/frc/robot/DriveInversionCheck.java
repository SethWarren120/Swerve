package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveInversionCheck extends CommandBase {
    double driveInitial;

    CANSparkMax driveController;
    RelativeEncoder driveEncoder;

    double absoluteOffset;

    public DriveInversionCheck(CANSparkMax drive) {
        driveController = drive;
        driveEncoder = drive.getEncoder();
    }

    @Override
    public void initialize() {
        driveController.setInverted(false);

        driveEncoder.setPosition(180);

        driveInitial = driveEncoder.getPosition();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driveEncoder.getPosition() - driveInitial) >= 30;
    }

    @Override
    public void end(boolean interrupted) {
        driveController.setInverted((driveEncoder.getPosition() - driveInitial) < 0);
    }
}
