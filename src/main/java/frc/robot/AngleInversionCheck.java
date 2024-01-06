package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AngleInversionCheck extends CommandBase {
    
    double angleInitial;
    double angleAbsoluteInitial;

    CANSparkMax angleController;
    RelativeEncoder relativeEncoder;
    CANCoder absoluteEncoder;

    double absoluteOffset;

    public AngleInversionCheck(CANSparkMax angle, CANCoder absolute) {
        angleController = angle;
        relativeEncoder = angle.getEncoder();
        absoluteEncoder = absolute;
    }

    @Override
    public void initialize() {
        absoluteEncoder.configSensorDirection(false);
        angleController.setInverted(false);

        absoluteOffset = absoluteEncoder.configGetMagnetOffset();
        absoluteEncoder.configMagnetOffset(absoluteEncoder.configGetMagnetOffset() + absoluteEncoder.getPosition() + 180);

        relativeEncoder.setPosition(180);

        angleInitial = relativeEncoder.getPosition();
        angleAbsoluteInitial = absoluteEncoder.getPosition();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(absoluteEncoder.getPosition() - angleAbsoluteInitial) >= 30;
    }

    @Override
    public void end(boolean interrupted) {

        absoluteEncoder.configMagnetOffset(absoluteOffset);

        absoluteEncoder.configSensorDirection((absoluteEncoder.getPosition() - angleAbsoluteInitial) < 0);
        angleController.setInverted((relativeEncoder.getPosition() - angleInitial) < 0);

        relativeEncoder.setPosition(absoluteEncoder.getPosition());
    }
}
