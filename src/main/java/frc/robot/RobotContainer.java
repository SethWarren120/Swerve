// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  public RobotContainer() {
    controller = new CommandXboxController(0);

    configureBindings();
  }

  CommandXboxController controller;

  private void configureBindings() {
    Swerve.instance.setDefaultCommand(Commands.run(() -> {
      Swerve.instance.drivePercentRobotRelative(controller.getLeftY(), controller.getLeftX(), controller.getRightX());
    }, Swerve.instance));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
