// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(controller);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveCmd(swerveSubsystem, 
    () -> -controller.getRawAxis(1), () -> -controller.getRawAxis(0), () -> controller.getRawAxis(4), () -> controller.button(1).getAsBoolean()));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
