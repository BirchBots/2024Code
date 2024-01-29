// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final boolean useController = false;
  private final Joystick rightStick = new Joystick(1);
  private final XboxController controller = new XboxController(0);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public RobotContainer() {
    if (useController) {
      swerveSubsystem.setDefaultCommand(new SwerveCmd(swerveSubsystem, 
      () -> -controller.getRawAxis(1), () -> -controller.getRawAxis(0), () -> controller.getRawAxis(4), () -> controller.getLeftBumper()));
    } else {
      swerveSubsystem.setDefaultCommand(new SwerveCmd(swerveSubsystem,
      () -> -controller.getRawAxis(1), () -> -controller.getRawAxis(0), () -> rightStick.getRawAxis(0), () -> controller.getRawButton(1)));
    }
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
