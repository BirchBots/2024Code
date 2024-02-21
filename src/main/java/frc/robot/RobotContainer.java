package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.GamepadUtils;
import java.util.List;

public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -GamepadUtils.squareInput(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -GamepadUtils.squareInput(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -GamepadUtils.squareInput(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true,
                    false),
            m_robotDrive));
  }

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}