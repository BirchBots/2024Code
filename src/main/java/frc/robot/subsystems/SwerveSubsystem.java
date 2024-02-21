// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveModule flModule = new SwerveModule(kFlDrive, kFlTurn, kFlOffset);
  private SwerveModule frModule = new SwerveModule(kFrDrive, kFrTurn, kFrOffset);
  private SwerveModule blModule = new SwerveModule(kBlDrive, kBlTurn, kBlOffset);
  private SwerveModule brModule = new SwerveModule(kBrDrive, kBrTurn, kBrOffset);

  public SwerveSubsystem() {}
    
  public void stopModules() {
    flModule.stop();
    frModule.stop();
    blModule.stop();
    brModule.stop();
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxMeterPerSec);
    blModule.setDesiredState(states[0]);
    brModule.setDesiredState(states[1]);
    flModule.setDesiredState(states[2]);
    frModule.setDesiredState(states[3]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("flDesired ", flModule.desired.angle.getRadians());
    SmartDashboard.putNumber("flMotor", flModule.turnMotor.getAppliedOutput());
    double[] states = {
      flModule.getState().angle.getRadians(),
      flModule.getState().speedMetersPerSecond,
      frModule.getState().angle.getRadians(),
      frModule.getState().speedMetersPerSecond,
      blModule.getState().angle.getRadians(),
      blModule.getState().speedMetersPerSecond,
      brModule.getState().angle.getRadians(),
      brModule.getState().speedMetersPerSecond
    };
    SmartDashboard.putNumberArray("states", states);
  }
}