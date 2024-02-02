// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.*;
import static frc.robot.Constants.SwerveConstants.*;
import static java.lang.Math.*;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveModule flModule = new SwerveModule(kFlDrive, kFlTurn, kFlEncoder, kFlOffset);
  private SwerveModule frModule = new SwerveModule(kFrDrive, kFrTurn, kFrEncoder, kFrOffset);
  private SwerveModule blModule = new SwerveModule(kBlDrive, kBlTurn, kBlEncoder, kBlOffset);
  private SwerveModule brModule = new SwerveModule(kBrDrive, kBrTurn, kBrEncoder, kBrOffset);


  /*private double rot;

  public void setRot(double in) {
    rot -= in;
  }

  public Rotation2d getRot() {
    return Rotation2d.fromDegrees(rot);
  }

  private Pigeon2 gyro = new Pigeon2(8); */

  public SwerveSubsystem() {
    /*new Thread(() -> {
      try {
        Thread.sleep(1000);
        resetHeading();
      } catch (Exception e) {}}
    ).start();*/
  }

  /*public void resetHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getRot2d() {
    return Rotation2d.fromDegrees(getHeading());
  }*/

  public void stopModules() {
    flModule.stop();
    frModule.stop();
    blModule.stop();
    brModule.stop();
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxMeterPerSec);
    blModule.setState(states[0]);
    brModule.setState(states[1]);
    flModule.setState(states[2]);
    frModule.setState(states[3]);
  }

  @Override
  public void periodic() {
    //double rotation = rot;
    SmartDashboard.putNumber("blSet", blModule.current.angle.getRadians());
    SmartDashboard.putNumber("blCurrent", blModule.getTurnPosition());
    SmartDashboard.putNumber("blMod", degreesToRadians(blModule.pid.calculate(blModule.getTurnPosition(), blModule.current.angle.getRadians())/2)/(2*PI));
    SmartDashboard.putNumber("brSet", brModule.current.angle.getRadians());
    SmartDashboard.putNumber("brCurrent", brModule.getTurnPosition());
    SmartDashboard.putNumber("brMod", degreesToRadians(brModule.pid.calculate(brModule.getTurnPosition(), brModule.current.angle.getRadians())/2)/(2*PI));
    SmartDashboard.putNumber("flSet", flModule.current.angle.getRadians());
    SmartDashboard.putNumber("flCurrent", flModule.getTurnPosition());
    SmartDashboard.putNumber("flMod", degreesToRadians(flModule.pid.calculate(flModule.getTurnPosition(), flModule.current.angle.getRadians())/2)/(2*PI));
    SmartDashboard.putNumber("frSet", frModule.current.angle.getRadians());
    SmartDashboard.putNumber("frCurrent", frModule.getTurnPosition());
    SmartDashboard.putNumber("frMod", degreesToRadians(frModule.pid.calculate(frModule.getTurnPosition(), frModule.current.angle.getRadians())/2)/(2*PI));
    double states[] = {
      flModule.current.angle.getRadians(),
      flModule.current.speedMetersPerSecond,
      frModule.current.angle.getRadians(),
      frModule.current.speedMetersPerSecond,
      blModule.current.angle.getRadians(),
      blModule.current.speedMetersPerSecond,
      brModule.current.angle.getRadians(),
      brModule.current.speedMetersPerSecond
    };
    SmartDashboard.putNumberArray("States", states);
  }
}