// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.*;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static class SwerveConstants {
    public static final MotorType kBrushless = MotorType.kBrushless;

    public static final double kP = 1; //needs tuning
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kDriveGearRatio = 1/6.75; //probably??
    public static final double kSteerGearRatio = 150/7;
    public static final int kWheelDiameter = 4;
    
    public static final double kMaxMeterPerSec = feetToMeters(15.1); //idk how accurate this is

    public static final int kFlDrive = 2;
    public static final int kFlSteer = 1;
    public static final int kFrDrive = 4;
    public static final int kFrSteer = 3;
    public static final int kBlDrive = 8;
    public static final int kBlSteer = 7;
    public static final int kBrDrive = 5;
    public static final int kBrSteer = 6;

    public static final int kFlEncoder = 2;
    public static final int kFrEncoder = 1;
    public static final int kBlEncoder = 3;
    public static final int kBrEncoder = 0;

    public static final double kFlOffset = 0; //all need to be tested (rad)
    public static final double kFrOffset = 0;
    public static final double kBlOffset = 0;
    public static final double kBrOffset = 0;

    public static final double kChassisWidth = inchesToMeters(30.25); //idk either of these
    public static final double kChassisLength = inchesToMeters(30.25);

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
      new Translation2d(kChassisLength/2, -kChassisWidth/2),
      new Translation2d(kChassisLength/2, kChassisWidth/2),
      new Translation2d(-kChassisLength/2, -kChassisWidth/2),
      new Translation2d(-kChassisLength/2, kChassisWidth/2)
    );
    public static final double kSlewRate = 2;

    public static final double kDeadZone = 0.05;

    public static final int kXAxis = 1;
    public static final int kYAxis = 2;
    public static final int kRelativeButton = 1;
  }
}
