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
    public static final double kI = 0.1;
    public static final double kD = 1;

    public static final double kDriveGearRatio = 6.75;
    public static final double kTurnGearRatio = 150/7;
    public static final int kWheelDiameter = 4;
    
    public static final double kMaxMeterPerSec = feetToMeters(5) ; // 15.1 L2 drivetrain free speed
    public static final double kRadToMotor = 1.570796;

    public static final int kFlDrive = 2;
    public static final int kFlTurn = 1;
    public static final int kFrDrive = 4;
    public static final int kFrTurn = 3;
    public static final int kBrDrive = 6;
    public static final int kBrTurn = 5;
    public static final int kBlDrive = 8;
    public static final int kBlTurn = 7;

    public static final int kFlEncoder = 0;
    public static final int kFrEncoder = 1;
    public static final int kBlEncoder = 3;
    public static final int kBrEncoder = 2;

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
    public static final double kSlewRate = 1;
    public static final double kTurnSlewRate = 2;
    public static final double kDeadZone = 0.1;
  }
}
