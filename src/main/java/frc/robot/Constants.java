// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.PIDGains;

public final class Constants {
  public static class SwerveConstants {
    
    public static final MotorType kBrushless = MotorType.kBrushless;

    // MK4i Module Constants
    public static final double kDriveGearRatio = 6.75;
    public static final double kTurnGearRatio = 150/7;
    public static final int kWheelDiameter = 4; // inches

    // TODO: Check this section against REV code
    public static final double kFreeSpeedRpm = 5676; // NEO motor free speed
    public static final double kMaxMeterPerSec = feetToMeters(5) ; // 15.1 L2 drivetrain free speed
    public static final double kRadToMotor = 1.570796; // what is this?
    public static final double kSlewRate = 1.8; // updated to match REV
    public static final double kTurnSlewRate = 2.0; // updated to match REV
    public static final double kDeadZone = 0.05; // updated to match REV (kDriveDeadband)

    // PID Constants (NOTE: REV code has different #s for driving & turning)
    public static final double kP = 0.04; // updated to match REV
    public static final double kI = 0; // updated to match REV
    public static final double kD = 0; // updated to match REV

    // SparkMax CAN IDs
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

    // Angular Offsets (values updated to match REV)
    public static final double kFlOffset = -Math.PI / 2;
    public static final double kFrOffset = 0;
    public static final double kBlOffset = Math.PI;
    public static final double kBrOffset = Math.PI / 2;

    // Chassis Configuration
    // Distance btwn centers of R & L wheels (kTrackWidth in REV)
    public static final double kChassisWidth = inchesToMeters(21.5); // updated to match REV
    // Distance between F & B wheels (kWheelBase in REV)
    public static final double kChassisLength = inchesToMeters(21.5); // updated to match REV
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
      new Translation2d(kChassisLength/2, -kChassisWidth/2),
      new Translation2d(kChassisLength/2, kChassisWidth/2),
      new Translation2d(-kChassisLength/2, -kChassisWidth/2),
      new Translation2d(-kChassisLength/2, kChassisWidth/2)
    );
    
  }
}
