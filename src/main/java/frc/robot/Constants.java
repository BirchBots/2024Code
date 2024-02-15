// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.*;
import static java.lang.Math.*;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static class SwerveConstants {
    
    public static final MotorType kBrushless = MotorType.kBrushless;
    public static final boolean kGyroReversed = false; // added from REV code
    public static final double kDeadZone = 0.1; // updated to match REV (kDriveDeadband)

    public static final double kMotorPinionTeeth = 14;
    public static final double kMotorReduction = (45 * 20) / (kMotorPinionTeeth * 15);

    public static final double kWheelDiameter = inchesToMeters(4);
    public static final double kWheelCircumference = kWheelDiameter*PI;
    public static final double kNeoRpm = 5676;
    public static final double kWheelRps = ((kNeoRpm/60)*kWheelCircumference)/kMotorReduction;

    public static final double kInPidMin = -PI;
    public static final double kInPidMax = PI;
    public static final double kOutPidMin = -1;
    public static final double kOutPidMax = 1;
    public static final double kP = 2; //needs tuning
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 1/kWheelRps;

    public static final double kDriveGearRatio = 6.75;
    public static final double kTurnGearRatio = 150/7;
    
    public static final double kMaxMeterPerSec = feetToMeters(0.01) ; // 15.1 L2 drivetrain free speed

    public static final double kMotorPinionTeeth = 14;
    public static final double kMotorReduction = (45 * 20) / (kMotorPinionTeeth * 15);

    public static final double kWheelDiameter = inchesToMeters(4);
    public static final double kWheelCircumference = kWheelDiameter*PI;
    public static final double kNeoRpm = 5676;
    public static final double kWheelRps = ((kNeoRpm/60)*kWheelCircumference)/kMotorReduction;

    public static final double kPidMin = -1;
    public static final double kPidMax = 1;
    public static final double kP = 2; //needs tuning
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 1/kWheelRps;

    public static final double kDriveGearRatio = 6.75;
    public static final double kTurnGearRatio = 150/7;
    
    public static final double kMaxMeterPerSec = feetToMeters(5) ; // 15.1 L2 drivetrain free speed
    public static final double kRadToMotor = 1.570796;

    // Driving Parameters
    public static final double kFreeSpeedRpm = 5676; // NEO motor free speed
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kSlewRate = 1.8; // updated to match REV
    public static final double kTurnSlewRate = 2.0; // updated to match REV

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
