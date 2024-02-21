// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.lang.Math.*;
import static edu.wpi.first.math.util.Units.*;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kTriggerButtonThreshold = 0.5;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAngularSpeed = 2 * PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = inchesToMeters(21.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFlDriveCAN = 5; 
    public static final int kBlDriveCAN = 1; 
    public static final int kFrDriveCAN = 3;
    public static final int kBrDriveCAN = 7;

    public static final int kFlTurnCAN = 6;
    public static final int kBlTurnCAN = 2;
    public static final int kFrTurnCAN = 4;
    public static final int kBrTurnCAN = 8;

    public static final int kFlEncoderId = 0;
    public static final int kFrEncoderId = 1;
    public static final int kBlEncoderId = 3;
    public static final int kBrEncoderId = 2;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {

    // MK4i (not sure where this fits in the REV code)
    public static final double kDriveGearRatio = 6.75;
    public static final double kTurnGearRatio = 150/7;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * PI;
    
    // 45 teeth on the wheel's bevel gear, 20 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction = (45*17*50)/(kDrivingMotorPinionTeeth*15*27);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * PI)/kTurnGearRatio; // radians
    public static final double kTurningEncoderVelocityFactor = (2 * PI)/60.0/kTurnGearRatio; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = 2*PI; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1; //tune
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}