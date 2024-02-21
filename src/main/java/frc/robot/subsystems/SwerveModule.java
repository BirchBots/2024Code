package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import static frc.robot.Constants.ModuleConstants.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveModule {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;

  private final AnalogEncoder m_turningEncoder;
  private final AnalogInput m_turningEncoderInput;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, int encoderId) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoderInput = new AnalogInput(encoderId); 
    m_turningEncoder = new AnalogEncoder(m_turningEncoderInput);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController = m_turningSparkMax.getPIDController();

    m_drivingEncoder.setPositionConversionFactor(kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(kDrivingEncoderVelocityFactor);

    m_drivingPIDController.setPositionPIDWrappingEnabled(true);
    m_drivingPIDController.setPositionPIDWrappingMinInput(kTurningEncoderPositionPIDMinInput);
    m_drivingPIDController.setPositionPIDWrappingMaxInput(kTurningEncoderPositionPIDMaxInput);

    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(kTurningEncoderPositionPIDMaxInput);

    m_drivingPIDController.setP(kDrivingP);
		m_drivingPIDController.setI(kDrivingI);
		m_drivingPIDController.setD(kDrivingD);
		m_drivingPIDController.setFF(kDrivingFF);
		m_drivingPIDController.setOutputRange(kDrivingMinOutput, kDrivingMaxOutput);

    m_turningPIDController.setP(kTurningP);
    m_turningPIDController.setI(kTurningI);
    m_turningPIDController.setD(kTurningD);
    m_turningPIDController.setFF(kTurningFF);
    m_turningPIDController.setOutputRange(kTurningMinOutput, kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(kTurningMotorCurrentLimit);

    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.get());
    m_drivingEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(Math.toRadians(m_turningEncoder.get() * 360) - m_chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(Math.toRadians(m_turningEncoder.get() * 360) - m_chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));

    m_drivingPIDController.setReference(desiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(desiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("Speed", desiredState.speedMetersPerSecond);

    m_desiredState = desiredState;
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}