package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.*;
import static frc.robot.Constants.SwerveConstants.*;
import static java.lang.Math.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;

class SwerveModule {
    private CANSparkMax driveMotor, turnMotor;
    private RelativeEncoder driveEncoder, turnEncoder;
    private AnalogEncoder absoluteEncoder;
    private PIDController pid;
    private double offsetRad;
    private boolean absoluteReversed;
    SwerveModuleState current = new SwerveModuleState();
  
    SwerveModule(int driveId, int turnId, int encoderId, boolean driveReversed, boolean turnReversed, boolean absoluteReversed, double offsetRad) {
      driveMotor = new CANSparkMax(driveId, kBrushless);
      driveEncoder = driveMotor.getEncoder();
      driveMotor.setInverted(driveReversed);
      driveEncoder.setPositionConversionFactor(kDriveGearRatio*PI*inchesToMeters(kWheelDiameter));
      driveEncoder.setVelocityConversionFactor(driveEncoder.getPositionConversionFactor()/60);
      turnMotor = new CANSparkMax(turnId, kBrushless);
      turnMotor.setInverted(turnReversed);
      turnEncoder = turnMotor.getEncoder();
      turnEncoder.setPositionConversionFactor(kTurnGearRatio*2*PI);
      turnEncoder.setVelocityConversionFactor(turnEncoder.getPositionConversionFactor()/60);
      absoluteEncoder = new AnalogEncoder(encoderId);
      absoluteEncoder.setDistancePerRotation(1);
      pid = new PIDController(kP, kI, kD);
      pid.enableContinuousInput(-PI, PI);
      resetEncoders();
      this.absoluteReversed = absoluteReversed;
      this.offsetRad = offsetRad;
    }
  
    public CANSparkMax getDriveMotor() {
      return driveMotor;
    }
  
    public CANSparkMax getTurnMotor() {
      return turnMotor;
    }
  
    public double getTurnPosition() {
      return turnEncoder.getPosition();
    }
  
    public double getDriveVelocity() {
      return driveEncoder.getVelocity();
    }
  
    public double getTurnVelocity() {
      return turnEncoder.getVelocity();
    }
  
    public double getAbsoluteEncoderRad() {
      double angle = 2*PI*absoluteEncoder.getDistance() - this.offsetRad;
      return angle * (absoluteReversed ? -1 : 1);
    }
  
    public void resetEncoders() {
      driveEncoder.setPosition(0);
      turnEncoder.setPosition(absoluteEncoder.getDistance());
    }
  
    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }
  
    public void setState(SwerveModuleState state) {
      if (abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
      }
      current = SwerveModuleState.optimize(state, getState().angle);
      driveMotor.set(current.speedMetersPerSecond/kMaxMeterPerSec);
      //turnMotor.set(pid.calculate(getTurnPosition(), current.angle.getRadians()));
      turnMotor.set(current.angle.getRadians());
    }
    
    public void stop() {
      driveMotor.set(0);
      turnMotor.set(0);
    }
  }