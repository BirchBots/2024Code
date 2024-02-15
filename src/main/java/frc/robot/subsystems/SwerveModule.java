package frc.robot.subsystems;

import static frc.robot.Constants.SwerveConstants.*;
import static java.lang.Math.*;


import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SwerveModule {
    CANSparkMax driveMotor, turnMotor;
    private RelativeEncoder driveEncoder;
    private AbsoluteEncoder turnEncoder;
    private SparkPIDController pid;
    private double offsetRad;
    SwerveModuleState desired = new SwerveModuleState(0, new Rotation2d());
  
    SwerveModule(int driveId, int turnId, double offsetRad) {
      driveMotor = new CANSparkMax(driveId, kBrushless);
      driveMotor.burnFlash();
      driveEncoder = driveMotor.getEncoder();

      driveEncoder.setPositionConversionFactor(PI*kWheelDiameter/kMotorReduction);
      driveEncoder.setVelocityConversionFactor(driveEncoder.getPositionConversionFactor()/60);

      turnMotor = new CANSparkMax(turnId, kBrushless);
      turnMotor.burnFlash();
      turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
      turnEncoder.setPositionConversionFactor(2*PI);
      turnEncoder.setVelocityConversionFactor(turnEncoder.getPositionConversionFactor()/60);

      pid = turnMotor.getPIDController();
      pid.setPositionPIDWrappingEnabled(true);
      pid.setPositionPIDWrappingMinInput(kInPidMin);
      pid.setPositionPIDWrappingMaxInput(kInPidMax);
      pid.setFeedbackDevice(turnEncoder);
      pid.setP(kP);
      pid.setI(kI);
      pid.setD(kD);
      pid.setFF(kFF);
      pid.setOutputRange(kOutPidMin, kOutPidMax);

      this.offsetRad = offsetRad;
      this.desired.angle = new Rotation2d(turnEncoder.getPosition());
      resetEncoders();
    }
  
    public double getTurnPosition() {
      return turnEncoder.getPosition() % (2*PI);
    }
  
    public double getDriveVelocity() {
      return driveEncoder.getVelocity();
    }
  
    public void resetEncoders() {
      driveEncoder.setPosition(0);
    }
  
    public SwerveModuleState getState() {
      return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition() - offsetRad));
    }
  
    public void setDesiredState(SwerveModuleState desiredState) {
      if (abs(desiredState.speedMetersPerSecond) < 0.001) {
        stop();
      } else {
        desiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(offsetRad));
        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnPosition()));
        //driveMotor.set(current.speedMetersPerSecond/kMaxMeterPerSec);
        pid.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
        this.desired = desiredState;
      }
    }
    
    public void stop() {
      driveMotor.set(0);
      turnMotor.set(0);
    }
  }
