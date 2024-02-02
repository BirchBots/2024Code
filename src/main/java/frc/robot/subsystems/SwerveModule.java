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
    CANSparkMax driveMotor;
    private CANSparkMax turnMotor;
    private RelativeEncoder driveEncoder, turnEncoder;
    private AnalogEncoder absoluteEncoder;
    PIDController pid;
    private double offsetRad;
    SwerveModuleState current = new SwerveModuleState();
    double pidOut;
  
    SwerveModule(int driveId, int turnId, int encoderId, double offsetRad) {
      driveMotor = new CANSparkMax(driveId, kBrushless);
      driveEncoder = driveMotor.getEncoder();
      driveEncoder.setPositionConversionFactor(kDriveGearRatio*PI*inchesToMeters(kWheelDiameter));
      driveEncoder.setVelocityConversionFactor(driveEncoder.getPositionConversionFactor()/60);

      turnMotor = new CANSparkMax(turnId, kBrushless);
      turnEncoder = turnMotor.getEncoder();
      turnEncoder.setPositionConversionFactor(kTurnGearRatio*2*PI);
      turnEncoder.setVelocityConversionFactor(turnEncoder.getPositionConversionFactor()/60);

      absoluteEncoder = new AnalogEncoder(encoderId);
      absoluteEncoder.setDistancePerRotation(1);

      pid = new PIDController(kP, kI, kD);

      pidOut = 0;
      this.offsetRad = offsetRad;

      resetEncoders();
    }
  
    public double getTurnPosition() {
      return turnEncoder.getPosition() % (2 * PI);
    }
  
    public double getDriveVelocity() {
      return driveEncoder.getVelocity();
    }
  
    public double getAbsoluteEncoderRad() {
      return 2*PI*absoluteEncoder.getDistance() - this.offsetRad;
    }
  
    public void resetEncoders() {
      driveEncoder.setPosition(0);
      turnEncoder.setPosition(0);
    }
  
    public SwerveModuleState getState() {
      return current;
    }
  
    public void setState(SwerveModuleState state) {
      if (abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
      }
      current = state;
      SwerveModuleState.optimize(state, current.angle);
      driveMotor.set(current.speedMetersPerSecond/kMaxMeterPerSec);
      pidOut = degreesToRadians(pid.calculate(getTurnPosition(), current.angle.getRadians()/2))/(2*PI);
      turnMotor.set(pidOut > 2*PI ? -pidOut - PI : pidOut);
    }
    
    public void stop() {
      driveMotor.set(0);
      turnMotor.set(0);
    }
  }