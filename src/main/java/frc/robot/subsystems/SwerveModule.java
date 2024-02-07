package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.*;
import static frc.robot.Constants.SwerveConstants.*;
import static java.lang.Math.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder; // not in REV code

class SwerveModule {
    CANSparkMax driveMotor;
    private CANSparkMax turnMotor;
    private RelativeEncoder driveEncoder, turnEncoder;
    private AnalogEncoder absoluteEncoder;
    private SparkPIDController pid;
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

      pid = turnMotor.getPIDController();
      pid.setFeedbackDevice(turnEncoder);
      pid.setPositionPIDWrappingEnabled(true);
      pid.setPositionPIDWrappingMinInput(kPidMin);
      pid.setPositionPIDWrappingMaxInput(kPidMax);
      pid.setP(kP);
      pid.setI(kI);
      pid.setD(kD);
      pid.setFF(kFF);
      pid.setOutputRange(kPidMin, kPidMax);

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
      } else {
        current = SwerveModuleState.optimize(state, new Rotation2d(getTurnPosition()));
        driveMotor.set(current.speedMetersPerSecond/kMaxMeterPerSec);
        //set turnMotor here
      }
    }
    
    public void stop() {
      driveMotor.set(0);
      turnMotor.set(0);
    }
  }
