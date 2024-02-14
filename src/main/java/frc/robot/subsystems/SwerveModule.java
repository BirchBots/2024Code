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
      //driveMotor.restoreFactoryDefaults(); // included in REV code
      driveEncoder.setPositionConversionFactor((PI*kWheelDiameter)) / kMotorReduction); // meters
      driveEncoder.setVelocityConversionFactor(driveEncoder.getPositionConversionFactor()/60); // radians per second

      turnMotor = new CANSparkMax(turnId, kBrushless);
      turnEncoder = turnMotor.getEncoder(); // REV uses turnEncoder.getAbsoluteEncoder(Type.kDutyCycle)
      //turnMotor.restoreFactoryDefaults(); // included in REV code
      turnEncoder.setPositionConversionFactor(2*PI); // radians
      turnEncoder.setVelocityConversionFactor(turnEncoder.getPositionConversionFactor()/60); // radians per second

      // what is this section for?
      absoluteEncoder = new AnalogEncoder(encoderId);
      absoluteEncoder.setDistancePerRotation(1);

      // Why do we only get turnMotorPID? REV has driveMotorPID too
      pid = turnMotor.getPIDController();
      pid.setFeedbackDevice(turnEncoder);
      pid.setPositionPIDWrappingEnabled(true); 
      pid.setPositionPIDWrappingMinInput(kPidMin); 
      pid.setPositionPIDWrappingMaxInput(kPidMax); 
      pid.setP(kP); // REV has 0.4 for drivePID & 1 for turnPID
      pid.setI(kI); // likely 0 for both drivePID & turnPID
      pid.setD(kD); // likely 0 for both drivePID & turnPID
      pid.setFF(kFF); // kFF is different for drivePID & turnPID
      pid.setOutputRange(kPidMin, kPidMax);

      // below adapted from REV code
      driveMotor.setSmartCurrentLimit(50); // amps
      turnMotor.setSmartCurrentLimit(20); // amps

      pidOut = 0; // remove??
      this.offsetRad = offsetRad; 
      //this.desiredState.angle = new Rotation2d(turnEncoder.getPosition()); // included in REV code
      resetEncoders(); // REV only has driveEncoder.setPosition(0);
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
