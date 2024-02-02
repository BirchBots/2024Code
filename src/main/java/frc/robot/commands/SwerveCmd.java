package frc.robot.commands;

import static frc.robot.Constants.SwerveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCmd extends Command {
    private SwerveSubsystem swerveSubsystem;
    private Supplier<Double> xSpdFunction, ySpdFunction, turnSpdFunction;
    private Supplier<Boolean> fieldOrientFunction;
    private SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    public SwerveCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turnSpdFunction, Supplier<Boolean> fieldOrientFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turnSpdFunction = turnSpdFunction;
        this.fieldOrientFunction = fieldOrientFunction;
        this.xLimiter = new SlewRateLimiter(kSlewRate);
        this.yLimiter = new SlewRateLimiter(kSlewRate);
        this.turnLimiter = new SlewRateLimiter(kTurnSlewRate);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turnSpeed = turnSpdFunction.get();

        xSpeed = Math.abs(xSpeed) > kDeadZone ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > kDeadZone ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > kDeadZone ? turnSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed)*kMaxMeterPerSec;
        ySpeed = yLimiter.calculate(ySpeed)*kMaxMeterPerSec;
        turnSpeed = turnLimiter.calculate(turnSpeed)*kMaxMeterPerSec;

        ChassisSpeeds speeds;
        //if (fieldOrientFunction.get()) {
            SmartDashboard.putBoolean("Field", false);
            //swerveSubsystem.setRot(turnSpeed);
            speeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        //} else {
            /*SmartDashboard.putBoolean("Field", true);
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRot());
        }*/
        SwerveModuleState[] moduleStates = kKinematics.toSwerveModuleStates(speeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}