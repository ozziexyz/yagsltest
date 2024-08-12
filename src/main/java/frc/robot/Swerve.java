package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
    private final SwerveDrive swerve;

    public Swerve() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        File configDir = new File(Filesystem.getDeployDirectory(), Constants.CONFIG_DIR);
        try {
            swerve = new SwerveParser(configDir).createSwerveDrive(Constants.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerve.setHeadingCorrection(false);
        swerve.setCosineCompensator(false);
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        return run(() -> {
            double xSpeed =  MathUtil.applyDeadband(translationX.getAsDouble(), Constants.STICK_DEADBAND) * swerve.getMaximumVelocity();
            double ySpeed = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.STICK_DEADBAND) * swerve.getMaximumVelocity();
            double omega = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.STICK_DEADBAND) * swerve.getMaximumAngularVelocity();
            ChassisSpeeds velocity = swerve.swerveController.getRawTargetSpeeds(xSpeed, ySpeed, omega);
            driveFieldOriented(velocity);
        });
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerve.driveFieldOriented(velocity);
    }
 
    @AutoLogOutput
    public Pose2d getPose() {
        return swerve.getPose();
    }

    @Override
    public void periodic() {
    }
}