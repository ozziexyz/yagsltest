package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
    private final SwerveDrive swerve;
    private final PathConstraints pathConstraints;

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
        configurePathPlanner();
        pathConstraints = new PathConstraints(
            swerve.getMaximumVelocity(), 
            4.0,
            swerve.getMaximumAngularVelocity(), 
            Units.degreesToRadians(720)
        );
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

    public Command driveToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(
            pose,
            pathConstraints,
            0.0, 
            0.0
        );
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerve.driveFieldOriented(velocity);
    }

    public void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry, 
            this::getRobotVelocity, 
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                Constants.TRANSLATION_PID,
                Constants.ANGLE_PID,
                4.5,
                swerve.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                new ReplanningConfig()
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this
        );
    }
 
    @AutoLogOutput
    public Pose2d getPose() {
        return swerve.getPose();
    }

    @AutoLogOutput
    public Pose3d getPose3d() {
        return new Pose3d(swerve.getPose());
    }

    @Override
    public void periodic() {
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerve.resetOdometry(initialHolonomicPose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerve.getRobotVelocity();
    }

    public void setChassisSpeeds(ChassisSpeeds velocity) {
        swerve.setChassisSpeeds(velocity);
    }
}