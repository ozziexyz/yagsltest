package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoManager {
    private Swerve swerve;

    public AutoManager(Swerve swerve) {
        this.swerve = swerve;
    }

    public Command getAutoFromFile(String name) {
        swerve.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(name));
        return AutoBuilder.buildAuto(name);
    }
}
