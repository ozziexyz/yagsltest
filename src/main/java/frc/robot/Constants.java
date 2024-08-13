package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final String CONFIG_DIR = "swerve";
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    public static final double STICK_DEADBAND = 0.1;
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    public static final String AUTO_NAME = "test";
}
