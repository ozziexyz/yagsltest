// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private Swerve swerve;
  private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    swerve = new Swerve();
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
      swerve.driveCommand(
        () -> controller.getLeftY(), 
        () -> controller.getLeftX(), 
        () -> controller.getRightY()
      )
    );
  }

  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand();
  }
}
