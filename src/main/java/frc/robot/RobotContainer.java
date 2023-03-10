// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.OI.XBOX_CONTROLLER_PORT;

public class RobotContainer {

  private final Drivetrain drivetrain = new Drivetrain();

  private XboxController xboxController = new XboxController(XBOX_CONTROLLER_PORT);

  public RobotContainer() {
    drivetrain.register();

    // TODO: May have to negate these values.
    drivetrain.setDefaultCommand(new DriveCommand(
      drivetrain, 
      () -> xboxController.getLeftY(), 
      () -> xboxController.getLeftX(), 
      () -> xboxController.getRightX()
    ));

    configureBindings();
  }

  private void configureBindings() {
    new Trigger(xboxController::getStartButtonPressed)
        .onTrue(new InstantCommand(drivetrain::zeroModules));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
