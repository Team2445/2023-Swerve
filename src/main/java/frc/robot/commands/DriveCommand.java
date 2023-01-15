// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {

  private Drivetrain drivetrain;
  private DoubleSupplier x, y, rotation;

  /** Creates a new DriveCommand. */
  public DriveCommand(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
    this.drivetrain = drivetrain;
    this.x = x; 
    this.y = y;
    this.rotation = rotation;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xPercent = x.getAsDouble();
    double yPercent = y.getAsDouble();
    double rotationPercent = rotation.getAsDouble();

    this.drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      xPercent * MAX_VELOCITY_METERS_PER_SECOND, 
      yPercent * MAX_VELOCITY_METERS_PER_SECOND, 
      rotationPercent * MAX_VELOCITY_METERS_PER_SECOND,
      this.drivetrain.getRotation()
    ));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
