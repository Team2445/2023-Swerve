// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    // Creates Swerve Modules (Drive Motor, Steer Motor, and Encoder)
    private SwerveModule frontLeftModule, frontRightModule, rearLeftModule, rearRightModule;

    // TODO: Need to find gyro
    private Gyro gyro;

    // TODO: Measure Trackwidth and Wheelbase for Kinematics
    private final SwerveDriveKinematics kinematics;

    private final SwerveDriveOdometry odometry;

    private ChassisSpeeds chassisSpeeds;
  
    /** Creates a new Drivetrain. */
    public Drivetrain() {
      ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drive");

      kinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
      );
      
      // TODO: Determine Offsets
      frontLeftModule = new MkSwerveModuleBuilder()
        .withLayout(shuffleboardTab.getLayout("front left module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(0, 0))
        .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
        .withDriveMotor(MotorType.NEO, FRONT_LEFT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, FRONT_LEFT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_MOTOR)
        .withSteerOffset(FRONT_LEFT_MODULE_OFFSET)
        .build();

      frontRightModule = new MkSwerveModuleBuilder()
        .withLayout(shuffleboardTab.getLayout("front right module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(2, 0))
        .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
        .withDriveMotor(MotorType.NEO, FRONT_RIGHT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, FRONT_RIGHT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_MOTOR)
        .withSteerOffset(FRONT_RIGHT_MODULE_OFFSET)
        .build();

      rearLeftModule = new MkSwerveModuleBuilder()
        .withLayout(shuffleboardTab.getLayout("rear left module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(4, 0))
        .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
        .withDriveMotor(MotorType.NEO, REAR_LEFT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, REAR_LEFT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(REAR_LEFT_MODULE_STEER_MOTOR)
        .withSteerOffset(REAR_LEFT_MODULE_OFFSET)
        .build();
      
      rearRightModule = new MkSwerveModuleBuilder()
        .withLayout(shuffleboardTab.getLayout("rear right module", BuiltInLayouts.kList)
          .withSize(2, 4)
          .withPosition(6, 0))
        .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
        .withDriveMotor(MotorType.NEO, REAR_RIGHT_MODULE_DRIVE_MOTOR)
        .withSteerMotor(MotorType.NEO, REAR_RIGHT_MODULE_STEER_MOTOR)
        .withSteerEncoderPort(REAR_RIGHT_MODULE_STEER_MOTOR)
        .withSteerOffset(REAR_RIGHT_MODULE_OFFSET)
        .build();

      odometry = new SwerveDriveOdometry(
        kinematics, 
        gyro.getRotation2d(), 
        new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
        }
      );

      chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

      shuffleboardTab.addNumber("Gyro Angle", () -> odometry.getPoseMeters().getRotation().getDegrees());
      shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
      shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
    }

    public void zeroModules() {
      odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeftModule.getPosition(), 
          frontRightModule.getPosition(), 
          rearLeftModule.getPosition(), 
          rearRightModule.getPosition() 
        },
        new Pose2d(
          odometry.getPoseMeters().getTranslation(), 
          Rotation2d.fromDegrees(0.0)
        )
      );
    }

    public Rotation2d getRotation() {
      return odometry.getPoseMeters().getRotation();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
      this.chassisSpeeds = chassisSpeeds;
    }
 
    @Override
    public void periodic() {
      // This method will be called once per scheduler run

      odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
        }
      );

      SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

      frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
      frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
      rearLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
      rearRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }
}
