// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.math.GeomUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class StickAngleDriveCommand extends CommandBase {

  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightYSupplier;
  private final Supplier<Double> rightXSupplier;
  private final Supplier<Boolean> robotRelativeOverride;
  private final Supplier<Double> linearSpeedLimitSupplier;
  private final Supplier<Double> angularSpeedLimitSupplier;
  private final Supplier<Boolean> rightBumperSupplier;
  private final Supplier<Boolean> leftBumperSupplier;
  private final Supplier<Boolean> bButtonSupplier;
  private final Supplier<Double> targetNodeSupplier;

  private static final double deadband = 0.1;

  private double targetHeading = 0.0;

  /** Creates a new StickAngleDriveCommand. */
  public StickAngleDriveCommand(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightYSupplier,
      Supplier<Double> rightXSupplier,
      Supplier<Boolean> robotRelativeOverride,
      Supplier<Double> linearSpeedLimitSupplier,
      Supplier<Double> angularSpeedLimitSupplier,
      Supplier<Boolean> rightBumperSupplier,
      Supplier<Boolean> leftBumperSupplier,
      Supplier<Boolean> bButtonSupplier,
      Supplier<Double> targetNodeSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightYSupplier = rightYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.robotRelativeOverride = robotRelativeOverride;
    this.linearSpeedLimitSupplier = linearSpeedLimitSupplier;
    this.angularSpeedLimitSupplier = angularSpeedLimitSupplier;
    this.rightBumperSupplier = rightBumperSupplier;
    this.leftBumperSupplier = leftBumperSupplier;
    this.bButtonSupplier = bButtonSupplier;
    this.targetNodeSupplier = targetNodeSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Send Automation Target To Logger
    Logger.getInstance()
        .recordOutput(
            "Target Pose", FieldConstants.getTargetPose(targetNodeSupplier.get().intValue()));

    // Get values from double suppliers
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();
    double rightX = rightXSupplier.get();
    double rightY = rightYSupplier.get();

    // Snap stick angle to x and y
    if (rightX >= -0.1 && rightX <= 0.1) {
      rightX = 0;
    }
    if (rightY >= -0.1 && rightY <= 0.1) {
      rightY = 0;
    }

    SmartDashboard.putBoolean("left bumper", leftBumperSupplier.get());

    SmartDashboard.putBoolean("right bumper", rightBumperSupplier.get());

    double rightStickAngle = Math.atan2(-rightX, rightY);

    boolean rightStickPressed = Math.max(Math.abs(rightX), Math.abs(rightY)) > 0.3;

    // vector to target
    Pose2d TargetPose = RobotContainer.autoDriveTarget;

    Transform2d relTargetPose = drive.getPose().minus(TargetPose);

    double absAngleToTarget = -Math.atan2(relTargetPose.getY(), -relTargetPose.getX());

    SmartDashboard.putNumber("Abs angle to target", absAngleToTarget);

    // Update target heading
    if (rightStickPressed) {
      targetHeading = rightStickAngle;
    } else if (rightBumperSupplier.get()) {
      targetHeading -= 0.1;
    } else if (leftBumperSupplier.get()) {
      targetHeading += 0.1;
    } else if (bButtonSupplier.get()) {
      targetHeading = absAngleToTarget;
    }

    // Unwrap
    if (targetHeading > Math.PI) {
      targetHeading -= 2 * Math.PI;
    } else if (targetHeading < -Math.PI) {
      targetHeading += 2 * Math.PI;
    }

    SmartDashboard.putNumber("Target heading", targetHeading);

    double angleDiffRad = targetHeading - (drive.getRotation().getRadians());

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband);

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);

    // Apply speed limits
    linearMagnitude *= linearSpeedLimitSupplier.get();
    angleDiffRad *= angularSpeedLimitSupplier.get();

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(GeomUtil.transformFromTranslation(linearMagnitude, 0.0))
            .getTranslation();

    double angularRate = angleDiffRad * drive.getMaxAngularSpeedRadPerSec();

    // Unwrap Angle Diff
    if (angleDiffRad > Math.PI) {
      angularRate = (angleDiffRad - 2 * Math.PI) * drive.getMaxAngularSpeedRadPerSec();
    } else if (angleDiffRad < -Math.PI) {
      angularRate = (angleDiffRad + 2 * Math.PI) * drive.getMaxAngularSpeedRadPerSec();
    } else {
      angularRate = angleDiffRad * drive.getMaxAngularSpeedRadPerSec();
    }

    // Convert to meters per second
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            angularRate);

    // Convert from field relative
    if (!robotRelativeOverride.get()) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond,
              drive.getRotation().minus(Rotation2d.fromDegrees(270)));
    }

    // Apply auto drive
    // ChassisSpeeds autoDriveSpeeds =
    // AutoDriveSoftWithSpline.calculate(drive.getPose(),
    // autoDriveSupplier.get() * drive.getMaxLinearSpeedMetersPerSec(),
    // drive.getFieldVelocity());
    // speeds = new ChassisSpeeds(
    // speeds.vxMetersPerSecond + autoDriveSpeeds.vxMetersPerSecond,
    // speeds.vyMetersPerSecond + autoDriveSpeeds.vyMetersPerSecond,
    // speeds.omegaRadiansPerSecond + autoDriveSpeeds.omegaRadiansPerSecond);

    // Send to drive
    drive.runVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
