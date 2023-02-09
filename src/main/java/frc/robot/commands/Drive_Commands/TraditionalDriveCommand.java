// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.math.GeomUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TraditionalDriveCommand extends CommandBase {

  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;
  private final Supplier<Boolean> robotRelativeOverride;
  private final Supplier<JoystickMode> modeSupplier;
  private final Supplier<Double> linearSpeedLimitSupplier;
  private final Supplier<Double> angularSpeedLimitSupplier;
  private final Supplier<Double> targetNodeSupplier;

  private static final double deadband = 0.1;

  private SlewRateLimiter limiter1 = new SlewRateLimiter(15.0);
  private SlewRateLimiter limiter2 = new SlewRateLimiter(15.0);
  private SlewRateLimiter limiter3 = new SlewRateLimiter(70.0);

  /** Creates a new TraditionalDriveCommand. */
  public TraditionalDriveCommand(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightXSupplier,
      Supplier<Boolean> robotRelativeOverride,
      Supplier<JoystickMode> modeSupplier,
      Supplier<Double> linearSpeedLimitSupplier,
      Supplier<Double> angularSpeedLimitSupplier,
      Supplier<Double> targetNodeSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.robotRelativeOverride = robotRelativeOverride;
    this.modeSupplier = modeSupplier;
    this.linearSpeedLimitSupplier = linearSpeedLimitSupplier;
    this.angularSpeedLimitSupplier = angularSpeedLimitSupplier;
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
    double rightY = rightXSupplier.get();

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband);
    rightY = MathUtil.applyDeadband(rightY, deadband);

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    rightY = Math.copySign(rightY * rightY, rightY);

    // Apply speed limits
    linearMagnitude *= linearSpeedLimitSupplier.get();
    rightY *= angularSpeedLimitSupplier.get();

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(GeomUtil.transformFromTranslation(linearMagnitude, 0.0))
            .getTranslation();
    if (modeSupplier.get() == JoystickMode.Tank) {
      linearVelocity = new Translation2d(linearVelocity.getX(), 0.0);
    }

    // Convert to meters per second
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            limiter1.calculate(linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec()),
            limiter2.calculate(linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec()),
            limiter3.calculate(rightY * drive.getMaxAngularSpeedRadPerSec()));

    // Convert from field relative
    if (!robotRelativeOverride.get() && modeSupplier.get() == JoystickMode.Standard) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond,
              drive.getRotation());
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

  public static enum JoystickMode {
    Standard,
    Tank
  }
}
