// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.SplineHelper;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.math.GeomUtil;
import java.util.List;

public class AutoDriveSoftWithSpline {
  private static final Pose2d targetPose = RobotContainer.autoDriveTarget;
  private static final Rotation2d targetHolonomicRotation = new Rotation2d();

  private AutoDriveSoftWithSpline() {}

  public static ChassisSpeeds calculate(
      Pose2d currentPose, double requestedVelocity, Translation2d currentVelocity) {
    Translation2d splineVelocity;
    double angularVelocity = 0.0;
    if (currentPose.relativeTo(targetPose).getX() < 0.0) {
      // Calculate spline velocity
      Pose2d startPose =
          new Pose2d(
              currentPose.getTranslation(),
              GeomUtil.direction(targetPose.getTranslation().minus(currentPose.getTranslation())));
      Translation2d nextSplinePoint =
          getNextPoint(startPose, targetPose, requestedVelocity * Constants.loopPeriodSecs);
      splineVelocity =
          new Translation2d(requestedVelocity, 0.0)
              .rotateBy(GeomUtil.direction(nextSplinePoint.minus(currentPose.getTranslation())));

      // Calculate angular velocity
      double t =
          (requestedVelocity * Constants.loopPeriodSecs)
              / currentPose.getTranslation().getDistance(targetPose.getTranslation());
      angularVelocity = targetHolonomicRotation.minus(currentPose.getRotation()).getRadians() * t;

    } else {
      // Go straight forward
      splineVelocity = new Translation2d(requestedVelocity, 0.0).rotateBy(targetPose.getRotation());
    }

    // Convert to robot relative speeds
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        splineVelocity.getX(), splineVelocity.getY(), angularVelocity, currentPose.getRotation());
  }

  /** Generates a spline and moves forward by the distance specified. */
  private static Translation2d getNextPoint(
      Pose2d startPoseMeters, Pose2d endPoseMeters, double targetDistanceMeters) {

    // Generate spline
    Spline spline =
        SplineHelper.getQuinticSplinesFromWaypoints(List.of(startPoseMeters, endPoseMeters))[0];

    // Find next point
    double distance = 0.0;
    Translation2d lastPoint = startPoseMeters.getTranslation();
    for (double t = 0.0; t < 1.0; t += 0.001) {
      Translation2d newPoint = spline.getPoint(t).poseMeters.getTranslation();
      distance += lastPoint.getDistance(newPoint);
      lastPoint = newPoint;
      if (distance > targetDistanceMeters) {
        return newPoint;
      }
    }
    return endPoseMeters.getTranslation();
  }
}
