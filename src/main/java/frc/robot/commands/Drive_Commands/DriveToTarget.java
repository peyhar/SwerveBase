// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SuppliedCommand;
import frc.robot.util.math.GeomUtil;
import frc.robot.util.trajectory.Waypoint;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class DriveToTarget extends SequentialCommandGroup {

  /** Creates a new DriveToTarget. */
  public DriveToTarget(Drive drive) {
    addCommands(
        new SuppliedCommand(
            () -> {
              Pose2d holonomicPose = drive.getPose();
              Translation2d fieldVelocity = drive.getFieldVelocity();
              boolean fieldVelocityIsZero =
                  Math.abs(fieldVelocity.getX()) < 1e-3 && Math.abs(fieldVelocity.getY()) < 1e-3;

              Waypoint start =
                  fieldVelocityIsZero
                      ? Waypoint.fromHolonomicPose(holonomicPose)
                      : Waypoint.fromHolonomicPose(
                          holonomicPose, GeomUtil.direction(fieldVelocity));
              Waypoint end = Waypoint.fromHolonomicPose(RobotContainer.autoDriveTarget);

              // Send Automation Target To Logger
              Logger.getInstance().recordOutput("Target Pose", RobotContainer.autoDriveTarget);

              // Waypoint next = Waypoint.fromDifferentialPose(new Pose2d(
              // start.getTranslation().getX(), start.getTranslation().getY() + 1.0,
              // holonomicPose.getRotation()));

              // Waypoint next2 = Waypoint
              // .fromDifferentialPose(new Pose2d(next.getTranslation().getX() + 1.0,
              // next.getTranslation().getY() + 1.0, holonomicPose.getRotation()));

              return new DriveTrajectory(drive, List.of(start, end));
            },
            drive));
  }
}
