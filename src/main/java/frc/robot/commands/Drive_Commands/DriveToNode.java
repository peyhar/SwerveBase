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
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SuppliedCommand;
import frc.robot.util.trajectory.Waypoint;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToNode extends SequentialCommandGroup {

  private final Supplier<Double> nodeSupplier;

  /** Creates a new DriveToTarget. */
  public DriveToNode(Drive drive, Supplier<Double> nodeSupplier) {
    this.nodeSupplier = nodeSupplier;

    addCommands(
        new SuppliedCommand(
            () -> {
              Pose2d holonomicPose = drive.getPose();
              Translation2d fieldVelocity = drive.getFieldVelocity();
              boolean fieldVelocityIsZero =
                  Math.abs(fieldVelocity.getX()) < 1e-3 && Math.abs(fieldVelocity.getY()) < 1e-3;

              Waypoint start = Waypoint.fromHolonomicPose(holonomicPose);
              // fieldVelocityIsZero
              //     ? Waypoint.fromHolonomicPose(holonomicPose)
              //     : Waypoint.fromHolonomicPose(
              //         holonomicPose, GeomUtil.direction(fieldVelocity));
              Waypoint end =
                  Waypoint.fromHolonomicPose(
                      FieldConstants.getTargetPose(nodeSupplier.get().intValue()));

              // Send Automation Target To Logger
              Logger.getInstance()
                  .recordOutput(
                      "Target Pose", FieldConstants.getTargetPose(nodeSupplier.get().intValue()));

              return new DriveTrajectory(drive, List.of(start, end));
            },
            drive));
  }
}
