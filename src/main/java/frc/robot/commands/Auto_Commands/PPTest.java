// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.Auto_Commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class PPTest extends SequentialCommandGroup {

  PathPlannerTrajectory examplePath = PathPlanner.loadPath("VPath", new PathConstraints(4, 3));

  /** Creates a new DriveToTarget. */
  public PPTest(Drive drive) {

    addCommands(drive.followPPTrajectoryCommand(examplePath, true));
  }
}
