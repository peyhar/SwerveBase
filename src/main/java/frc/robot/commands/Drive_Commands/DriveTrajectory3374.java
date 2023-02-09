// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.trajectory.CustomHolonomicDriveController3374;
import frc.robot.util.trajectory.CustomTrajectoryGenerator3374;
import frc.robot.util.trajectory.RotationSequence;
import frc.robot.util.trajectory.Waypoint;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class DriveTrajectory3374 extends CommandBase {
  private final double maxVelocityMetersPerSec;
  private final double maxAccelerationMetersPerSec2;
  private final double maxCentripetalAccelerationMetersPerSec2;

  private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

  private final LoggedTunableNumber driveKp = new LoggedTunableNumber("AutoDrive/DriveKp");
  private final LoggedTunableNumber driveKd = new LoggedTunableNumber("AutoDrive/DriveKd");

  private final LoggedTunableNumber turnKp = new LoggedTunableNumber("AutoDrive/TurnKp");
  private final LoggedTunableNumber turnKd = new LoggedTunableNumber("AutoDrive/TurnKd");

  private final CustomHolonomicDriveController3374 customHolonomicDriveController =
      new CustomHolonomicDriveController3374(xController, yController, thetaController);

  private final Drive drive;
  private final Timer timer = new Timer();

  private final CustomTrajectoryGenerator3374 customGenerator = new CustomTrajectoryGenerator3374();
  private static final Alert generatorAlert =
      new Alert("Failed to generate all trajectories, check constants.", AlertType.ERROR);

  public DriveTrajectory3374(Drive drive, List<Waypoint> waypoints) {
    this(drive, waypoints, 0.0, 0.0, List.of());
  }

  public DriveTrajectory3374(
      Drive drive, List<Waypoint> waypoints, double startVelocity, double endVelocity) {
    this(drive, waypoints, startVelocity, endVelocity, List.of());
  }

  public DriveTrajectory3374(
      Drive drive, List<Waypoint> waypoints, List<TrajectoryConstraint> constraints) {
    this(drive, waypoints, 0.0, 0.0, constraints);
  }

  public DriveTrajectory3374(
      Drive drive,
      List<Waypoint> waypoints,
      double startVelocity,
      double endVelocity,
      List<TrajectoryConstraint> constraints) {
    addRequirements(drive);
    this.drive = drive;

    boolean supportedRobot = true;
    switch (Constants.getRobot()) {
      case ROBOT_2023:
        maxVelocityMetersPerSec = Units.inchesToMeters(100.0);
        maxAccelerationMetersPerSec2 = Units.inchesToMeters(100.0);
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(10.0);

        driveKp.initDefault(22.0);
        driveKd.initDefault(0.0);

        turnKp.initDefault(22.0);
        turnKd.initDefault(0.0);
        break;
      case ROBOT_SIMBOT:
        maxVelocityMetersPerSec = Units.inchesToMeters(150.0);
        maxAccelerationMetersPerSec2 = Units.inchesToMeters(200.0);
        maxCentripetalAccelerationMetersPerSec2 = Units.inchesToMeters(120.0);

        driveKp.initDefault(22.0);
        driveKd.initDefault(0.0);

        turnKp.initDefault(22.0);
        turnKd.initDefault(0.0);
        break;
      default:
        supportedRobot = false;
        maxVelocityMetersPerSec = 0.0;
        maxAccelerationMetersPerSec2 = 0.0;
        maxCentripetalAccelerationMetersPerSec2 = 0.0;

        driveKp.initDefault(0.0);
        driveKd.initDefault(0.0);

        turnKp.initDefault(0.0);
        turnKd.initDefault(0.0);
        break;
    }

    // setup trajectory configuration
    TrajectoryConfig config =
        new TrajectoryConfig(maxVelocityMetersPerSec, maxAccelerationMetersPerSec2)
            .setKinematics(new SwerveDriveKinematics(drive.getModuleTranslations()))
            .setStartVelocity(startVelocity)
            .setEndVelocity(endVelocity)
            .addConstraint(
                new CentripetalAccelerationConstraint(maxCentripetalAccelerationMetersPerSec2))
            .addConstraints(constraints);

    // Generate trajectory
    try {
      customGenerator.generate(config, waypoints);
    } catch (TrajectoryGenerationException exception) {
      if (supportedRobot) {
        generatorAlert.set(true);
        DriverStation.reportError("Failed to generate trajectory.", true);
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    xController.reset();
    yController.reset();
    thetaController.reset();

    xController.setD(driveKd.get());
    xController.setP(driveKp.get());

    yController.setD(driveKd.get());
    yController.setP(driveKp.get());

    thetaController.setD(driveKd.get());
    thetaController.setP(driveKp.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State driveState;
    Trajectory trajectory = customGenerator.getDriveTrajectory();

    try {
      driveState = trajectory.sample(timer.get());
    } catch (Exception e) {
      DriverStation.reportError("Error generating trajectory", false);
      return;
    }

    RotationSequence.State holonomicRotationState =
        customGenerator.getHolonomicRotationSequence().sample(timer.get());

    ChassisSpeeds nextDriveState =
        customHolonomicDriveController.calculate(
            drive.getPose(), driveState, holonomicRotationState);

    drive.runVelocity(nextDriveState);

    Logger.getInstance()
        .recordOutput(
            "Odometry/ProfileSetpoint",
            new double[] {
              driveState.poseMeters.getX(),
              driveState.poseMeters.getY(),
              holonomicRotationState.position.getRadians()
            });

    Logger.getInstance().recordOutput("Trajectory", trajectory);

    if (driveKd.hasChanged()
        || driveKp.hasChanged()
        || turnKd.hasChanged()
        || turnKp.hasChanged()) {
      xController.setD(driveKd.get());
      xController.setP(driveKp.get());

      yController.setD(driveKd.get());
      yController.setP(driveKp.get());

      thetaController.setD(turnKd.get());
      thetaController.setP(turnKp.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(customGenerator.getDriveTrajectory().getTotalTimeSeconds());
  }
}
