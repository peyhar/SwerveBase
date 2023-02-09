// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseEstimator;
import frc.robot.util.math.GeomUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private static Drive my_instance = null;

  public static synchronized Drive getInstance() {
    if (my_instance == null) {
      if (Constants.getMode() != Mode.REPLAY) {
        switch (Constants.getRobot()) {
          case ROBOT_2023:
            my_instance =
                new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(0),
                    new ModuleIOTalonFX(1),
                    new ModuleIOTalonFX(2),
                    new ModuleIOTalonFX(3));
            break;
          case ROBOT_2023P:
            my_instance =
                new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFX(0),
                    new ModuleIOTalonFX(1),
                    new ModuleIOTalonFX(2),
                    new ModuleIOTalonFX(3));
            break;
          case ROBOT_SIMBOT:
            my_instance =
                new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim(),
                    new ModuleIOSim());
            break;
          default:
            break;
        }
      }

      my_instance =
          my_instance != null
              ? my_instance
              : new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
    }
    return my_instance;
  }

  private static final double maxCoastVelocityMetersPerSec = 0.05; // Need to be under this to
  // switch to coast when disabling

  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private final ModuleIO[] moduleIOs = new ModuleIO[4]; // FL, FR, BL, BR
  private final ModuleIOInputs[] moduleInputs =
      new ModuleIOInputs[] {
        new ModuleIOInputs(), new ModuleIOInputs(), new ModuleIOInputs(), new ModuleIOInputs()
      };

  private final double maxLinearSpeed;
  private final double maxAngularSpeed;
  private final double wheelRadius;
  private final double trackWidthX;
  private final double trackWidthY;

  private final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/DriveKp");
  private final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/DriveKd");
  private final LoggedTunableNumber driveKs = new LoggedTunableNumber("Drive/DriveKs");
  private final LoggedTunableNumber driveKv = new LoggedTunableNumber("Drive/DriveKv");

  private final LoggedTunableNumber turnKp = new LoggedTunableNumber("Drive/TurnKp");
  private final LoggedTunableNumber turnKd = new LoggedTunableNumber("Drive/TurnKd");

  private final SwerveDriveKinematics kinematics;
  private SimpleMotorFeedforward driveFeedforward;
  private final PIDController[] driveFeedback = new PIDController[4];
  private final PIDController[] turnFeedback = new PIDController[4];

  private Pose2d odometryPose = new Pose2d();
  private Translation2d fieldVelocity = new Translation2d();
  private double[] lastModulePositionsRad = new double[] {0.0, 0.0, 0.0, 0.0};
  private double lastGyroPosRad = 0.0;
  private boolean brakeMode = false;

  private DriveMode driveMode = DriveMode.NORMAL;
  private ChassisSpeeds closedLoopSetpoint = new ChassisSpeeds();
  private double characterizationVoltage = 0.0;
  private SwerveModuleState[] ModuleStates;

  private PoseEstimator poseEstimator = PoseEstimator.getInstance();

  private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
  private Rotation2d lastGyroYaw = new Rotation2d();

  /** Creates a new Drive. */
  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    moduleIOs[0] = flModuleIO;
    moduleIOs[1] = frModuleIO;
    moduleIOs[2] = blModuleIO;
    moduleIOs[3] = brModuleIO;

    switch (Constants.getRobot()) {
      case ROBOT_2023:
        maxLinearSpeed = Units.feetToMeters(15);
        wheelRadius = Units.inchesToMeters(2.0);
        trackWidthX = Units.inchesToMeters(20.75);
        trackWidthY = Units.inchesToMeters(20.75);

        driveKp.initDefault(0.0);
        driveKd.initDefault(0.0);

        driveKs.initDefault(0.73899);
        driveKv.initDefault(0.24);

        turnKp.initDefault(4.5);
        turnKd.initDefault(0.0);
        break;
      case ROBOT_SIMBOT:
        maxLinearSpeed = Units.feetToMeters(15);
        wheelRadius = Units.inchesToMeters(4.0);
        trackWidthX = Units.inchesToMeters(20.75);
        trackWidthY = Units.inchesToMeters(20.75);

        driveKp.initDefault(0.9);
        driveKd.initDefault(0.0);
        driveKs.initDefault(0.116970);
        driveKv.initDefault(0.133240);

        turnKp.initDefault(23.0);
        turnKd.initDefault(0.0);
        break;
      default:
        maxLinearSpeed = 0.0;
        wheelRadius = 0.0;
        trackWidthX = 0.0;
        trackWidthY = 0.0;

        driveKp.initDefault(0.0);
        driveKd.initDefault(0.0);
        driveKs.initDefault(0.0);
        driveKv.initDefault(0.0);

        turnKp.initDefault(0.0);
        turnKd.initDefault(0.0);
        break;
    }

    kinematics = new SwerveDriveKinematics(getModuleTranslations());
    driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
    for (int i = 0; i < 4; i++) {
      driveFeedback[i] =
          new PIDController(driveKp.get(), 0.0, driveKd.get(), Constants.loopPeriodSecs);
      turnFeedback[i] =
          new PIDController(turnKp.get(), 0.0, turnKd.get(), Constants.loopPeriodSecs);
      turnFeedback[i].enableContinuousInput(-Math.PI, Math.PI);
    }

    // Calculate max angular speed
    maxAngularSpeed =
        maxLinearSpeed
            / Arrays.stream(getModuleTranslations())
                .map(translation -> translation.getNorm())
                .max(Double::compare)
                .get();
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
    for (int i = 0; i < 4; i++) {
      moduleIOs[i].updateInputs(moduleInputs[i]);
      Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i), moduleInputs[i]);
    }

    // Update objects based on TunableNumbers
    Logger.getInstance().recordOutput("TunableNumberTest", turnKp.get());
    if (driveKp.hasChanged()
        || driveKd.hasChanged()
        || driveKs.hasChanged()
        || driveKv.hasChanged()
        || turnKp.hasChanged()
        || turnKd.hasChanged()) {
      driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
      for (int i = 0; i < 4; i++) {
        driveFeedback[i].setP(driveKp.get());
        driveFeedback[i].setD(driveKd.get());
        turnFeedback[i].setP(turnKp.get());
        turnFeedback[i].setD(turnKd.get());
      }
    }

    // Update angle measurements
    Rotation2d[] turnPositions = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      turnPositions[i] = new Rotation2d(moduleInputs[i].turnAbsolutePositionRad);
    }

    if (DriverStation.isDisabled()) {
      // Disable output while disabled
      for (int i = 0; i < 4; i++) {
        moduleIOs[i].setTurnVoltage(0.0);
        moduleIOs[i].setDriveVoltage(0.0);
      }
    } else {
      switch (driveMode) {
        case NORMAL:
          // In normal mode, run the controllers for turning and driving based on the current
          // setpoint
          SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(closedLoopSetpoint);
          SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeed);

          // If stationary, go to last state
          boolean isStationary =
              Math.abs(closedLoopSetpoint.vxMetersPerSecond) < 1e-3
                  && Math.abs(closedLoopSetpoint.vyMetersPerSecond) < 1e-3
                  && Math.abs(closedLoopSetpoint.omegaRadiansPerSecond) < 1e-3;

          SwerveModuleState[] setpointStatesOptimized =
              new SwerveModuleState[] {null, null, null, null};
          for (int i = 0; i < 4; i++) {
            // Run turn controller
            setpointStatesOptimized[i] =
                SwerveModuleState.optimize(setpointStates[i], turnPositions[i]);
            if (isStationary) {
              moduleIOs[i].setTurnVoltage(0.0);
            } else {
              moduleIOs[i].setTurnVoltage(
                  turnFeedback[i].calculate(
                      turnPositions[i].getRadians(),
                      setpointStatesOptimized[i].angle.getRadians()));
            }

            // Update velocity based on turn error
            setpointStatesOptimized[i].speedMetersPerSecond *=
                Math.cos(turnFeedback[i].getPositionError());

            // Run drive controller
            double velocityRadPerSec =
                setpointStatesOptimized[i].speedMetersPerSecond / wheelRadius;
            moduleIOs[i].setDriveVoltage(
                driveFeedforward.calculate(velocityRadPerSec)
                    + driveFeedback[i].calculate(
                        moduleInputs[i].driveVelocityRadPerSec, velocityRadPerSec));

            // Log individual setpoints
            Logger.getInstance()
                .recordOutput("SwerveDriveSetpoints/" + Integer.toString(i), velocityRadPerSec);
            Logger.getInstance()
                .recordOutput(
                    "SwerveTurnSetpoints/" + Integer.toString(i),
                    setpointStatesOptimized[i].angle.getRadians());
          }

          // Log all module setpoints
          logModuleStates("SwerveModuleStates/Setpoints", setpointStates);
          logModuleStates("SwerveModuleStates/SetpointsOptimized", setpointStatesOptimized);
          break;

        case FROMMODULESTATES:

          // In pptrajectory mode, run the controllers for turning and driving based on the
          // setpoint generated by pathplanner
          SwerveDriveKinematics.desaturateWheelSpeeds(ModuleStates, maxLinearSpeed);

          for (int i = 0; i < 4; i++) {

            moduleIOs[i].setTurnVoltage(
                turnFeedback[i].calculate(
                    turnPositions[i].getRadians(), ModuleStates[i].angle.getRadians()));

            // Update velocity based on turn error
            ModuleStates[i].speedMetersPerSecond *= Math.cos(turnFeedback[i].getPositionError());

            // Run drive controller
            double velocityRadPerSec = ModuleStates[i].speedMetersPerSecond / wheelRadius;
            moduleIOs[i].setDriveVoltage(
                driveFeedforward.calculate(velocityRadPerSec)
                    + driveFeedback[i].calculate(
                        moduleInputs[i].driveVelocityRadPerSec, velocityRadPerSec));

            // Log individual setpoints
            Logger.getInstance()
                .recordOutput("SwerveDriveSetpoints/" + Integer.toString(i), velocityRadPerSec);
            Logger.getInstance()
                .recordOutput(
                    "SwerveTurnSetpoints/" + Integer.toString(i),
                    ModuleStates[i].angle.getRadians());
          }

          // Log all module setpoints
          logModuleStates("SwerveModuleStates/Setpoints", ModuleStates);

          break;

        case CHARACTERIZATION:
          // In characterization mode, drive at the specified voltage (and turn to zero degrees)
          for (int i = 0; i < 4; i++) {
            moduleIOs[i].setTurnVoltage(
                turnFeedback[i].calculate(turnPositions[i].getRadians(), 0.0));
            moduleIOs[i].setDriveVoltage(characterizationVoltage);
          }
          break;

        case X:
          for (int i = 0; i < 4; i++) {
            Rotation2d targetRotation = GeomUtil.direction(getModuleTranslations()[i]);
            Rotation2d currentRotation = turnPositions[i];
            if (Math.abs(targetRotation.minus(currentRotation).getDegrees()) > 90.0) {
              targetRotation = targetRotation.minus(Rotation2d.fromDegrees(180.0));
            }
            moduleIOs[i].setTurnVoltage(
                turnFeedback[i].calculate(
                    currentRotation.getRadians(), targetRotation.getRadians()));
            moduleIOs[i].setDriveVoltage(0.0);
          }
          break;
      }
    }

    // // Update odometry
    // SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
    // for (int i = 0; i < 4; i++) {
    // measuredStatesDiff[i] =
    // new SwerveModuleState(
    // (moduleInputs[i].drivePositionRad - lastModulePositionsRad[i]) * wheelRadius,
    // turnPositions[i]);
    // lastModulePositionsRad[i] = moduleInputs[i].drivePositionRad;
    // }
    // ChassisSpeeds chassisStateDiff = kinematics.toChassisSpeeds(measuredStatesDiff);
    // if (gyroInputs.connected) { // Use gyro for angular change when connected
    // odometryPose =
    // odometryPose.exp(
    // new Twist2d(
    // chassisStateDiff.vxMetersPerSecond,
    // chassisStateDiff.vyMetersPerSecond,
    // gyroInputs.positionRad - lastGyroPosRad));
    // } else { // Fall back to using angular velocity (disconnected or sim)
    // odometryPose =
    // odometryPose.exp(
    // new Twist2d(
    // chassisStateDiff.vxMetersPerSecond,
    // chassisStateDiff.vyMetersPerSecond,
    // chassisStateDiff.omegaRadiansPerSecond));
    // }
    // lastGyroPosRad = gyroInputs.positionRad;

    // Update field velocity
    SwerveModuleState[] measuredStates = new SwerveModuleState[] {null, null, null, null};
    for (int i = 0; i < 4; i++) {
      measuredStates[i] =
          new SwerveModuleState(
              moduleInputs[i].driveVelocityRadPerSec * wheelRadius, turnPositions[i]);
    }
    ChassisSpeeds chassisState = kinematics.toChassisSpeeds(measuredStates);
    fieldVelocity =
        new Translation2d(chassisState.vxMetersPerSecond, chassisState.vyMetersPerSecond)
            .rotateBy(getRotation());

    // Log measured states
    logModuleStates("SwerveModuleStates/Measured", measuredStates);

    // Update odometry
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] =
          new SwerveModulePosition(
              ((moduleInputs[i].drivePositionRad * wheelRadius) - lastModulePositionsMeters[i]),
              new Rotation2d(MathUtil.angleModulus(moduleInputs[i].turnAbsolutePositionRad)));
      lastModulePositionsMeters[i] = (moduleInputs[i].drivePositionRad * wheelRadius);
    }
    var twist = kinematics.toTwist2d(wheelDeltas);
    var gyroYaw = new Rotation2d(gyroInputs.positionRad);
    if (gyroInputs.connected) {
      twist = new Twist2d(-twist.dx, -twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
    }
    lastGyroYaw = gyroYaw;
    poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
    Logger.getInstance().recordOutput("Odometry", getPose());

    // // Log odometry pose
    // Logger.getInstance()
    // .recordOutput(
    // "Odometry",
    // new double[] {
    // odometryPose.getX(), odometryPose.getY(), odometryPose.getRotation().getRadians()
    // });

    // Enable/disable brake mode
    if (DriverStation.isEnabled()) {
      if (!brakeMode) {
        brakeMode = true;
        for (int i = 0; i < 4; i++) {
          moduleIOs[i].setTurnBrakeMode(true);
          moduleIOs[i].setDriveBrakeMode(true);
        }
      }
    } else {
      boolean stillMoving = false;
      for (int i = 0; i < 4; i++) {
        if (Math.abs(moduleInputs[i].driveVelocityRadPerSec * wheelRadius)
            > maxCoastVelocityMetersPerSec) {
          stillMoving = true;
        }
      }

      if (brakeMode && !stillMoving) {
        brakeMode = false;
        for (int i = 0; i < 4; i++) {
          moduleIOs[i].setTurnBrakeMode(false);
          moduleIOs[i].setDriveBrakeMode(false);
        }
      }
    }
  }

  private void logModuleStates(String key, SwerveModuleState[] states) {
    List<Double> dataArray = new ArrayList<Double>();
    for (int i = 0; i < 4; i++) {
      dataArray.add(states[i].angle.getRadians());
      dataArray.add(states[i].speedMetersPerSecond);
    }
    Logger.getInstance()
        .recordOutput(key, dataArray.stream().mapToDouble(Double::doubleValue).toArray());
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    driveMode = DriveMode.NORMAL;
    closedLoopSetpoint = speeds;
  }

  public void setModuleStates(SwerveModuleState[] states) {
    driveMode = DriveMode.FROMMODULESTATES;
    ModuleStates = states;
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
  public Command followPPTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                this.setPose(traj.getInitialHolonomicPose());
              }
            }),
        new PPSwerveControllerCommand(
            traj,
            this::getPose, // Pose supplier
            this.kinematics, // SwerveDriveKinematics
            new PIDController(
                22.0, 0, 0), // X controller. Tune these values for your robot. Leaving
            // them
            // 0 will only use feedforwards.
            new PIDController(22.0, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(22.0, 0, 0), // Rotation controller. Tune these values for your robot.
            // Leaving them 0 will only use feedforwards.
            this::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color.
            // Optional,
            // defaults to true
            this // Requires this drive subsystem
            ));
  }

  public void goToX() {
    driveMode = DriveMode.X;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxLinearSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxAngularSpeed;
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    // return odometryPose;
    return poseEstimator.getLatestPose();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    // return odometryPose.getRotation();
    return poseEstimator.getLatestPose().getRotation();
  }

  public double getPitch() {
    return gyroInputs.pitchDeg;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  public Rotation2d getGyroRotation() {
    return new Rotation2d(gyroInputs.positionRad);
  }

  public Translation2d getFieldVelocity() {
    return fieldVelocity;
  }

  /** Returns an array of module translations. */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
      new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0)
    };
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    driveMode = DriveMode.CHARACTERIZATION;
    characterizationVoltage = volts;
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (int i = 0; i < 4; i++) {
      driveVelocityAverage += moduleInputs[i].driveVelocityRadPerSec;
    }
    return driveVelocityAverage / 4.0;
  }

  private static enum DriveMode {
    NORMAL,
    FROMMODULESTATES,
    X,
    CHARACTERIZATION,
  }
}
