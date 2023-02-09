// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm_Commands.ArmCommand;
import frc.robot.commands.Auto_Commands.DriveForward;
import frc.robot.commands.Drive_Commands.AutoBalance;
import frc.robot.commands.Drive_Commands.DriveToNode;
import frc.robot.commands.Drive_Commands.DriveToTarget;
import frc.robot.commands.Drive_Commands.FeedForwardCharacterization;
import frc.robot.commands.Drive_Commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.Drive_Commands.TraditionalDriveCommand;
import frc.robot.commands.Drive_Commands.TraditionalDriveCommand.JoystickMode;
import frc.robot.commands.Grab_Commands.GrabCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extender.Extender;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.drivers.SparkMAXBurnManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final Pose2d autoDriveTarget = new Pose2d(0.0, 0.0, new Rotation2d());

  // Subsystems
  private Drive drive;
  private Arm arm;
  private Extender extender;
  private Vision vision;
  private Grabber grabber;

  // OI objects
  private XboxController driverController = new XboxController(0);
  private XboxController operatorController = new XboxController(1);

  private boolean isFieldRelative = true;

  // Choosers
  private final LoggedDashboardChooser<AutoRoutine> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  private final LoggedDashboardChooser<JoystickMode> joystickModeChooser =
      new LoggedDashboardChooser<>("Joystick Mode");
  private final LoggedDashboardChooser<Double> demoLinearSpeedLimitChooser =
      new LoggedDashboardChooser<>("Linear Speed Limit");
  private final LoggedDashboardChooser<Double> demoAngularSpeedLimitChooser =
      new LoggedDashboardChooser<>("Angular Speed Limit");
  private final LoggedDashboardChooser<Double> targetNodeChooser =
      new LoggedDashboardChooser<>("Target Node");
  private final LoggedDashboardChooser<Double> targetLevelChooser =
      new LoggedDashboardChooser<>("Target Level");

  Field2d field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Check if flash should be burned
    SparkMAXBurnManager.update();

    // Instantiate active subsystems
    drive = Drive.getInstance();
    arm = Arm.getInstance();
    extender = Extender.getInstance();
    vision = Vision.getInstance();
    grabber = Grabber.getInstance();

    // Set up auto routines
    autoChooser.addDefaultOption(
        "Do Nothing", new AutoRoutine(AutoPosition.ORIGIN, new InstantCommand()));
    autoChooser.addOption(
        "Drive Forward", new AutoRoutine(AutoPosition.ORIGIN, new DriveForward(drive, false)));
    // autoChooser.addOption(
    //     "Path Planner Test", new AutoRoutine(AutoPosition.ORIGIN, new PPTest(drive)));
    autoChooser.addOption(
        "Leave Community Left",
        new AutoRoutine(AutoPosition.COMMUNITY_LEFT, new DriveForward(drive, false)));
    autoChooser.addOption(
        "Leave Community Right",
        new AutoRoutine(AutoPosition.COMMUNITY_RIGHT, new DriveForward(drive, true)));
    autoChooser.addOption(
        "Drive On To Charge Station",
        new AutoRoutine(
            AutoPosition.CHARGE_STATION_START,
            new WaitCommand(5)
                .andThen(new DriveForward(drive, false))
                .andThen(new AutoBalance(drive))));
    autoChooser.addOption(
        "Drive Characterization",
        new AutoRoutine(
            AutoPosition.ORIGIN,
            new FeedForwardCharacterization(
                drive,
                true,
                new FeedForwardCharacterizationData("drive"),
                drive::runCharacterizationVolts,
                drive::getCharacterizationVelocity)));

    // Set up choosers

    // Target Level Chooser
    targetLevelChooser.addDefaultOption("Low", 1.0);
    targetLevelChooser.addOption("Mid", 2.0);
    targetLevelChooser.addOption("High", 3.0);

    // Target Node Chooser
    targetNodeChooser.addDefaultOption("Node 1 (Cone)", 1.0);
    targetNodeChooser.addOption("Node 2 (Cube)", 2.0);
    targetNodeChooser.addOption("Node 3 (Cone)", 3.0);
    targetNodeChooser.addOption("Node 4 CO-OP (Cone)", 4.0);
    targetNodeChooser.addOption("Node 5 CO-OP (Cube)", 5.0);
    targetNodeChooser.addOption("Node 6 CO-OP (Cone)", 6.0);
    targetNodeChooser.addOption("Node 7 Outer (Cone)", 7.0);
    targetNodeChooser.addOption("Node 8 Outer (Cube)", 8.0);
    targetNodeChooser.addOption("Node 9 Outer (Cone)", 9.0);

    // Joystick Mode Chooser
    joystickModeChooser.addDefaultOption("Standard", JoystickMode.Standard);
    joystickModeChooser.addOption("Tank", JoystickMode.Tank);

    // Speed Choosers
    demoLinearSpeedLimitChooser.addDefaultOption("--Competition Mode--", 1.0);
    demoLinearSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    demoLinearSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    demoLinearSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);
    demoAngularSpeedLimitChooser.addDefaultOption("--Competition Mode--", 1.0);
    demoAngularSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    demoAngularSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    demoAngularSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, expect decreased network performance.", AlertType.INFO)
          .set(true);
    }

    logAprilTagPoses();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    arm.setDefaultCommand(
        new ArmCommand(
            arm,
            extender,
            () -> operatorController.getLeftY(),
            () -> operatorController.getRightY()));

    grabber.setDefaultCommand(
        new GrabCommand(
            grabber, () -> operatorController.getBButton(), () -> operatorController.getXButton()));

    // Driving controls
    new Trigger(driverController::getStartButton)
        .or(new Trigger(driverController::getBackButton))
        .onTrue(
            new InstantCommand(
                    () -> {
                      isFieldRelative = !isFieldRelative;
                      SmartDashboard.putBoolean("Field Relative", isFieldRelative);
                    })
                .ignoringDisable(true));

    SmartDashboard.putBoolean("Field Relative", isFieldRelative);

    drive.setDefaultCommand(
        new TraditionalDriveCommand(
            drive,
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> driverController.getRightX(),
            () -> !isFieldRelative,
            () -> joystickModeChooser.get(),
            () -> demoLinearSpeedLimitChooser.get(),
            () -> demoAngularSpeedLimitChooser.get(),
            () -> targetNodeChooser.get()));

    // new Trigger(driverController::getAButton).toggleOnTrue(
    // new StickAngleDriveCommand(drive, () -> driverController.getLeftX(),
    // () -> -driverController.getLeftY(),
    // () -> -driverController.getRightY(),
    // () -> driverController.getRightX(), () -> false,
    // () -> demoLinearSpeedLimitChooser.get(),
    // () -> demoAngularSpeedLimitChooser.get(),
    // () -> driverController.getRightBumper(),
    // () -> driverController.getLeftBumper(),
    // () -> driverController.getBButton(),
    // () -> targetNodeChooser.get()));

    new Trigger(driverController::getXButton).toggleOnTrue(new AutoBalance(drive));

    // new Trigger(driverController::getYButton)
    // .toggleOnTrue(new DriveToPose(drive, drive.getPose()));

    new Trigger(driverController::getYButton).toggleOnTrue(new DriveToTarget(drive));

    new Trigger(driverController::getBButton)
        .toggleOnTrue(new DriveToNode(drive, () -> targetNodeChooser.get()));

    // Reset gyro command
    Command resetGyroCommand =
        new InstantCommand(
                () -> {
                  drive.setPose(autoDriveTarget);
                },
                drive)
            .ignoringDisable(true);

    Command rumbleCommand =
        new StartEndCommand(
            () -> driverController.setRumble(RumbleType.kRightRumble, 0.5),
            () -> driverController.setRumble(RumbleType.kRightRumble, 0.0)) {
          @Override
          public boolean runsWhenDisabled() {
            return true;
          }
        }.withTimeout(0.2);

    new Trigger(driverController::getLeftBumper)
        .and(new Trigger(driverController::getRightBumper))
        .onTrue(resetGyroCommand)
        .onTrue(rumbleCommand);

    // Auto drive controls
    // new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
    // .whileTrue(new AutoDriveHard(drive));

  }

  // Send April Tags to Logger
  private void logAprilTagPoses() {
    Logger.getInstance().recordOutput("AprilTags/Tag1", FieldConstants.aprilTags.get(1));
    Logger.getInstance().recordOutput("AprilTags/Tag2", FieldConstants.aprilTags.get(2));
    Logger.getInstance().recordOutput("AprilTags/Tag3", FieldConstants.aprilTags.get(3));
    Logger.getInstance().recordOutput("AprilTags/Tag4", FieldConstants.aprilTags.get(4));
    Logger.getInstance().recordOutput("AprilTags/Tag5", FieldConstants.aprilTags.get(5));
    Logger.getInstance().recordOutput("AprilTags/Tag6", FieldConstants.aprilTags.get(6));
    Logger.getInstance().recordOutput("AprilTags/Tag7", FieldConstants.aprilTags.get(7));
    Logger.getInstance().recordOutput("AprilTags/Tag8", FieldConstants.aprilTags.get(8));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutoRoutine routine = autoChooser.get();
    drive.setPose(routine.position.getPose());
    return routine.command;
  }

  public double getTargetNode() {
    Double node = targetNodeChooser.get();
    return node;
  }

  private static class AutoRoutine {
    public final AutoPosition position;
    public final Command command;

    public AutoRoutine(AutoPosition position, Command command) {
      this.position = position;
      this.command = command;
    }
  }

  public static enum AutoPosition {
    ORIGIN,
    CHARGE_STATION_START,
    COMMUNITY_LEFT,
    COMMUNITY_RIGHT;

    public Pose2d getPose() {
      switch (this) {
        case ORIGIN:
          return new Pose2d();
        case CHARGE_STATION_START:
          return autoDriveTarget;
        case COMMUNITY_LEFT:
          return new Pose2d(2.0, 4.7, new Rotation2d());
        case COMMUNITY_RIGHT:
          return new Pose2d(2.0, 0.7, new Rotation2d());
        default:
          return new Pose2d();
      }
    }
  }
}
