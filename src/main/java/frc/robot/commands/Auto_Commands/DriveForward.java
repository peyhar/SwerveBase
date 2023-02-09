// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.Auto_Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class DriveForward extends CommandBase {
  private static LoggedTunableNumber longDuration =
      new LoggedTunableNumber("DriveForward/LongDurationSecs");
  private static LoggedTunableNumber shortDuration =
      new LoggedTunableNumber("DriveForward/ShortDurationSecs");
  private static LoggedTunableNumber speed = new LoggedTunableNumber("DriveForward/Speed");

  private final Drive drive;
  private final boolean isLong;
  private final Timer timer = new Timer();

  /** Drives straight forward. */
  public DriveForward(Drive drive, boolean isLong) {
    addRequirements(drive);
    this.drive = drive;
    this.isLong = isLong;

    longDuration.initDefault(2.0);
    shortDuration.initDefault(1.0);
    speed.initDefault(Units.inchesToMeters(70.0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.runVelocity(new ChassisSpeeds(speed.get(), 0.0, 0.0));
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(isLong ? longDuration.get() : shortDuration.get());
  }
}
