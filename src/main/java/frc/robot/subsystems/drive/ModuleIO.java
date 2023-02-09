// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO {

  // @AutoLog
  public static class ModuleIOInputs implements LoggableInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveVelocityFilteredRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};
    public double[] driveTempCelcius = new double[] {};

    public double turnAbsolutePositionRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};
    public double[] turnTempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("drivePositionRad", drivePositionRad);
      table.put("driveVelocityRadPerSec", driveVelocityRadPerSec);
      table.put("driveVelocityFilteredRadPerSec", driveVelocityFilteredRadPerSec);
      table.put("driveAppliedVolts", driveAppliedVolts);
      table.put("driveCurrentAmps", driveCurrentAmps);
      table.put("driveTempCelcius", driveTempCelcius);

      table.put("turnAbsolutePositionRad", turnAbsolutePositionRad);
      table.put("turnPositionRad", turnPositionRad);
      table.put("turnVelocityRadPerSec", turnVelocityRadPerSec);
      table.put("turnAppliedVolts", turnAppliedVolts);
      table.put("turnCurrentAmps", turnCurrentAmps);
      table.put("turnTempCelcius", turnTempCelcius);
    }

    public void fromLog(LogTable table) {
      drivePositionRad = table.getDouble("drivePositionRad", drivePositionRad);
      driveVelocityRadPerSec = table.getDouble("driveVelocityRadPerSec", driveVelocityRadPerSec);
      driveVelocityFilteredRadPerSec =
          table.getDouble("driveVelocityFilteredRadPerSec", driveVelocityFilteredRadPerSec);
      driveAppliedVolts = table.getDouble("driveAppliedVolts", driveAppliedVolts);
      driveCurrentAmps = table.getDoubleArray("driveCurrentAmps", driveCurrentAmps);
      driveTempCelcius = table.getDoubleArray("driveTempCelcius", driveTempCelcius);

      turnAbsolutePositionRad = table.getDouble("turnAbsolutePositionRad", turnAbsolutePositionRad);
      turnPositionRad = table.getDouble("turnPositionRad", turnPositionRad);
      turnPositionRad = table.getDouble("turnPositionRad", turnPositionRad);
      turnAppliedVolts = table.getDouble("turnAppliedVolts", turnAppliedVolts);
      turnCurrentAmps = table.getDoubleArray("turnCurrentAmps", turnCurrentAmps);
      turnTempCelcius = table.getDoubleArray("turnTempCelcius", turnTempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}
}
