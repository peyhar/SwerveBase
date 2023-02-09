// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public interface GyroIO {

  // @AutoLog
  public static class GyroIOInputs implements LoggableInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double pitchDeg = 0.0;
    public double velocityRadPerSec = 0.0;

    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("PositionRad", positionRad);
      table.put("VelocityRadPerSec", velocityRadPerSec);
      table.put("PitchDeg", pitchDeg);
    }

    public void fromLog(LogTable table) {
      connected = table.getBoolean("Connected", connected);
      positionRad = table.getDouble("PositionRad", positionRad);
      velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
      pitchDeg = table.getDouble("PitchDeg", pitchDeg);
    }
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  /** Set the Gyroscope heading to a value in degrees. */
  public default void setHeading(double angleDeg) {}
}
