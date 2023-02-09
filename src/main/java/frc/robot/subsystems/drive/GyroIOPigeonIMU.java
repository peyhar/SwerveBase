// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** IO implementation for Pigeon2 */
public class GyroIOPigeonIMU implements GyroIO {
  private final PigeonIMU gyro;
  private final double[] xyzDps = new double[3];

  public GyroIOPigeonIMU() {
    switch (Constants.getRobot()) {
      case ROBOT_2023:
        gyro = new PigeonIMU(12);
        break;
      default:
        throw new RuntimeException("Invalid robot for GyroIOPigeonIMU");
    }
  }

  public void updateInputs(GyroIOInputs inputs) {
    // Again, the 2022 code base has good examples for reading this data. We generally prefer to use
    // "getAngle" instead of "getYaw" (what's the difference?)
    //
    // Remember to pay attention to the UNITS.
    gyro.getRawGyro(xyzDps);
    inputs.connected = gyro.getLastError().equals(ErrorCode.OK);
    inputs.positionRad = Units.degreesToRadians(gyro.getFusedHeading());
    inputs.velocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
  }

  public void setHeading(double angleDeg) {
    gyro.setFusedHeading(angleDeg);
  }
}
