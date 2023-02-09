// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalonFX;
  private final TalonFX turnTalonFX;

  private final CANCoder turnAbsoluteEncoder;

  private final double driveAfterEncoderReduction;
  private final double turnAfterEncoderReduction = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final boolean isAbsoluteEncoderInverted = false;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOTalonFX(int index) {
    switch (Constants.getRobot()) {
      case ROBOT_2023:
        switch (index) {
            // Front Left
          case 0:
            driveTalonFX = new TalonFX(0);
            turnTalonFX = new TalonFX(2);
            turnAbsoluteEncoder = new CANCoder(40);
            absoluteEncoderOffset = new Rotation2d(2.44);
            break;

            // Front Right
          case 1:
            driveTalonFX = new TalonFX(3);
            turnTalonFX = new TalonFX(5);
            turnAbsoluteEncoder = new CANCoder(20);
            absoluteEncoderOffset = new Rotation2d(-2.32);
            break;

            // Back Left
          case 2:
            driveTalonFX = new TalonFX(9);
            turnTalonFX = new TalonFX(11);
            turnAbsoluteEncoder = new CANCoder(10);
            absoluteEncoderOffset = new Rotation2d(1.30);
            break;

            // Back Right
          case 3:
            driveTalonFX = new TalonFX(6);
            turnTalonFX = new TalonFX(8);
            turnAbsoluteEncoder = new CANCoder(4);
            absoluteEncoderOffset = new Rotation2d(-1.98);
            break;
          default:
            throw new RuntimeException("Invalid module index for ModuleIOTalonFX");
        }
        driveAfterEncoderReduction = 8.14;

        driveTalonFX.configFactoryDefault();
        turnTalonFX.configFactoryDefault();

        turnAbsoluteEncoder.configFactoryDefault();

        turnTalonFX.setInverted(isTurnMotorInverted);
        turnTalonFX.setInverted(true);

        break;
      case ROBOT_2023P:
        switch (index) {
            // Front Left
          case 0:
            driveTalonFX = new TalonFX(0);
            turnTalonFX = new TalonFX(2);
            turnAbsoluteEncoder = new CANCoder(40);
            absoluteEncoderOffset = new Rotation2d(2.44);
            break;

            // Front Right
          case 1:
            driveTalonFX = new TalonFX(3);
            turnTalonFX = new TalonFX(5);
            turnAbsoluteEncoder = new CANCoder(20);
            absoluteEncoderOffset = new Rotation2d(-2.32);
            break;

            // Back Left
          case 2:
            driveTalonFX = new TalonFX(9);
            turnTalonFX = new TalonFX(11);
            turnAbsoluteEncoder = new CANCoder(10);
            absoluteEncoderOffset = new Rotation2d(1.30);
            break;

            // Back Right
          case 3:
            driveTalonFX = new TalonFX(6);
            turnTalonFX = new TalonFX(8);
            turnAbsoluteEncoder = new CANCoder(4);
            absoluteEncoderOffset = new Rotation2d(-1.98);
            break;
          default:
            throw new RuntimeException("Invalid module index for ModuleIOTalonFX");
        }
        driveAfterEncoderReduction = 8.14;

        driveTalonFX.configFactoryDefault();
        turnTalonFX.configFactoryDefault();

        turnAbsoluteEncoder.configFactoryDefault();

        turnTalonFX.setInverted(false);
        break;
      default:
        throw new RuntimeException("Invalid robot for ModuleIOTalonFX");
    }

    turnTalonFX.setSelectedSensorPosition(0.0);

    driveTalonFX.setSelectedSensorPosition(0.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveTalonFX.getSelectedSensorPosition() / 2048)
            / driveAfterEncoderReduction;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
                driveTalonFX.getSelectedSensorVelocity() / 2048 * Units.millisecondsToSeconds(100))
            / driveAfterEncoderReduction;
    inputs.driveVelocityFilteredRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
                driveTalonFX.getSelectedSensorVelocity() / 2048 * Units.millisecondsToSeconds(100))
            / driveAfterEncoderReduction;
    inputs.driveAppliedVolts =
        driveTalonFX.getMotorOutputVoltage() * RobotController.getBatteryVoltage();

    inputs.driveCurrentAmps = new double[] {driveTalonFX.getSupplyCurrent()};
    inputs.driveTempCelcius = new double[] {driveTalonFX.getTemperature()};

    // double absolutePositionPercent =
    //     turnAbsoluteEncoder.getBusVoltage() / RobotController.getVoltage5V();
    // if (isAbsoluteEncoderInverted) {
    //   absolutePositionPercent = 1 - absolutePositionPercent;
    // }

    inputs.turnAbsolutePositionRad =
        Rotation2d.fromDegrees(turnAbsoluteEncoder.getAbsolutePosition())
            .minus(absoluteEncoderOffset)
            .getRadians();
    inputs.turnPositionRad =
        Units.rotationsToRadians(turnTalonFX.getSelectedSensorPosition() / 2048)
            / turnAfterEncoderReduction;
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
                turnTalonFX.getSelectedSensorVelocity() / 2048 * Units.millisecondsToSeconds(100))
            / turnAfterEncoderReduction;
    inputs.turnAppliedVolts =
        turnTalonFX.getMotorOutputVoltage() * RobotController.getBatteryVoltage();
    inputs.turnCurrentAmps = new double[] {turnTalonFX.getSupplyCurrent()};
    inputs.turnTempCelcius = new double[] {turnTalonFX.getTemperature()};
  }

  public void setDriveVoltage(double volts) {
    driveTalonFX.set(ControlMode.PercentOutput, volts / 12.0);
  }

  public void setTurnVoltage(double volts) {
    turnTalonFX.set(ControlMode.PercentOutput, volts / 12.0);
  }

  public void setDriveBrakeMode(boolean enable) {
    driveTalonFX.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public void setTurnBrakeMode(boolean enable) {
    turnTalonFX.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }
}
