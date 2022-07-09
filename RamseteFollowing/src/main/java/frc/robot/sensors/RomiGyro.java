// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.SimDouble;

public class RomiGyro {
  private SimDouble simRateX;
  private SimDouble simRateY;
  private SimDouble simRateZ;
  private SimDouble simAngleX;
  private SimDouble simAngleY;
  private SimDouble simAngleZ;

  private double angleXOffset;
  private double angleYOffset;
  private double angleZOffset;

  /** Create a new RomiGyro. */
  public RomiGyro() {
    SimDevice gyroSimDevice = SimDevice.create("Gyro:RomiGyro");
    if (gyroSimDevice != null) {
      gyroSimDevice.createBoolean("init", Direction.kOutput, true);
      simRateX = gyroSimDevice.createDouble("rate_x", Direction.kInput, 0.0);
      simRateY = gyroSimDevice.createDouble("rate_y", Direction.kInput, 0.0);
      simRateZ = gyroSimDevice.createDouble("rate_z", Direction.kInput, 0.0);

      simAngleX = gyroSimDevice.createDouble("angle_x", Direction.kInput, 0.0);
      simAngleY = gyroSimDevice.createDouble("angle_y", Direction.kInput, 0.0);
      simAngleZ = gyroSimDevice.createDouble("angle_z", Direction.kInput, 0.0);
      angleZOffset=simAngleZ.get();
    }
  }

  /**
   * Get the rate of turn in degrees-per-second around the X-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateX() {
    if (simRateX != null) {
      return simRateX.get();
    }

    return 0.0;
  }

  /**
   * Get the rate of turn in degrees-per-second around the Y-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateY() {
    if (simRateY != null) {
      return simRateY.get();
    }

    return 0.0;
  }

  /**
   * Get the rate of turn in degrees-per-second around the Z-axis.
   *
   * @return rate of turn in degrees-per-second
   */
  public double getRateZ() {
    if (simRateZ != null) {
      return simRateZ.get();
    }

    return 0.0;
  }

  /**
   * Get the currently reported angle around the X-axis.
   *
   * @return current angle around X-axis in degrees
   */
  public double getAngleX() {
    if (simAngleX != null) {
      return simAngleX.get() - angleXOffset;
    }

    return 0.0;
  }

  /**
   * Get the currently reported angle around the X-axis.
   *
   * @return current angle around Y-axis in degrees
   */
  public double getAngleY() {
    if (simAngleY != null) {
      return simAngleY.get() - angleYOffset;
    }

    return 0.0;
  }

  /**
   * Get the currently reported angle around the Z-axis.
   *
   * @return current angle around Z-axis in degrees
   */
  public double getAngleZ() {
    if (simAngleZ != null) {
      return simAngleZ.get() - angleZOffset;
    }

    return 0.0;
  }
  public double getProcessedAngle(){
    double currAngle=(double)Math.floorMod((long)getAngleZ(),360l);
        if(currAngle>180){
            currAngle-=360;
        }
     return -currAngle;
  }
  public Rotation2d getHeading(){
      return new Rotation2d(getAngleZ()/180*Math.PI);
  }
  /** Reset the gyro angles to 0. */
  public void reset() {
    if (simAngleX != null) {
      angleXOffset = simAngleX.get();
      angleYOffset = simAngleY.get();
      angleZOffset = simAngleZ.get();
    }
    angleZOffset=simAngleZ.get();
  }
}