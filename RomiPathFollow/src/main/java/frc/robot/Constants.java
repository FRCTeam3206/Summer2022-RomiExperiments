// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        // Generated from SysId file "sysid_data20220716-212236.json"
        public static final double ksVolts = 0.16645;
        public static final double kvVoltSecondsPerMeter = 10.363;
        public static final double kaVoltSecondsSquaredPerMeter = 0.92327;

        public static final double kPDriveVal = 6.9641;

        // Track width for kinematics
        public static final double kTrackWidthMeters = 0.141;  // known value from Romi documentation
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackWidthMeters);
        
        // Max speed (should be less than top speed) and acceleration (not as critical)
        public static final double kMaxSpeedMetersPerSecond = 0.6;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.6;
        
        // Ramsete parameters
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
