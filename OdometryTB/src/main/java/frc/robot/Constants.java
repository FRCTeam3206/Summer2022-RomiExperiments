// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DIST_PER_ENCODER_PULSE=Math.PI*7/1440;
    public static double turnToAngle(double curr,double goal){
        double angleError=goal-curr;
        SmartDashboard.putNumber("Error", angleError);
        if(angleError>180){
            angleError-=360;
        }
        if(angleError<-180){
            angleError+=360;
        }
        double mul=1;
        if(angleError>0){
            mul=-1;
        }
        return mul*(1-Math.pow((1-Math.abs(angleError/180)),5));
    }
}
