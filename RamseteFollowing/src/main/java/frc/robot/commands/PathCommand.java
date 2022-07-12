package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.RomiDrivetrain;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.AutoConstants.*;
public class PathCommand extends CommandBase{
    RomiDrivetrain drive;
    Pose2d initialPose;
    List<Pose2d> poses=new ArrayList<>();
    RamseteCommand currCommand;
    public PathCommand(RomiDrivetrain drive,Pose2d initialPose,List<Pose2d> pts,Pose2d finalPose){
        this.drive=drive;
        this.initialPose=initialPose;
        poses=pts;
        poses.add(finalPose);
        createNextCommand();
    }
    public void execute(){
        SmartDashboard.putNumber("Pts left", poses.size());
        currCommand.execute();
        Translation2d error=drive.getPose().minus(poses.get(0)).getTranslation();
        boolean near=(Math.abs(error.getX())<.05&&Math.abs(error.getY())<.05);
        if(currCommand.isFinished()||near){
            if(near)
            poses.remove(0);
            if(poses.size()>0){
                createNextCommand();
            }
        }
    }
    public boolean isFinished(){
        return poses.size()==0;
    }
    private void createNextCommand(){
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(ksVolts, 
                                       kvVoltSecondsPerMeter, 
                                       kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            6);

    TrajectoryConfig config =
        new TrajectoryConfig(kMaxSpeedMetersPerSecond, 
                             kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(kDriveKinematics)
            .addConstraint(autoVoltageConstraint);
    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    Pose2d nextPose=poses.get(0);
    List<Translation2d> trans=new ArrayList<Translation2d>();
    trans.add(nextPose.getTranslation());//We need this so the program does not crash
    System.out.println(nextPose.toString());
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        drive.getPose(),
        trans,
        poses.get(poses.size()-1),
        config);
    double expectedTime=exampleTrajectory.getTotalTimeSeconds();
    SmartDashboard.putNumber("ExpectedTime",expectedTime);
    Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds=() -> drive.getWheelSpeeds();
    BiConsumer<Double,Double> bc=(left, right) -> drive.tankDriveVolts(left,right);
    RamseteController rc=new RamseteController(kRamseteB, kRamseteZeta);
    currCommand = new RamseteCommand(
        exampleTrajectory,
        drive::getPose,
        rc,
        new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
        kDriveKinematics,
        wheelSpeeds,
        new PIDController(kPDriveVel, 0, 0),
        new PIDController(kPDriveVel, 0, 0),
        bc,
        drive);
    currCommand.schedule();
    }
}
