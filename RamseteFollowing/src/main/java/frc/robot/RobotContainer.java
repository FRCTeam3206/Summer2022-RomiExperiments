// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Filesystem;
import java.util.function.BiConsumer;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.function.Supplier;

import org.opencv.ml.StatModel;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.AutoConstants.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain drive = new RomiDrivetrain();
  private final GenericHID controller=new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(getArcadeDriveCommand(controller));
    JoystickButton startButton=new JoystickButton(controller, 2);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return getRamseteCommand();
  }
  public Trajectory getTrajectoryFromPathFile(String path) throws FileNotFoundException{
    Scanner scan=new Scanner(new FileInputStream(Filesystem.getDeployDirectory().getAbsolutePath()+"/"+path));
    scan.nextLine();//Skip headers
    ArrayList<Translation2d> trans=new ArrayList<Translation2d>();
    String line;
    Scanner lscan=null;
    boolean first=true;
    double startAngle=0,endAngle=0;
    while(scan.hasNextLine()){
      line=scan.nextLine();
      lscan=new Scanner(line).useDelimiter(",");
      double x=lscan.nextDouble();
      double y=2.2+lscan.nextDouble();
      if(first){
        double tx=lscan.nextDouble();
        double ty=lscan.nextDouble();
        startAngle=Math.atan(ty/tx);
        first=false;
      }
      trans.add(new Translation2d(x,y));
    }
    double tx=lscan.nextDouble();
    double ty=lscan.nextDouble();
    endAngle=Math.atan(ty/tx);
    Pose2d start=new Pose2d(trans.get(0),new Rotation2d(startAngle));
    Pose2d end=new Pose2d(trans.get(trans.size()-1),new Rotation2d(endAngle));
    trans.remove(0);
    trans.remove(trans.size()-1);
    
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
    return TrajectoryGenerator.generateTrajectory(start, trans, end, config);
  }
  public Command getPathWeaverCommand(){
    String trajectoryJSON = "paths/parklast.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      ex.printStackTrace();
   }
   drive.resetOdometry(trajectory.getInitialPose());
   Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds=() -> drive.getWheelSpeeds();
    BiConsumer<Double,Double> bc=(left, right) -> drive.tankDriveVolts(left,right);
    RamseteController rc=new RamseteController(kRamseteB, kRamseteZeta);
    //rc.setEnabled(false);
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        drive::getPose,
        rc,
        new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
        kDriveKinematics,
        wheelSpeeds,
        new PIDController(kPDriveVel, 0, 0),
        new PIDController(kPDriveVel, 0, 0),
        bc,
        drive);
    return ramseteCommand

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> drive.tankDriveVolts(0, 0), drive));
  }
  public Command getRamseteCommand(){
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
    List<Translation2d> trans=new ArrayList<Translation2d>();
    trans.add(new Translation2d(1,0));
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        drive.getPose(),
        trans,
        new Pose2d(),
        config);
    try {
      exampleTrajectory=getTrajectoryFromPathFile("paths/simplepark.path");
    } catch (FileNotFoundException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    drive.resetOdometry(exampleTrajectory.getInitialPose());
    double expectedTime=exampleTrajectory.getTotalTimeSeconds();
    SmartDashboard.putNumber("ExpectedTime",expectedTime);
    Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds=() -> drive.getWheelSpeeds();
    BiConsumer<Double,Double> bc=(left, right) -> drive.tankDriveVolts(left,right);
    RamseteController rc=new RamseteController(kRamseteB, kRamseteZeta);
    RamseteCommand ramseteCommand = new RamseteCommand(
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
    return ramseteCommand

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> drive.tankDriveVolts(0, 0), drive));
  }
  public Command getArcadeDriveCommand(GenericHID controller){
      
    return new ArcadeDrive(drive,() -> controller.getRawAxis(0), () -> controller.getRawAxis(1));
  }
}
