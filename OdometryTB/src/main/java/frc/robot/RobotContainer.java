// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FODrive;
import frc.robot.commands.PathFollower;
import frc.robot.commands.RamseteCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.commands.UpdateOdometry;
import frc.robot.sensors.RomiGyro;
import frc.robot.sensors.RomiOdometer;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private RomiGyro gyro=new RomiGyro();
  private RomiOdometer od=new RomiOdometer(gyro);
  private UpdateOdometry upOd=new UpdateOdometry(od);
  private PathFollower pathFollower=new PathFollower(drive, od, new Pose2d(0,0,new Rotation2d()));
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
    drive.setDefaultCommand(getFOCommand());
    od.setDefaultCommand(upOd);
    JoystickButton aButton=new JoystickButton(controller, 1);
    aButton.whenPressed(new InstantCommand(new Runnable() {

      @Override
      public void run() {
        System.out.println(od.getX());
        
      }
      
    }));
    JoystickButton startButton=new JoystickButton(controller, 2);
    startButton.whenPressed(PathFollower.followPath(drive, od, new Translation2d[]{
      new Translation2d(0,0),
    })).whenReleased(new InstantCommand(new Runnable() {

      @Override
      public void run() {
        //pathFollower.cancel();
        
      }
      
    }));
  }
  public Command getAutoCommand(){
    return new SequentialCommandGroup(
        PathFollower.followPath(drive, od, new Translation2d[]{
          new Translation2d(-40,-40),
          new Translation2d(-100,-40),
          new Translation2d(-100,0)
        }),
        new TurnCommand(drive, gyro, 0)
    );
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getArcadeDriveCommand(){
      
      return new ArcadeDrive(drive,() -> controller.getRawAxis(1), () -> controller.getRawAxis(0));
    }
    public Command getFOCommand(){
      
      return new FODrive(drive,gyro,() -> controller.getRawAxis(0), () -> controller.getRawAxis(1));
    }
}
