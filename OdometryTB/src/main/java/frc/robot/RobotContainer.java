// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.UpdateOdometry;
import frc.robot.sensors.RomiGyro;
import frc.robot.sensors.RomiOdometer;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain romiDrivetrain = new RomiDrivetrain();
  private final GenericHID controller=new XboxController(0);
  private RomiGyro gyro=new RomiGyro();
  private RomiOdometer od=new RomiOdometer(gyro);
  private final UpdateOdometry upOd=new UpdateOdometry(od);
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
    romiDrivetrain.setDefaultCommand(getArcadeDriveCommand());
    od.setDefaultCommand(upOd);
    JoystickButton aButton=new JoystickButton(controller, 1);
    aButton.whenPressed(new InstantCommand(new Runnable() {

      @Override
      public void run() {
        System.out.println(od.getX());
        
      }
      
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getArcadeDriveCommand(){
      
      return new ArcadeDrive(romiDrivetrain,() -> controller.getRawAxis(1), () -> controller.getRawAxis(0));
    }
}
