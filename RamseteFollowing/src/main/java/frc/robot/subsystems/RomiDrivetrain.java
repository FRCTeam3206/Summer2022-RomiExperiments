package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeter = 0.07;

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);

  // Set up the RomiGyro
  private final RomiGyro gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;

  // Also show a field diagram
  private final Field2d field2d = new Field2d();

  /** Creates a new Drivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    resetEncoders();

    odometry = new DifferentialDriveOdometry(gyro.getHeading());
    SmartDashboard.putData("field", field2d);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(-leftVolts);
    rightMotor.setVoltage(rightVolts); // We invert this to maintain +ve = forward
    diffDrive.feed();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return rightEncoder.get();
  }

  public double getLeftDistanceMeter() {
    return leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return rightEncoder.getDistance();
  }

  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(gyro.getHeading(), leftEncoder.getDistance(), rightEncoder.getDistance());
    SmartDashboard.putNumber("X",getPose().getX());
    SmartDashboard.putNumber("Y",getPose().getY());
    SmartDashboard.putNumber("T",getPose().getRotation().getDegrees());
    // Also update the Field2D object (so that we can visualize this in sim)
    field2d.setRobotPose(getPose());
  }

  /**
   * Returns the currently estimated pose of the robot.
   * @return The pose
   */
  public Pose2d getPose() {
    Pose2d pose= odometry.getPoseMeters();
    return new Pose2d(-pose.getY(),-pose.getX(),new Rotation2d(Math.PI/2-pose.getRotation().getRadians()));
  }

  public Field2d getField(){
    return field2d;
  }

  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose
   * @param pose The pose to which to set the odometry
   */

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly
   * @param maxOutput The maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    diffDrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot
   * @return The robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getHeading().getDegrees();
  }
  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(pose, pose.getRotation());
  }
}