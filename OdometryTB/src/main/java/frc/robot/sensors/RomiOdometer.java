package frc.robot.sensors;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
public class RomiOdometer extends SubsystemBase{
    private RomiGyro gyro;
    private DifferentialDriveOdometry od;
    private Encoder leftEncoder=new Encoder(4,5);
    private Encoder rightEncoder=new Encoder(6,7);
    public RomiOdometer(RomiGyro gyro){
        this.gyro=gyro;
        od=new DifferentialDriveOdometry(gyro.getHeading());
        leftEncoder.setDistancePerPulse(DIST_PER_ENCODER_PULSE);
        rightEncoder.setDistancePerPulse(DIST_PER_ENCODER_PULSE);
    }
    public void update(){
        od.update(gyro.getHeading(), leftEncoder.getDistance(), rightEncoder.getDistance());
        SmartDashboard.putNumber("X", od.getPoseMeters().getX());
        SmartDashboard.putNumber("Y", od.getPoseMeters().getY());
        SmartDashboard.putNumber("T", gyro.getHeading().getDegrees());
    }
    public double getX(){
        return od.getPoseMeters().getX();
    }
    public double getY(){
        return od.getPoseMeters().getY();
    }
    public Pose2d getPose(){
        return new Pose2d(od.getPoseMeters().getTranslation(),gyro.getHeading());
    }
}
