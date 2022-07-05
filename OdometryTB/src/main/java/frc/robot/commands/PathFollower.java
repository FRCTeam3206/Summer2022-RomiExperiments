package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RomiOdometer;
import frc.robot.subsystems.RomiDrivetrain;

public class PathFollower extends CommandBase{
    private RomiDrivetrain drive;
    private RomiOdometer od;
    private Pose2d goal;
    public PathFollower(RomiDrivetrain drive,RomiOdometer od,Pose2d goal){
        this.drive=drive;
        this.od=od;
        this.goal=goal;
        addRequirements(drive);
    }
    public void execute(){
        Pose2d current=new Pose2d(od.getX(),od.getY(),new Rotation2d());
        Pose2d error=goal.relativeTo(current);
        double angleTo=Math.atan(error.getY()/error.getX())*180/Math.PI;
        if(Math.signum(error.getX())!=Math.signum(error.getY())){
            angleTo+=180;
        }
        if(Math.abs(error.getX())<.001){
            angleTo=Math.PI*3/2;
        }
        SmartDashboard.putNumber("Angle To Target", angleTo);
        double currAngle=(double)Math.floorMod((long)od.getPose().getRotation().getDegrees(),360l);
        if(error.getY()>0){
            currAngle-=180;
        }
        double angleError=angleTo-currAngle;
        SmartDashboard.putNumber("Angle",currAngle);
        SmartDashboard.putNumber("Angle Error", angleError);
        double angleSpeed=1-Math.pow((1-Math.abs(angleError/180)),5);
        double forwardSpeed=1-angleSpeed;
        double distance=Math.sqrt(Math.pow(error.getX(),2)+Math.pow(error.getY(),2));
        SmartDashboard.putNumber("Distance", distance);
        forwardSpeed*=1-Math.pow(1.5,-distance);
        forwardSpeed*=.8;
        angleSpeed*=Math.signum(angleError);
        drive.arcadeDrive(-forwardSpeed, angleSpeed);
    }
    public boolean isFinished(){
        Pose2d error=goal.relativeTo(od.getPose());
        double distance=Math.sqrt(Math.pow(error.getX(),2)+Math.pow(error.getY(),2));
        return distance<2;
    }
}
