package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.sensors.RomiOdometer;
import frc.robot.subsystems.RomiDrivetrain;

public class PathFollower extends CommandBase{
    private RomiDrivetrain drive;
    private RomiOdometer od;
    private Pose2d goal;
    private boolean slowToTarget=true;
    public PathFollower(RomiDrivetrain drive,RomiOdometer od,Pose2d goal,boolean slowToTarget){
        this.drive=drive;
        this.od=od;
        this.goal=goal;
        addRequirements(drive);
        this.slowToTarget=slowToTarget;
    }
    public PathFollower(RomiDrivetrain drive,RomiOdometer od,Pose2d goal){
        this.drive=drive;
        this.od=od;
        this.goal=goal;
        addRequirements(drive);
    }
    public void execute(){
        SmartDashboard.putString("Going to",""+goal.toString());
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
        if(angleError>180){
            angleError-=360;
        }
        if(angleError<-180){
            angleError+=360;
        }
        SmartDashboard.putNumber("Angle",currAngle);
        SmartDashboard.putNumber("Angle Error", angleError);
        double angleSpeed=1-Math.pow((1-Math.abs(angleError/180)),4);
        double forwardSpeed=Math.pow((1-Math.abs(angleError/180)),6);
        double distance=Math.sqrt(Math.pow(error.getX(),2)+Math.pow(error.getY(),2));
        SmartDashboard.putNumber("Distance", distance);
        if(slowToTarget){
            forwardSpeed*=1-Math.pow(1.3,-distance);
        }
        //forwardSpeed*=.8;
        angleSpeed*=Math.signum(angleError);
        drive.arcadeDrive(-forwardSpeed, angleSpeed);
    }
    public boolean isFinished(){
        Pose2d error=goal.relativeTo(od.getPose());
        double distance=Math.sqrt(Math.pow(error.getX(),2)+Math.pow(error.getY(),2));
        return distance<3;
    }
    public static Command followPath(RomiDrivetrain drive,RomiOdometer od,Translation2d[] points){
        Command[] commands=new Command[points.length];
        int i=0;
        for(Translation2d trans:points){
            boolean slow=false;
            if(i+1==points.length){
                slow=true;
            }
            commands[i]=new PathFollower(drive, od, new Pose2d(trans.getX(),trans.getY(),new Rotation2d()),slow);
            i++;
        }
        return new SequentialCommandGroup(commands);
    }
}
