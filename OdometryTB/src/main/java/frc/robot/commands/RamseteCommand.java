package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RomiOdometer;
import frc.robot.subsystems.RomiDrivetrain;

public class RamseteCommand extends CommandBase {
    private RomiDrivetrain drive;
    private RomiOdometer od;
    private RamseteController rc;
    private Pose2d goal;
    public RamseteCommand(RomiDrivetrain drive,RomiOdometer od,Pose2d goal){
        this.drive=drive;
        this.od=od;
        this.goal=goal;
        addRequirements(drive);
        rc=new RamseteController(2.0/300,.7);
        rc.setTolerance(new Pose2d(5.0,5.0,new Rotation2d(Math.PI)));
    }
    private DifferentialDriveKinematics kin=new DifferentialDriveKinematics(14.7);
    public void execute(){
        Pose2d trans=od.getPose().relativeTo(goal);
        double dist=Math.sqrt(Math.pow(trans.getX(),2)+Math.pow(trans.getY(),2));
        double angleOff=trans.getRotation().getRadians();

        double angleSpeed=Math.abs(angleOff%2*Math.PI/Math.PI)*.5;
        SmartDashboard.putNumber("RawAngleSpeed", angleSpeed);
        if(dist<7){
            angleSpeed*=dist/5;
        }
        double speed=1-angleSpeed;
        speed*=.7;
        if(dist<20){
            speed*=dist/20;
        }
        double goalAngle=Math.atan((goal.getX()-od.getX())/(goal.getY()-od.getY()));
        if(dist<10){
            goalAngle=od.getPose().getRotation().getRadians();
        }
        Pose2d updatedGoal=new Pose2d(goal.getX(), goal.getY(), new Rotation2d(goalAngle));
        ChassisSpeeds speeds=rc.calculate(od.getPose(), updatedGoal, speed, angleSpeed);
        DifferentialDriveWheelSpeeds wheelSpeeds = kin.toWheelSpeeds(speeds);
        double left=wheelSpeeds.leftMetersPerSecond;
        double right=wheelSpeeds.rightMetersPerSecond;
        SmartDashboard.putNumber("LeftWheel", left);
        SmartDashboard.putNumber("RightWheel", right);
        double div=Math.max(left,right);
        if(div>1){
            right/=div;
            left/=div;
            right*=.7;
            left*=.7;
        }
        drive.tankDrive(left,right);
    }
    @Override
    public boolean isFinished(){
        return rc.atReference();
    }
}
