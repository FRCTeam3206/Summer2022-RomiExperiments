package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;
import static frc.robot.Constants.*;
public class TurnCommand extends CommandBase{
    private RomiDrivetrain drive;
    private RomiGyro gyro;
    private double angle;
    public TurnCommand(RomiDrivetrain drive,RomiGyro gyro,double angle){
        this.drive=drive;
        this.gyro=gyro;
        this.angle=angle;
        addRequirements(drive);
    }
    public void execute(){
        double currAngle=(double)Math.floorMod((long)gyro.getAngleZ(),360l);
        if(currAngle>180){
            currAngle-=360;
        }
        currAngle*=-1;
        drive.arcadeDrive(0, turnToAngle(currAngle, angle));
    }
    public void end(){
        drive.arcadeDrive(0, 0);
    }
    public boolean isFinished(){
        return Math.abs(gyro.getProcessedAngle()-angle)<2;
    }
}
