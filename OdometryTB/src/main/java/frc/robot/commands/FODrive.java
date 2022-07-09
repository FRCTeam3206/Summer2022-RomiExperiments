package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;
import static frc.robot.Constants.*;
public class FODrive extends CommandBase{
    RomiDrivetrain drive;
    RomiGyro gyro;
    Supplier<Double> side;
    Supplier<Double> forward;
    public FODrive(RomiDrivetrain drive, RomiGyro gyro, Supplier<Double> side, Supplier<Double> forward){
        this.drive=drive;
        this.gyro=gyro;
        this.side=side;
        this.forward=forward;
        addRequirements(drive);
    }
    public void execute(){
        double desiredAngle=Math.atan(-forward.get()/side.get())*180/Math.PI;
        SmartDashboard.putNumber("RawDesNum", desiredAngle);
        if(side.get()<0){
            if(-forward.get()>0){
                desiredAngle+=180;
            }else{
                desiredAngle-=180;
            }
        }
        SmartDashboard.putNumber("DesAngle", desiredAngle);
        double currAngle=(double)Math.floorMod((long)gyro.getAngleZ()-90,360l);
        if(currAngle>180){
            currAngle-=360;
        }
        currAngle*=-1;
        SmartDashboard.putNumber("CurrAngle", currAngle);
        double turn=turnToAngle(currAngle, desiredAngle);
        SmartDashboard.putNumber("turn", turn);
        double forwardPower=-Math.sqrt(Math.pow(forward.get(),2)+Math.pow(side.get(),2))*(1-turn);
        drive.arcadeDrive(forwardPower, turn*Math.sqrt(Math.pow(forward.get(),2)+Math.pow(side.get(),2)));
    }
    
}
