package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnAngle extends CommandBase{
    private Drivetrain drivetrain;
    private double angle;
    private double sign;
    public TurnAngle(double angle, Drivetrain drivetrain){
        addRequirements(drivetrain);
        this.drivetrain=drivetrain;
        this.angle=Math.abs(angle);
        this.sign=Math.signum(angle);
    }
    @Override
    public void initialize() {
        drivetrain.resetEncoders();
        drivetrain.resetGyro();
        timerStart=System.currentTimeMillis();
    }
    @Override
    public void execute() {
        double currAngle=drivetrain.getGyroAngleZ();
        double angleDeficit=angle*sign-currAngle;
        drivetrain.arcadeDrive(0,angleDeficit/60);
    }
    long timerStart;
    boolean timerRunning=false;
    @Override
    public boolean isFinished() {
        double currAngle=drivetrain.getGyroAngleZ();
        double angleDeficit=angle*sign-currAngle;
        if(angleDeficit<3&&!timerRunning){
            //timerStart=System.currentTimeMillis();
            timerRunning=true;
        }
        if(System.currentTimeMillis()-timerStart<2){
            timerRunning=false;
        }
        return timerStart+1500<=System.currentTimeMillis();
    }
}
