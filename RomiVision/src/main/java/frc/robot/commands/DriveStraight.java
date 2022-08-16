package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RomiGyro;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends CommandBase{
    private Drivetrain drivetrain;
    private double distance;
    public DriveStraight(double distance,Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain=drivetrain;
        this.distance=distance;
    }
    @Override
    public void initialize() {
        drivetrain.resetEncoders();
        drivetrain.resetGyro();
    }
    @Override
    public void execute() {
        double angle=drivetrain.getGyroAngleZ();
        double speed=0.7;
        double currDistance=drivetrain.getAverageDistanceInch();
        SmartDashboard.putNumber("Curr distance",currDistance);
        if(currDistance<distance){
            drivetrain.arcadeDrive(speed,-angle/60);
        }
    }
    @Override
    public boolean isFinished() {
        return drivetrain.getAverageDistanceInch()>distance;
    }
}
