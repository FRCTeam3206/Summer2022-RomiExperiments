package frc.robot.commands;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Characterize extends CommandBase{
    Drivetrain drive;
    public Characterize(Drivetrain drive){
        this.drive=drive;
        addRequirements(drive);
    }
    ArrayList<double[]> values=new ArrayList<double[]>();
    private double percent;
    @Override
    public void initialize(){
        //test(.4);
        //test(.5);
        //test(.6);
        for(double d=.4;d<=1.0;d+=.1){
            test(d);
        }
        for(double[] row:values){
            for(double val :row){
                System.out.print(val+" ");
            }
            System.out.println();
        }
    }
    private void test(double percent){
        this.percent=percent;
        drive.arcadeDrive(percent, 0);
        delay(100);
        evaluate();
        this.percent=0;
        drive.arcadeDrive(0, 0);
        delay(500);
    }
    private void evaluate(){
        double v1=drive.getAveVel();
        delay(100);
        double v2=drive.getAveVel();
        delay(500);
        double v3=drive.getAveVel();
        values.add(new double[]{percent,v3,10*(v2-v1)});
    }
    private void delay(long time){
        long start=System.currentTimeMillis();
        while(start+time>System.currentTimeMillis()){
            drive.arcadeDrive(percent,0);
        }
    }
    public boolean isFinished(){
        return true;
    }
}
