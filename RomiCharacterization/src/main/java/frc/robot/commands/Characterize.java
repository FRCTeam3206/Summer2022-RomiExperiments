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
    private double volatage;
    @Override
    public void initialize(){
        //test(.4);
        //test(.5);
        //test(.6);
        for(double d=5;d<=8;d+=.5){
            test(d);
        }
        for(double[] row:values){
            for(double val :row){
                System.out.print(val+" ");
            }
            System.out.println();
        }
    }
    private void test(double volatage){
        this.volatage=volatage;
        drive.drive(volatage);
        delay(100);
        evaluate();
        this.volatage=0;
        drive.drive(0);
        delay(500);
    }
    private void evaluate(){
        double v1=drive.getAveVel();
        delay(100);
        double v2=drive.getAveVel();
        delay(500);
        double v3=drive.getAveVel();
        values.add(new double[]{volatage,v3,10*(v2-v1)});
    }
    private void delay(long time){
        long start=System.currentTimeMillis();
        while(start+time>System.currentTimeMillis()){
            drive.drive(volatage);
        }
    }
    public boolean isFinished(){
        return true;
    }
}
