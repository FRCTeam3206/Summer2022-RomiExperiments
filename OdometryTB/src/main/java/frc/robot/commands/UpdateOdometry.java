package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RomiOdometer;

public class UpdateOdometry extends CommandBase{
    RomiOdometer od;
    public UpdateOdometry(RomiOdometer od){
        this.od=od;
    }
    @Override
    public void initialize() {
    }
    @Override
    public void execute(){
        od.update();
    }
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
