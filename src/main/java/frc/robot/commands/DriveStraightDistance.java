package frc.robot.commands;

import frc.robot.Robot;
//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to run a simple PID loop that is only enabled while this
 * command is running. The input is the averaged values of the left and right
 * encoders.
 */
public class DriveStraightDistance extends Command {
    private double speed;
    private double targetDistance;

    public DriveStraightDistance(double distance) {
        requires(Robot.getInstance().getDrive());
        this.targetDistance = distance;
    }

    public DriveStraightDistance(double distance, double speed) {
        this(distance);
        this.speed = speed;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        // Get everything in a safe starting state.
        Robot.getInstance().getDrive().reset();
        //m_pid.reset();
        //m_pid.enable();
        //this.speed = SmartDashboard.getNumber("AutoSpeed", 0.75);
        //drive
        Robot.getInstance().getDrive().drive(speed, speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        System.out.println("Im checking!");
        double distance = Math.abs(Robot.getInstance().getDrive().getDistance());
        if(distance>=targetDistance) {
            System.out.println("done!");
            return true;
        }
        return false;

        /*m_pid.onTarget()*/
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        // Stop PID and the wheels
        //m_pid.disable();
        Robot.getInstance().getDrive().reset();
        Robot.getInstance().getDrive().drive(0, 0);
    }
}

