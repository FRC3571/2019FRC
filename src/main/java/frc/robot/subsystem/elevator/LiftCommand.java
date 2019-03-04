package frc.robot.subsystem.elevator;


import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class LiftCommand extends Command {

    private boolean up;

    private static int TOP = 100;
    private static int MIDDLE = 50;
    private static int BOTTOM = 25;

    private static double LIFT_SPEED = 0.7;

    private static int targetDistance;

    private static boolean finished = false;


    public LiftCommand(boolean up) {
        requires(Robot.getInstance().getElevator());
        this.up = up;
    }

    @Override
    public void initialize() {
        double currDistance = Math.abs(Robot.getInstance().getElevator().getDistance());
        if(up) {
            if(currDistance >= TOP) {
                finished = true;
            }
            else {
                if(currDistance < MIDDLE) {
                    targetDistance = MIDDLE;
                }
                else if(currDistance < TOP) {
                    targetDistance = TOP;
                }
            }
        }
        else {
            if(currDistance <= BOTTOM) {
                finished = true;
            }
            else {
                if(currDistance > MIDDLE) {
                    targetDistance = MIDDLE;
                }
                else if(currDistance > BOTTOM) {
                    targetDistance = BOTTOM;
                }
            }
        }
    }

    @Override
    protected void execute() {
        double currDistance = Math.abs(Robot.getInstance().getElevator().getDistance());
        if(up) {
            if(currDistance < targetDistance) {
                Robot.getInstance().getElevator().getElevatorMotor().set(LIFT_SPEED);
            }
            else {
                finished = true;
            }
        }
        else {
            if(currDistance > targetDistance) {
                Robot.getInstance().getElevator().getElevatorMotor().set(-LIFT_SPEED);
            }
            else {
                finished = true;
            }
        }

    }

    @Override
    protected boolean isFinished() {
        return finished;
    }
}
