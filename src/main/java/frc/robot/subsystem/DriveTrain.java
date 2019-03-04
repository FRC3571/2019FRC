package frc.robot.subsystem;

import frc.robot.commands.TankDriveCommand;
import frc.robot.util.Loggable;
import frc.robot.util.RobotMath;
import frc.robot.util.XboxController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
//import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends PIDSubsystem implements Loggable, Refreshable {

    //motor ports
    private static int LEFT_MOTOR_GROUP, RIGHT_MOTOR_GROUP;
    //encoder ports/channels
    private static int FRONT_LEFT_ENCODER_CHANNEL_A,
    FRONT_LEFT_ENCODER_CHANNEL_B,
    FRONT_RIGHT_ENCODER_CHANNEL_A,
    FRONT_RIGHT_ENCODER_CHANNEL_B;

    private static boolean FORWARD_DIRECTION, REVERSE_DIRECTION;
    private static CounterBase.EncodingType ENCODER_TYPE;

    //encoder mapping
    private static int COUNTS_PER_REVOLUTION;
    private static double WHEEL_RADIUS;

    //controller port
    private static int CONTROLLER_PORT;

    //gear ratios
    public static final double GEAR_RATIO_LOW;
    public static final double GEAR_RATIO_HIGH;

    static {
        //initialization
        LEFT_MOTOR_GROUP = 0;
        RIGHT_MOTOR_GROUP = 1;

        FRONT_LEFT_ENCODER_CHANNEL_A = 2;
        FRONT_LEFT_ENCODER_CHANNEL_B = 3;
        FRONT_RIGHT_ENCODER_CHANNEL_A = 4;
        FRONT_RIGHT_ENCODER_CHANNEL_B = 5;

        FORWARD_DIRECTION = false;
        REVERSE_DIRECTION = true;
        ENCODER_TYPE = CounterBase.EncodingType.k1X;
        COUNTS_PER_REVOLUTION = 2048;
        WHEEL_RADIUS = 62.5; //in mm

        CONTROLLER_PORT = 0;

        GEAR_RATIO_LOW = 4.6;
        GEAR_RATIO_HIGH = 2.7;
    }


    //left
    private Spark left;
    //right
    private Spark right;
    //underlying mechanism
    private DifferentialDrive drive;
    //distance encoders
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    //driver controller
    private XboxController controller;

    //private float error;


    private float lastSpeed;

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new TankDriveCommand(controller));
    }



    public DriveTrain() {
        super("DriveTrain", 2.0, 0,0);

        //initialize hardware
        right = new Spark(RIGHT_MOTOR_GROUP);
        left = new Spark(LEFT_MOTOR_GROUP);

        drive = new DifferentialDrive(right, left);

        initializeEncoders();

        left.setInverted(false);
        right.setInverted(false);

        drive(0, 0);
        drive.setSafetyEnabled(false);

        controller = new XboxController(CONTROLLER_PORT);
    }

    /**
     * The log method puts interesting information to the SmartDashboard.
     */
    @Override
    public void log() {
        SmartDashboard.putString("distance", String.valueOf(getDistance()));
    }

    /**
     * Tank style driving for the DriveTrain.
     *
     * @param left
     *            Speed in range [-1,1]
     * @param right
     *            Speed in range [-1,1]
     */
    public void drive(double left, double right) {
        lastSpeed = (float) right;
        //drive.tankDrive(left, right); //tank drive
        drive.arcadeDrive(left,right);  //arcade drive
    }

    /**
     * Tank style driving for the DriveTrain.
     *
     * @param xbox The XboxController use to drive tank style.
     */
    public void drive(XboxController xbox) {
        //drive(-xbox.getY(GenericHID.Hand.kLeft), xbox.getY(GenericHID.Hand.kRight));
        //drive(-xbox.LeftStick.Y, -xbox.RightStick.Y); //tank drive
        drive(xbox.RightStick.Y, xbox.RightStick.X); //arcade drive
    }

    /**
     * Reset the robots sensors to the zero states.
     */
    public void reset() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    /**
     * Get the average distance of the encoders since the last reset.
     *
     * @return The distance driven (average of leftControllerand rightControllerencoders).
     */
    public double getDistance() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance())/2;
    }

    //will be replace by gyro!

    public double getTurnDistance() {
        return (Math.abs(leftEncoder.getDistance()) + Math.abs(rightEncoder.getDistance()))/2;
    }

    private void initializeEncoders() {
        leftEncoder = new Encoder(FRONT_LEFT_ENCODER_CHANNEL_A,
                FRONT_LEFT_ENCODER_CHANNEL_B,
                REVERSE_DIRECTION,
                ENCODER_TYPE);

        rightEncoder = new Encoder(FRONT_RIGHT_ENCODER_CHANNEL_A,
                FRONT_RIGHT_ENCODER_CHANNEL_B,
                FORWARD_DIRECTION,
                ENCODER_TYPE);

        final double encoderLinearDistancePerPulse = RobotMath.getDistancePerPulse(COUNTS_PER_REVOLUTION, WHEEL_RADIUS);

        leftEncoder.setDistancePerPulse(encoderLinearDistancePerPulse);
        rightEncoder.setDistancePerPulse(encoderLinearDistancePerPulse);
    }

    @Override
    public void refresh() {
        controller.refresh();
        drive(controller);
    }

    @Override
    protected double returnPIDInput() {
        return (rightEncoder.getDistance() - leftEncoder.getDistance());
    }

    @Override
    protected void usePIDOutput(double output) {
        if(output > 0) {
            //too much
            lastSpeed += 0.01;
            drive.tankDrive(lastSpeed, lastSpeed);

        }
        else if(output < 0) {
            lastSpeed -= 0.01;
            drive.tankDrive(lastSpeed, lastSpeed);
            //too little
        }
        //debug output
        System.out.println("OUTPUT -> " + output);
    }
}
