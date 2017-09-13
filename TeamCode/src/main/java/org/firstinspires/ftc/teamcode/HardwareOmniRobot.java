package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Tile Runner.  The Tile Runner is setup with two motors
 * on each side.  This version assumes each motor is connected to an individual port.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor1:        "left_motor1"
 * Motor channel:  Left  drive motor2:        "left_motor2"
 * Motor channel:  Right drive motor1:        "right_motor1"
 * Motor channel:  Right drive motor2:        "right_motor2"
 */
public class HardwareOmniRobot
{
    /* Public OpMode members. */
    public DcMotor leftMotor1 = null;
    public DcMotor leftMotor2 = null;
    public DcMotor rightMotor1 = null;
    public DcMotor rightMotor2 = null;


    public static final String MESSAGETAG = "5040MSG";
    private final int NAVX_DIM_I2C_PORT = 0;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    public AHRS navx_device;

    /* local OpMode members. */
    HardwareMap hwMap;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareOmniRobot(){

        hwMap = null;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        try {
            leftMotor1 = hwMap.dcMotor.get("left_motor1");
            leftMotor2 = hwMap.dcMotor.get("left_motor2");
            rightMotor1 = hwMap.dcMotor.get("right_motor1");
            rightMotor2 = hwMap.dcMotor.get("right_motor2");

            leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

            // Set all motors to zero power
            leftMotor1.setPower(0);
            rightMotor1.setPower(0);
            leftMotor2.setPower(0);
            rightMotor2.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Enable NavX Sensor

            navx_device = AHRS.getInstance(hwMap.deviceInterfaceModule.get("DIM1"),
                    NAVX_DIM_I2C_PORT,
                    AHRS.DeviceDataType.kProcessedData,
                    NAVX_DEVICE_UPDATE_RATE_HZ);
        } catch (Exception e) {

            RobotLog.ee(MESSAGETAG,e.getMessage());

        }
    }

    public void setDrivePower(double leftMotors, double rightMotors){

        leftMotor1.setPower(leftMotors);
        rightMotor1.setPower(rightMotors);
        leftMotor2.setPower(leftMotors);
        rightMotor2.setPower(rightMotors);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    /***
     * onmiDrive is for the teleop drive part of the competition
     * Takes input of the joysticks

     */


    public void onmiDrive (double rx, double ry, double lx, double ly)
    {

        rx *= -1;
        ry *= -1;
        //rx *= .75;
        //ry *= -.75;
        //lx *= .75;
        //ly *= .75;
        leftMotor1.setPower(((-ry - rx)/2) * 1 + (-.75 * lx));
        leftMotor2.setPower(((-ry + rx)/2) * 1 + (-.75 * lx));
        rightMotor1.setPower(((ry - rx)/2) * 1 + (-.75 * lx));
        rightMotor2.setPower(((ry + rx)/2) * 1 + (-.75 * lx));


           /* leftMotor1.setPower(.5);
            leftMotor2.setPower(.5);
            rightMotor1.setPower(.5);
            rightMotor2.setPower(.5);*/



    }
}
