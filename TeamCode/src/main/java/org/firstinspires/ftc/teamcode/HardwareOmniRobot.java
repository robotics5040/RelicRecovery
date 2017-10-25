package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

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
    ElapsedTime runtime = new ElapsedTime();

    ColorSensor jkcolor;
    OpticalDistanceSensor odsfront;
    OpticalDistanceSensor odsback;
    OpticalDistanceSensor odsleft1;
    OpticalDistanceSensor odsleft2;

    /* Public OpMode members. */
    public DcMotor leftMotor1 = null;
    public DcMotor leftMotor2 = null;
    public DcMotor rightMotor1 = null;
    public DcMotor rightMotor2 = null;
    public DcMotor wheelie = null;
    public DcMotor grabber = null;
    public Servo jknock = null;
    public Servo dumper = null;
    // public Servo grabber = null;
    public Servo claw1 = null;
    public Servo claw2 = null;
    public DcMotor reel = null;
    public DcMotor slide = null;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;

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
        //try {
        leftMotor1 = hwMap.dcMotor.get("left_motor1");
        leftMotor2 = hwMap.dcMotor.get("left_motor2");
        rightMotor1 = hwMap.dcMotor.get("right_motor1");
        rightMotor2 = hwMap.dcMotor.get("right_motor2");
        wheelie = hwMap.dcMotor.get("wheelie");
        grabber = hwMap.dcMotor.get("grabber");
        slide = hwMap.dcMotor.get("slide");
        reel = hwMap.dcMotor.get("reel");
        dumper = hwMap.servo.get("dumper");
        claw1 = hwMap.servo.get("claw_1");
        //grabber = hwMap.servo.get("grabber");
        claw2 = hwMap.servo.get("claw_2");
        jknock = hwMap.servo.get("jknock");
        jkcolor = hwMap.colorSensor.get("color_sense");
        odsfront = hwMap.opticalDistanceSensor.get("ODSFront");
        odsback = hwMap.opticalDistanceSensor.get("ODSBack");
        odsleft1 = hwMap.opticalDistanceSensor.get("ODSLeft2");
        odsleft2 = hwMap.opticalDistanceSensor.get("ODSLeft1");

        reel.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.REVERSE);
        leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //grabber.setDirection(Servo.Direction.REVERSE);
        grabber.setDirection(DcMotor.Direction.REVERSE);
        grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set all motors to zero power
            leftMotor1.setPower(0);
            rightMotor1.setPower(0);
            leftMotor2.setPower(0);
            rightMotor2.setPower(0);
            wheelie.setPower(0);
            slide.setPower(0);
            reel.setPower(0);
            jknock.setPosition(1.2);
            claw1.setPosition(.75);
            claw2.setPosition(.25);
            //grabber.scaleRange(0.0, 0.25);
            //grabber.setPosition(0.220);
            
            grabber.setPower(0.75);
            grabber.setTargetPosition(1430);

        dumper.setPosition(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //reel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Enable NavX Sensor

        navx_device = AHRS.getInstance(hwMap.deviceInterfaceModule.get("DIM"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
       /* } catch (Exception e) {

            RobotLog.ee(MESSAGETAG,e.getMessage());

        }*/
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

    //rx *= -1;
    //ry *= -1;
    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    public void relicLifter(boolean dup, boolean ddown, boolean dleft, boolean dright) {
        if ((dup == true) && (ddown == false)) {
            slide.setPower(30);
        }
        else if ((dup == false)&&(ddown == true)) {
            slide.setPower(-30);

        }
        else {
            slide.setPower(0);
        }

        if ((dleft == true)&&(dright == false)) {
            reel.setPower(30);
        }
        else  if ((dleft == false)&&(dright== true)) {
            reel.setPower(-30);
        }
        else {

            reel.setPower(0);
        }

    }


    public void onmiDrive (double sideways, double forward, double rotation)
    {


        try {
            leftMotor1.setPower(limit(((forward - sideways)/2) * 1 + (-.314 * rotation)));
            leftMotor2.setPower(limit(((forward + sideways)/2) * 1 + (-.314 * rotation)));
            rightMotor1.setPower(limit(((-forward - sideways)/2) * 1 + (-.314 * rotation)));
            rightMotor2.setPower(limit(((-forward + sideways)/2) * 1 + (-.314 * rotation)));
        } catch (Exception e) {
            RobotLog.ee(MESSAGETAG, e.getStackTrace().toString());
        }


    }

    public void DriveFor(double time, double forward, double rotate) {

        onmiDrive(0.0,-forward,rotate); //starts moving in wanted direction
        runtime.reset(); //resets time

        while ((runtime.seconds() < time)) {    //runs for amount of time wanted
        }
        onmiDrive(0.0, 0.0, 0.0); //stops  moving after
    }

    public void JewelKnock() {

        boolean decided = false;
        runtime.reset();
        jkcolor.enableLed(true);
        jknock.setPosition(60);

        while(decided == false) {
            int color = jkcolor.alpha();
            if(color == 10) {
                DriveFor(0.5, 0.0, 1.0);
                jknock.setPosition(1.2);
                DriveFor(0.5, 0.0, -1.0);
                decided = true;
            }
            else if(color == 50) {
                DriveFor(0.5, 0.0, -1.0);
                jknock.setPosition(1.2);
                DriveFor(0.5, 0.0, 1.0);
                decided = true;
            }
            else {
                if(runtime.seconds() >= 1) {
                    jknock.setPosition(1.2);
                    decided = true;
                }
            }

        }
        jkcolor.enableLed(false);
    }

    public int Vuforia(int cameraMonitorViewId) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AUBrQCz/////AAAAGXg5njs2FEpBgEGX/o6QppZq8c+tG+wbAB+cjpPcC5bwtGmv+kD1lqGbNrlHctdvrdmTJ9Fm1OseZYM15VBaiF++ICnjCSY/IHPhjGW9TXDMAOv/Pdz/T5H86PduPVVKvdGiQ/gpE8v6HePezWRRWG6CTA21itPZfj0xDuHdqrAGGiIQXcUbCTfRAkY7HwwRfQOM1aDhmeAaOvkPPCnaA228iposAByBHmA2rkx4/SmTtN82rtOoRn3/I1PA9RxMiWHWlU67yMQW4ExpTe2eRtq7fPGCCjFeXqOl57au/rZySASURemt7pwbprumwoyqYLgK9eJ6hC2UqkJO5GFzTi3XiDNOYcaFOkP71P5NE/BB    ";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        int choosen = 0;

        while (choosen == 0) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                switch(vuMark) {
                    case LEFT:
                        choosen = 1;
                        break;
                    case CENTER:
                        choosen = 2;
                        break;
                    case RIGHT:
                        choosen = 3;
                        break;
                }
            }
        }
        return choosen;
    }

}
