package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 *
 * Created by Veggie on 9/17/2017.
 *
 */
//@TeleOp(name="Demo: Flex Sensor", group="Iterative Opmode")
public class FlexSensor{

    private double final TOLERANCE = 0.25;

    private double flexCurrent;
    private double flexPrevious;
    private int columnNum = 0;
    private AnalogInput voltageIn;


    public FlexSensor() {
        voltageIn = hardwareMap.analogInput.get("flx");
        telemetry.addData("Flex Sensor Status:", "Initialized");
    }
/* used to test the class
    @Override
    public void loop(){
        if(gamepad1.dpad_up){
            tolerance += 0.001;
        }else if(gamepad1.dpad_down){
            tolerance -= 0.001;
        }
        if(gamepad1.b){
            columnNum = 0;
        }
        /*   Left ovever Debug Stuff
        telemetry.addData("Tolerance: ", getTolerance());
        telemetry.addData("Flex Value: ", getFlex());
        telemetry.addData("Column Num: ", testColumnNum());
        */


    /**
     * Tests the amount of columns detected by the flex sensor.
     * @return the number of columns passed by the robot
     */
    public int getColumnNum(){
        flexCurrent = voltageIn.getVoltage();

        if (flexPrevious - TOLERANCE > flexCurrent) {
            columnNum++;
        }
        flexPrevious = flexCurrent;
        return columnNum;
    }

    public double getFlex() {
        flexCurrent = voltageIn.getVoltage();
        return flexCurrent;
    }
    public double getTolerance() {
        return TOLERANCE;
    }
}

