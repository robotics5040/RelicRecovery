/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Omnibot Pushbot", group="Pushbot")
//@Disabled
public class OmniBot_Iterative extends OpMode{
    private double position = 0.0;

    public int  pressed = 0;
    /* Declare OpMode members. */
    private HardwareOmniRobot robot; // use the class created to define a Pushbot's hardware

    public OmniBot_Iterative() {
        robot = new HardwareOmniRobot();
    }

    // could also use HardwarePushbotMatrix class.
   /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left_stick_x, left_stick_y,right_stick_x, right_stick_y, power, left_trigger, right_trigger,LX,RX;
        boolean left_bumper, right_bumper, a_button, b_button, x_button, y_button,dup,ddown,dleft,dright,left_bump1,right_bump1, d_up1,d_down1,d_left1,d_right1;






        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left_stick_x = gamepad1.left_stick_x;
        left_stick_y = gamepad1.left_stick_y;
        right_stick_x = gamepad1.right_stick_x;
        right_stick_y = gamepad1.right_stick_y;

        left_bumper = gamepad2.left_bumper;
        right_bumper = gamepad2.right_bumper;
        left_trigger = gamepad1.left_trigger;
        right_trigger = gamepad1.right_trigger;
        a_button = gamepad1.a;
        b_button = gamepad1.b;
        x_button = gamepad2.x;
        y_button = gamepad2.y;
        //b_button2 = gamepad2.b;
        LX = gamepad2.left_stick_y;
        RX = gamepad2.right_stick_y;
        dup = gamepad2.dpad_up;
        ddown = gamepad2.dpad_down;
        dleft = gamepad2.dpad_left;
        dright = gamepad2.dpad_right;
        left_bump1 = gamepad1.left_bumper;
        right_bump1 = gamepad1.right_bumper;
        d_down1 = gamepad1.dpad_down;
        d_up1 = gamepad1.dpad_up;
        d_left1 = gamepad1.dpad_left;
        d_right1 = gamepad1.dpad_right;


        robot.grabber.setPower(1);

        //slight adjustments for driver
        if(d_down1 == true) {
            left_stick_y = 0.4;
        }
        if(d_up1 == true) {
            left_stick_y = -0.4;
        }
        if(d_left1 == true) {
            left_stick_x = -0.4;
        }
        if(d_right1 == true) {
            left_stick_x = 0.4;
        }


        robot.onmiDrive(left_stick_x, left_stick_y, right_stick_x);


        if (left_bumper == true) {
            robot.grabber.setTargetPosition(1500);
        }
        else {
            robot.grabber.setTargetPosition(0);
        }


        if(left_bump1 == true) {
            robot.wheelie.setPower(-1.0);
        }
        else if(right_bump1 == true){
            robot.wheelie.setPower(1.0);
        }
        else {
            robot.wheelie.setPower(0.0);
        }


        if (right_bumper == true) {
            robot.dumper.setPosition(.5);
        }
        else {
            robot.dumper.setPosition(0);
        }


        if ((x_button == true)&& (left_bumper == false) ) {
            robot.claw1.setPosition(1);
            robot.claw2.setPosition(.0);
        }
        else if (x_button == true) {
            robot.claw1.setPosition(0.70);
            robot.claw2.setPosition(0.30);
        }
        else if(y_button == true) {
            robot.claw1.setPosition(0.60);
            robot.claw2.setPosition(0.35);
        }
        else {
            robot.claw1.setPosition(0.35);
            robot.claw2.setPosition(0.5);
        }


        /*if (b_button2 == true) {
            robot.grabber.setPower(-0.1);

        }
        else {
            robot.grabber.setPower(0);
        }*/



        //robot.JKnock.setPosition(0.9);

        //robot.relicLifter(dup,ddown,dleft,dright);

        //robot.grabber.setPosition(0.0);
        // Send telemetry message to signify robot running;

        telemetry.addLine("Controller Telemetry:");
        telemetry.addData("Left Bumper: ", left_bumper);
        telemetry.addData("Right Bumper: ", right_bumper );
        telemetry.addData("Left Trigger: ", left_trigger);
        telemetry.addData("Right Trigger: ", right_trigger);
        telemetry.addData("A Button: ",a_button);
        telemetry.addData("B Button: ",b_button);
        telemetry.addData("X Button: ",x_button);
        telemetry.addData("Y Button: ", y_button);
        telemetry.addData("2nd Left Trigger",LX);
        telemetry.addData("2nd Right Trigger",RX);
        telemetry.addData("Color Sensor Blue", robot.jkcolor.blue());
        telemetry.addData("Color Sensor Red", robot.jkcolor.red());
        telemetry.addData("Color Sensor Green", robot.jkcolor.green());
        telemetry.addData("Color Sensor Alpha", robot.jkcolor.alpha());
        telemetry.addData("Color Sensor ARGB", robot.jkcolor.argb());
        telemetry.addData("Color Sensor Hashcode", robot.jkcolor.hashCode());
        telemetry.addLine("What is my name?: 474675627377");

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}