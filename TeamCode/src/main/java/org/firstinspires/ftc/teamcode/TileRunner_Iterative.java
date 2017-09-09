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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
//@Disabled
public class TileRunner_Iterative extends OpMode{
    private double position = 0.0;
    /* Declare OpMode members. */
    private HardwareTileRunnerRobot robot; // use the class created to define a Pushbot's hardware

    public TileRunner_Iterative() {
        robot = new HardwareTileRunnerRobot();
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
        double left_stick, right_stick, power, left_trigger, right_trigger;
        boolean left_bumper, right_bumper, a_button, b_button, x_button, y_button;






        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left_stick = -gamepad1.left_stick_y;
        right_stick = -gamepad1.right_stick_y;
        left_bumper = gamepad1.left_bumper;
        right_bumper = gamepad1.right_bumper;
        left_trigger = gamepad1.left_trigger;
        right_trigger = gamepad1.right_trigger;
        a_button = gamepad1.a;
        b_button = gamepad1.b;
        x_button = gamepad1.x;
        y_button = gamepad1.y;



        power = 1;
        if(left_trigger > 0.5 ) {

            left_stick = left_stick* -1;
            power = 0.5;
        }


        if(right_trigger > 0.7) {

                right_stick = right_stick* -1;
                power = 0.7;
        }
        if (right_bumper == true){
            position++;

        }
        if  (left_bumper == true) {
            position--;
        }

        robot.servo_one.setPosition(position);
        robot.setDrivePower(left_stick*power,right_stick*power);

        // Send telemetry message to signify robot running;

        telemetry.addLine("Controller One Telemetry:");
        telemetry.addData("left stick: ",  "%.2f", left_stick);
        telemetry.addData("right stick: ", "%.2f", right_stick);
        telemetry.addData("Left Bumper: ", left_bumper);
        telemetry.addData("Right Bumper: ", right_bumper );
        telemetry.addData("Left Trigger: ", left_trigger);
        telemetry.addData("Right Trigger: ", right_trigger);
        telemetry.addData("A Button: ",a_button);
        telemetry.addData("B Button: ",b_button);
        telemetry.addData("X Button: ",x_button);
        telemetry.addData("Y Button: ", y_button);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}