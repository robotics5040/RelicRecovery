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

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Omnibot: Blue1Simple", group="Omnibot")
//@Disabled
public class Blue1Simple extends LinearOpMode {

    HardwareOmniRobot  robot   = new HardwareOmniRobot();
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.JewelKnock("blue");
        robot.DriveFor(0.3,0.0,0.0,0.0);
        robot.wheelie.setPower(1.0);
        robot.DriveFor(1.2,1.0,0.0,0.0);
        robot.wheelie.setPower(0.0);
        robot.DriveFor(0.3,0.0,0.0,0.0);
        robot.grabber.setTargetPosition(0);
        robot.claw1.setPosition(0.3);
        robot.claw2.setPosition(0.6);

        boolean dis = false;
        while(dis == false && runtime.seconds() < 26) {
            double distanceLeft = robot.ultra_left.getDistance(DistanceUnit.CM);

            telemetry.addData("Left", distanceLeft);
            telemetry.update();

            if(distanceLeft >= 14 && distanceLeft <= 16) {
                telemetry.addData("Done", distanceLeft);
                telemetry.update();
                RobotLog.ii("5040MSG","Done",distanceLeft);

                robot.onmiDrive(0.0,0.0,0.0);
                dis = true;
            }
            else if(distanceLeft < 14) {
                telemetry.addData("Towards", distanceLeft);
                telemetry.update();
                robot.onmiDrive(0.4,0.0,0.0);
            }
            else {
                telemetry.addData("Away", distanceLeft);
                telemetry.update();
                robot.onmiDrive(-0.4,0.0,0.0);
            }
        }
        while(runtime.seconds() < 28) {robot.jknock.setPosition(0.59);}
        robot.navx_device.close();
    }
}