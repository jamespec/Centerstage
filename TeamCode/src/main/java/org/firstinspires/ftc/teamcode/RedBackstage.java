/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.chassis.OmniChassisWithVision;


@Autonomous(name="RedBackstage")
public class RedBackstage extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException {
        OmniChassisWithVision chassis = new OmniChassisWithVision(hardwareMap, telemetry, (OmniChassisWithVision.Marker | OmniChassisWithVision.AprilTag) );
        double heading = chassis.getHeading();

        waitForStart();

        switch (chassis.getLocation() ) {
            case RIGHT:
                telemetry.addLine("RIGHT");
                telemetry.update();
                chassis.moveRobotForward(1.0,26, 0.25);
                chassis.turnRobotToHeading(heading-90, 0.4);
                chassis.moveRobotForward(1.0, 12, 0.25);
                chassis.setArmPosition(6400, 1.0, false);
                chassis.moveToApril(6,13.0,-5, 5);
                chassis.setArmPosition(6400,1.0, true);
                sleep(250);
                chassis.drop();
                sleep(500);
                chassis.setArmPosition(4000, 0.4, true);
                chassis.moveRobotStrafe(0.5,-16);
                chassis.moveRobotForward(0.5,15);
                break;

            case MIDDLE:
                telemetry.addLine("MIDDLE");
                telemetry.update();
                chassis.moveRobotForward(1.0, 40.0, 0.25);
                chassis.setArmPosition(500, 0.3, true);
                chassis.moveRobotForward(0.5, -18);
                chassis.turnRobotToHeading(heading-90, 0.4);
                chassis.moveRobotForward(1.0, 5.0);
                chassis.setArmPosition(6400, 1.0, false);
                chassis.moveToApril(5,13.0,6, 3);
                chassis.setArmPosition(6400, 1.0, true);
                sleep(250);
                chassis.drop();
                sleep(500);
                chassis.setArmPosition(4000, 0.4, true);
                chassis.moveRobotStrafe(0.5,-24);
                chassis.moveRobotForward(0.5, 15);
                break;

            case LEFT:
                telemetry.addLine("LEFT");
                telemetry.update();
                chassis.moveRobotForward(1.0,26, 0.25);
                chassis.turnRobotToHeading(heading-90, 0.4);
                chassis.moveRobotForward(1.0,-11, 0.25);
                chassis.setArmPosition(500, 0.7, true);
                chassis.moveRobotForward(1.0,20);
                chassis.setArmPosition(6400, 0.7, false);
                chassis.moveToApril(4,13.0,5, 4);
                chassis.setArmPosition(6400, 0.7, true);
                sleep(250);
                chassis.drop();
                sleep(500);
                chassis.setArmPosition(4000, 0.4, true);
                chassis.moveRobotStrafe(1.0,-30);
                chassis.moveRobotForward(0.5, 15);
                break;
        }
    }
}
