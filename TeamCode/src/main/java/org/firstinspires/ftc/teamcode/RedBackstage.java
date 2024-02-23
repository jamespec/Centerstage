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
                sleep(1000);
                chassis.moveRobotForward(0.5, 26);
                chassis.turnRobotToHeading(heading-90, 0.4);
                chassis.moveRobotForward(0.5, 18);
                chassis.setArmPosition(6200, 0.7, false);
                chassis.moveToApril(6,12,0);
                chassis.setArmPosition(6200, 0.7, true);
                chassis.drop();
                chassis.setArmPosition(5000, 0.7, true);
                break;


            case MIDDLE:
                telemetry.addLine("MIDDLE");
                telemetry.update();
                sleep(1000);
                chassis.moveRobotForward(0.5, 40);
                chassis.setArmPosition(500, 0.3, true);
                chassis.moveRobotForward(0.5, -17);
                chassis.turnRobotToHeading(heading-90, 0.4);
                chassis.setArmPosition(6300, 1.0, false);
                chassis.moveRobotForward(0.5, 20);
                chassis.moveToApril(5,13.5,5, 7);
                chassis.setArmPosition(6300, 1.0, true);
                chassis.drop();
                sleep(200);
                chassis.setArmPosition(5000, 0.2, true);
                break;

            case LEFT:
                chassis.moveRobotForward(0.5, 26);
                chassis.turnRobotToHeading(heading-90, 0.4);
                chassis.moveRobotForward(0.5, -3);
                chassis.setArmPosition(500, 0.3, true);
                chassis.moveRobotForward(0.5, 22);
                chassis.setArmPosition(6300, 1.0, false);
                chassis.moveToApril(4,13.5,-5, 5);
                chassis.setArmPosition(6300, 1.0, true);
                chassis.drop();
                chassis.setArmPosition(5000, 0.2, true);
                chassis.moveRobotForward(0.0, 30);
                break;
        }
    }
}
