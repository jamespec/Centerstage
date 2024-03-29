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


@Autonomous(name="SimpleAutoTest")
public class SimpleAutoTest extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException {
        OmniChassisWithVision chassis = new OmniChassisWithVision(hardwareMap, telemetry, 0);
        double heading = chassis.getHeading();

        waitForStart();

        chassis.moveRobotForward(0.2,60.0);

        //sleep(100000);

//        chassis.moveToApril(1, 12, 0);
//        chassis.moveRobotForward(0.3, 0.0, -20);
//        chassis.moveToApril(2, 12, 0);
//        chassis.moveRobotForward(0.3, 0.0, -20);
//        chassis.moveToApril(3, 12, 0);

        return;

//        switch (chassis.getLocation() ) {
//            case RIGHT:
//                chassis.moveRobotForward(0.5, 0.0, 27);
//                chassis.turnRobotToHeading(heading+90, 0.4);
//                chassis.moveRobotForward(0.5, 0.0, -1);
//                chassis.setArmPosition(500, 0.3);
//                chassis.moveRobotForward(0.5, 0.0, 60);
//                break;
//
//            case MIDDLE:
//                chassis.moveRobotForward(0.5, 0.0, 40);
//                chassis.setArmPosition(500, 0.3);
//                chassis.moveRobotForward(0.5, 0.0, 10);
//                chassis.turnRobotToHeading(heading+90, 0.4);
//                chassis.moveRobotForward(0.5, 0.0, 60);
//                break;
//
//            case LEFT:
//                chassis.moveRobotForward(0.5, 0.0, 27);
//                chassis.turnRobotToHeading(heading+90, 0.4);
//                chassis.moveRobotForward(0.5, 0.0, 21);
//                chassis.setArmPosition(500, 0.3);
//                chassis.moveRobotForward(0.5, 0.0, 39);
//                break;
//        }

        //chassis.turnRobotToHeading(heading-90, 0.4);

        //chassis.moveRobotForward(0.5, 0.0, 48);
        //sleep(5000);
        //chassis.moveToApril(3, 12, 0);
        //chassis.setArmPosition(6200, 1.0);
        //chassis.drop();
        //chassis.setArmPosition(4750, 0.2);
    }
}
