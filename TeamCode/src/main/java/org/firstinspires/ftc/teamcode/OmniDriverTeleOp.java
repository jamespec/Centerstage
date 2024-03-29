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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.chassis.OmniChassisWithVision;

@TeleOp(name = "OmniDriverTeleOp")
public class OmniDriverTeleOp extends LinearOpMode {
    OmniChassisWithVision chassis;

    final double TURN_CORRECT_GAIN = 0.04;    // Larger is more responsive, but also less stable
    final double MAX_TURN_CORRECT = 0.4;     //  Clip the turn speed to this max value (adjust for your robot) NOTE!!!! Was 0.3

    @Override
    public void runOpMode() {
        chassis = new OmniChassisWithVision(hardwareMap, telemetry, (OmniChassisWithVision.AprilTag | OmniChassisWithVision.Pixel) );

        double drive;      // Desired forward power/speed (-1 to +1)
        double strafe;     // Desired strafe power/speed (-1 to +1)
        double turn;       // Desired turning power/speed (-1 to +1)

        boolean captureNewHeading = false;
        double lastHeading = chassis.getHeading();

        waitForStart();

        while (opModeIsActive())
        {
            if (Math.abs(gamepad1.right_stick_x) > 0.01 || Math.abs(gamepad1.right_stick_y) > 0.01) {
                drive = -gamepad1.right_stick_y / 3.0;  // Reduce drive rate to 33%.
                strafe = -gamepad1.right_stick_x / 3.0;  // Reduce strafe rate to 33%.
            } else {
                drive = -gamepad1.left_stick_y;
                strafe = -gamepad1.left_stick_x / 1.2;
            }
            turn = (gamepad1.right_trigger - gamepad1.left_trigger) / 1.5;
            telemetry.addData("Uncorrected", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

            if (Math.abs(turn) > 0.05) {
                // Turn is requested, get new heading and don't correct
                captureNewHeading = true;
            } else {
                turn = chassis.getSteeringCorrection(lastHeading, TURN_CORRECT_GAIN);
                telemetry.addData("Heading Correcting Turn", "%5.2f", turn);

                // Clip the speed to the maximum permitted value.
                turn = Range.clip(turn, -MAX_TURN_CORRECT, MAX_TURN_CORRECT);
                telemetry.addData("Heading Correcting Turn Clipped", "%5.2f", turn);
            }
            chassis.moveRobot(drive, strafe, turn);

            double armPower = (gamepad2.right_trigger - gamepad2.left_trigger);
            chassis.setArmPower(armPower);
            telemetry.addData("Arm Power", "%3.0f", armPower);
            telemetry.addData("Arm Position", "%d", chassis.getArmPosition() );


            if (gamepad2.right_bumper) {
                chassis.setArmPower(0.2);
            }
            if (gamepad2.left_bumper) {
                chassis.setArmPower(-0.2);
            }
            if (gamepad2.x) {
                chassis.grab();
                telemetry.addLine("Grab");
            }
            if (gamepad2.y) {
                chassis.drop();
                telemetry.addLine("Drop");
            }

            if (gamepad2.back) {
                chassis.launch();
                telemetry.addLine("Launch");
            }

            telemetry.update();
            sleep(10);

            if( captureNewHeading ) {
                lastHeading = chassis.getHeading();
                captureNewHeading = false;
            }
        }
    }
}
