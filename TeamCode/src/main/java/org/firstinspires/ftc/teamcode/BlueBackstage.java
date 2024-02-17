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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Blue Backstage")
public class BlueBackstage extends LinearOpMode {

    private MarkerVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable

    final double MAX_AUTO_TURN = 0.5;   //  Clip the turn speed to this max value (adjust for your robot) NOTE!!!! Was 0.3

    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private Servo intake = null;//  Used to control the right back drive wheel
    private IMU imu = null;      // Control/Expansion Hub IMU


    @Override
    public void runOpMode() throws InterruptedException {

        visionProcessor = new MarkerVisionProcessor();

        WebcamName web = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = VisionPortal.easyCreateWithDefaults(web, visionProcessor);

        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        intake = hardwareMap.get(Servo.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Waiting for start", "");
        telemetry.update();
        waitForStart();
        visionPortal.stopStreaming();

        MarkerVisionProcessor.Location location = visionProcessor.getLocation();
	
	

        double heading = getHeading();

        // Autonomous Commands Here


	if (location == MarkerVisionProcessor.Location.LEFT) {
        moveRobot(0.375, 0.0, heading, 1.3);
        moveRobot(0.0, 0.0, heading, 0.75);// speed, how fast strafe, heading, time
        moveRobot(0.0, 0.1, heading + 90, 2.0);
        moveRobot(0.0, 0.0, heading + 90, 0.75);
        moveRobot(0.3,0.0, heading + 90, 1.25);
        moveRobot(0.0, 0.0, heading + 90, 0.75);
        setArmPosition(500);
        moveRobot(0.0, 0.0, heading + 90, 0.75);
        moveRobot(0.325,0.0, heading + 90, 1.0);
        moveRobot(0.0, 0.0, heading + 90, 0.75);
        moveRobot(0.0, 0.4, heading + 90, 0.3);
        moveRobot(0.0, 0.0, heading + 90, 0.75);
        setArmPosition(6200);
        moveRobot(0.0, 0.0, heading + 90, 0.75);
        moveRobot(0.2, 0.0, heading + 90, 1.25);
        moveRobot(0.0, 0.0, heading + 90, 0.75);
		drop();
        moveRobot(0.0, 0.0, heading + 90, 0.75);
        moveRobot(-0.2, 0.0, heading + 90, 0.25);
        moveRobot(0.0, 0.0, heading + 90, 0.75);
        setArmPosition(4000);
        moveRobot(0.0, 0.0, heading + 90, 0.75);
        moveRobot(0.0, 0.4, heading + 90, 0.25);
        moveRobot(0.0, 0.0, heading + 90, 0.75);
        moveRobot(0.3, 0.0, heading + 90, 0.75);
        moveRobot(0.0, 0.0, heading + 90, 0.75);
	}
	
	if (location == MarkerVisionProcessor.Location.RIGHT) {
		moveRobot(0.35, 0.0, heading, 1.3);
		moveRobot(0.0, 0.0, heading, 1.0);
		moveRobot(0.0, 0.1, heading + 90, 2.0);
        moveRobot(0.0, 0.0, heading + 90, 1.0);
		moveRobot(-0.25, 0.0, heading + 90, 0.4);
        moveRobot(0.0, 0.0, heading + 90, 1.0);
		setArmPosition(500);
        moveRobot(0.0, 0.0, heading + 90, 1.0);
		moveRobot(0.3, 0.0, heading + 90, 0.6);
        moveRobot(0.0, 0.0, heading + 90, 1.0);
		moveRobot(0.0, 0.3, heading + 90, 0.4);
        moveRobot(0.0, 0.0, heading + 90, 1.0);
		setArmPosition(5500);
		moveRobot(0.0, 0.0, heading + 90, 1.0);
		drop();

	}

	if (location == MarkerVisionProcessor.Location.MIDDLE) {
		moveRobot(0.35, 0.0, heading, 1.8);
		moveRobot(0.0, 0.0, heading, 1.0);
		setArmPosition(500);
		moveRobot(0.0, 0.0, heading, 1.0);
		moveRobot(-0.25, 0.0, heading, 0.8);
        moveRobot(0.0, 0.0, heading, 1.0);
		moveRobot(0.0, 0.1, heading + 90, 2.0);
        moveRobot(0.0, 0.0, heading + 90, 1.0);
		moveRobot(0.3, 0.0, heading + 90, 0.4);
        moveRobot(0.0, 0.0, heading + 90, 1.0);
		setArmPosition(5500);
		moveRobot(0.0, 0.0, heading + 90, 1.0);
		drop();

	}

        

        telemetry.update();

        //sleep(10);

    }

    public void moveRobot(double drive, double stafe, double heading, double timeSeconds) {
        long start = System.currentTimeMillis();
        long now = start;

        while (now < start + timeSeconds * 1000) {
            double turn = getSteeringCorrection(heading, P_TURN_GAIN);
            turn = Range.clip(turn, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            telemetry.addData("Heading Correcting Turn Clipped", "%5.2f", turn);

            double leftFrontPower = drive - stafe - turn;
            double rightFrontPower = drive + stafe + turn;
            double leftBackPower = drive + stafe - turn;
            double rightBackPower = drive - stafe + turn;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            now = System.currentTimeMillis();
            sleep(10);

        }
    }


    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        double headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * -proportionalGain, -1, 1);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void grab() {
        intake.setPosition(0.35);
    }

    public void drop() {
        intake.setPosition(0);
    }

    public void setArmPosition(int position) {
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1.0);
        while (arm.isBusy()) {
            sleep(10);
        }
    }

}
