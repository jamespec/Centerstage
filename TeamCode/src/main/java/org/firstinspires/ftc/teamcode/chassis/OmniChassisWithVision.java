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

package org.firstinspires.ftc.teamcode.chassis;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.vision.MarkerVisionProcessor;
import org.firstinspires.ftc.teamcode.vision.PixelVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.opencv.core.Point;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class OmniChassisWithVision
{
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagVisionProcessor;
    private final MarkerVisionProcessor markerVisionProcessor;
    private final PixelVisionProcessor pixelVisionProcessor;

    private final IMU     imu;              // Control/Expansion Hub IMU
    private final DcMotor leftFrontDrive;   //  Used to control the left front drive wheel
    private final DcMotor rightFrontDrive;  //  Used to control the right front drive wheel
    private final DcMotor leftBackDrive;    //  Used to control the left back drive wheel
    private final DcMotor rightBackDrive;
    private final DcMotor odometry;
    private final DcMotor arm;
    private final Servo   intake;
    private final Servo   drone;

    public OmniChassisWithVision(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        markerVisionProcessor = new MarkerVisionProcessor();
        pixelVisionProcessor = new PixelVisionProcessor();

        // Create the AprilTag processor by using a builder.
        aprilTagVisionProcessor = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagVisionProcessor.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                // .addProcessor(aprilTagVisionProcessor)
                // .addProcessor(markerVisionProcessor)
                .addProcessor(pixelVisionProcessor)
                .build();

        telemetry.addData("Camera", "Waiting");
        telemetry.update();
        // Carry on, we'll check for the vision being ready at the end.

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        odometry = hardwareMap.get(DcMotor.class, "odometry");

        intake = hardwareMap.get(Servo.class, "intake");
        drone = hardwareMap.get(Servo.class, "drone");
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

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for vision to finish initializing, only 10 secs.
        long start = System.currentTimeMillis();
        while ( visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING &&
                System.currentTimeMillis() < start + 10000 ) {
            sleep(100);
        }

        if( visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING ) {
            // setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
            telemetry.addLine("Waiting for start");
        }
        else
            telemetry.addLine("Waiting for start\n*** No Vision ***");

        telemetry.update();
    }

    public void moveToApril(int targetApril, double desiredDistance, double sideOffset ) {
        moveToApril( targetApril, desiredDistance, sideOffset, 30000);
    }

    // Automatically move robot in front of an April Tag, 'desiredDistance' back and 'sideOffset' to the side.
    public void moveToApril(int targetApril, double desiredDistance, double sideOffset, long maxTime )
    {
        final double SPEED_GAIN      = 0.04;  // Forward Speed Control "Gain".
        final double STRAFE_GAIN     = 0.02;  // Strafe Speed Control "Gain".
        final double TURN_GAIN       = 0.02;  // Turn Control "Gain".

        final double MAX_AUTO_SPEED  = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN   = 0.2;   //  Clip the turn speed to this max value (adjust for your robot) NOTE!!!! Was 0.3

        double rangeError = 100.0;
        double headingError = 1000.0;
        double yawError = 100.0;

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        long start = System.currentTimeMillis();

        while (Math.abs(rangeError) > 1.0 || Math.abs(headingError) > 4.0 || Math.abs(yawError) > 4.0) {
            AprilTagDetection desiredTag = null;
            boolean targetFound = false;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTagVisionProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == targetApril) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                moveRobot(0.0, 0.0, 0.0);
                telemetry.addLine("Target not found, giving up!\n");
                telemetry.update();
                sleep(10);
                if( System.currentTimeMillis() > start+(maxTime*1000) )
                    break;
                else
                    continue;
            }

            rangeError = (desiredTag.ftcPose.range - desiredDistance);
            headingError = -desiredTag.ftcPose.bearing - sideOffset;
            yawError = desiredTag.ftcPose.yaw;

            double drive = rangeError * SPEED_GAIN;
//            if (Math.abs(drive) < 0.10)
//                drive = Math.signum(drive) * 0.10; // Minimum power to overcome the dead zone of the motor.

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(drive, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            telemetry.update();

            sleep(10);
            if( System.currentTimeMillis() > start+(maxTime*1000) )
                break;
        }
        moveRobot(0.0, 0.0, 0.0);
    }

    public void moveRobotForward(double maxDrive, double strafe, double distInches)
    {
        final double SPEED_GAIN         = 0.04;    // Forward Speed Control "Gain".
        final double SPEED_I            = 0.02;    // Forward Speed I constant
        final double TURN_CORRECT_GAIN  = 0.04;    // Larger is more responsive, but also less stable
        final double MAX_TURN_CORRECT   = 0.3;     //  Clip the turn speed to this max value (adjust for your robot) NOTE!!!! Was 0.3

        double heading = getHeading();

        int distTicks = (int)((distInches * 25.4)/ (Math.PI * 48.0) * 2000.0);
        odometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double sumError = 0.0;
        int currentPos = -odometry.getCurrentPosition();
        double prevError = 0;
        double error = (distTicks - currentPos)/339.0;  //converted to inches, 339 ticks/inch

        // Correct while the error is larger than 0.25 inches or the error in still changing
        while ( Math.abs(error) > 0.25 || Math.abs(prevError - error) > 0.01) {
            // If the error is changing no need to collect sumError for I correction.
            if (Math.abs(error - prevError) < 0.05)
                sumError += error;
            else
                sumError = 0;

            double drive = error * SPEED_GAIN + sumError * SPEED_I;
            drive = Range.clip(drive, -maxDrive, maxDrive );

            double turn = getSteeringCorrection(heading, TURN_CORRECT_GAIN);
            turn = Range.clip(turn, -MAX_TURN_CORRECT, MAX_TURN_CORRECT);

            telemetry.addData("Heading Correcting Turn Clipped", "%5.2f", turn);
            telemetry.addData("Distance Ticks:", "%d", distTicks);
            telemetry.addData("Current Pos", "%d", currentPos);
            telemetry.addData("Drive Power", "%5.2f", drive);
            telemetry.addData("error", "%5.2f", error);
            telemetry.addData("preError", "%5.2f", prevError);
            telemetry.addData("sumError", "%5.2f", sumError);

            moveRobot( drive, strafe, turn );
            telemetry.update();

            sleep(10);
            currentPos = -odometry.getCurrentPosition();
            prevError = error;
            error = (distTicks - currentPos)/339.0;//inches
        }

        // Stop the robot once we reach the desired position.
        moveRobot( 0.0, 0.0, 0.0 );
    }

    public void turnRobotToHeading(double heading, double maxPower)
    {
        final double TURN_GAIN = 0.03;        // 0.1 power per degree error
        final double TURN_I    = 0.003;

        double sumError = 0.0;
        double prevError = 0;

        // Correct target heading
        while (heading > 180) heading -= 360;
        while (heading <= -180) heading += 360;

        double error = heading - getHeading();
        // Normalize the error to be within +/- 180 degrees
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        // Correct while the error is larger than 1 degree or the error in still changing
        while ( Math.abs(error) > 1.0 || Math.abs(prevError - error) > 0.01) {
            // If the error is changing no need to collect sumError for I correction.
            if (Math.abs(error - prevError) < 0.05)
                sumError += error;
            else
                sumError = 0;

            double turn = Range.clip(error * -TURN_GAIN + sumError * -TURN_I, -maxPower, maxPower);

            telemetry.addData("Heading Correcting Turn Clipped", "%5.2f", turn);
            telemetry.addData("error", "%5.2f", error);
            telemetry.addData("preError", "%5.2f", prevError);
            telemetry.addData("sumError", "%5.2f", sumError);

            moveRobot( 0.0, 0.0, turn );
            telemetry.update();

            sleep(10);
            prevError = error;
            error = heading - getHeading();
            // Normalize the error to be within +/- 180 degrees
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;
        }

        // Stop the robot once we reach the desired position.
        moveRobot( 0.0, 0.0, 0.0 );
    }

    public void moveRobot(double drive, double strafe, double turn)
    {
        // Calculate wheel powers.
        double leftFrontPower = drive - strafe - turn;
        double rightFrontPower = drive + strafe + turn;
        double leftBackPower = drive + strafe - turn;
        double rightBackPower = drive - strafe + turn;

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

        telemetry.addData("leftFrontPower", "%5.2f", leftFrontPower);
        telemetry.addData("rightFrontPower", "%5.2f", rightFrontPower);
        telemetry.addData("leftBackPower", "%5.2f", leftBackPower);
        telemetry.addData("rightBackPower", "%5.2f", rightBackPower);

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain)
    {
        // Determine the heading current error
        double headingError = desiredHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * -proportionalGain, -1, 1);
    }

    public double getHeading()
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void grab()
    {
        intake.setPosition(0.35);
    }

    public void drop()
    {
        intake.setPosition(0);
    }

    public void launch()
    {
        drone.setPosition(0.25);
    }

    public void setDrone(double power)
    {
        drone.setPosition(power);
    }

    public void setArmPosition(int position, double power, boolean wait)
    {
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
        while (wait && arm.isBusy()) {
            sleep(10);
        }
    }

    public int getArmPosition() {
        return arm.getCurrentPosition();
    }

    public void setArmPower(double power)
    {
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(power);
    }

    private void setManualExposure(int exposureMS, int gain)
    {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);
    }

    private void setAutomaticExposure()
    {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Auto) {
            exposureControl.setMode(ExposureControl.Mode.Auto);
            sleep(50);
        }
    }

    public MarkerVisionProcessor.Location getLocation() {
        return markerVisionProcessor.getLocation();
    }

    public Point getPixelCenters() { return pixelVisionProcessor.getCenter(); }

    private void sleep( int milli ) {
        try {
            Thread.sleep(milli);
        } catch( Exception ignored) {}
    }
}