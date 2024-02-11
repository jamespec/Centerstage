package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Locale;

// This class offers Gyro controlled motion on a Robot with two motor drive.
// The two motors must be labelled as "left_drive" and "right_drive"
// The onboard IMU of the Control Hub is used to ensure accurate tracking.
// It must be labelled: "imu"

public class OmniMotorChassis
{
    private static final double MAX_TURN_SPEED = 0.4;

    private final DcMotor leftFrontDrive;
    private final DcMotor rightFrontDrive;
    private final DcMotor leftBackDrive;
    private final DcMotor rightBackDrive;
    private final IMU imu;
    private final Telemetry telemetry;

    OmniMotorChassis(HardwareMap hardwareMap, Telemetry telemetry )
    {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

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

        this.telemetry  = telemetry;
        this.telemetry.addData("Waiting for start", "");
        this.telemetry.update();
    }

    void moveForward( double targetAngle, int target, double power, double maxError )
    {
        double P = 0.0035;
        double I = 0.000005;  // 0.000001;
        double sumError = 0;
        double error;

        turn( targetAngle, maxError );
        
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setTargetPosition(target);
        leftFrontDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        while(rightFrontDrive.isBusy() )  // && rightDrive.isBusy())
        {
            error = (targetAngle - getHeading());
            sumError += error;
            
            double errorPower = (error * P) + (sumError * I);
            if (target < 0) {
                errorPower *= -1;
            }
            leftFrontDrive.setPower(power-errorPower);
            rightFrontDrive.setPower(power+errorPower);
            leftBackDrive.setPower(power-errorPower);
            rightBackDrive.setPower(power+errorPower);

            telemetry.addData("IMU", "Angle: %s", formatDegrees(getHeading()));
            telemetry.addData("ErrorPower", "%f", errorPower);
            telemetry.addData("Error", "%f", error);
            telemetry.update();
        }
        
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void turn(double targetAngle, double maxError)
    {
        turn(targetAngle, MAX_TURN_SPEED, maxError);
    }

    void turn(double targetAngle, double speed, double maxError)
    {
        ElapsedTime runtime = new ElapsedTime();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double P = 0.005;
        double I = 0.0000;
        double D = 0.0000;

        double error = getAngleError(targetAngle);
        double sumError = error;
        double prevError = error;
        double power = 0.0;
        double prevPower = 0.0;

        while( (Math.abs(error) > maxError ) && runtime.seconds() < 10 )
        {
            power = (error * P) + Math.signum(error) * 0.2 + (sumError * I) + ((error-prevError) * D);
            if (Math.abs(power) > speed ) {
                power = Math.signum(power) * speed;
            }
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);

            telemetry.addData("Power", "%f", power);
            telemetry.addData("Error", "%f", error);
            telemetry.update();

            prevError = error;            
            error = getAngleError(targetAngle);
            sumError += error;
        }
        
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Error", "%f", error);
        telemetry.update();
    }

    double getAngleError(double target) 
    {
        double error = (target - getHeading());
        
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;        
        
        return error;
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
