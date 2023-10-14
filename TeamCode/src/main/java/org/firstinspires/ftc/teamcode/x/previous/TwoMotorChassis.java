package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// This class offers Gyro controlled motion on a Robot with two motor drive.
// The two motors must be labelled as "left_drive" and "right_drive"
// The onboard IMU of the Control Hub is used to ensure accurate tracking.
// It must be labelled: "imu"

public class TwoMotorChassis
{
    private static final double MAX_TURN_SPEED = 0.4;

    private final DcMotor leftDrive;
    private final DcMotor rightDrive;
    private final BNO055IMU imu;
    private final Telemetry telemetry;

    TwoMotorChassis(HardwareMap hardwareMap, Telemetry telemetry )
    {
        this.leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        this.rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        this.imu        = hardwareMap.get(BNO055IMU.class, "imu");
        this.telemetry  = telemetry;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    void move( double targetAngle, int target, double power, double maxError )
    {
        double P = 0.0035;
        double I = 0.000005;  // 0.000001;
        double sumError = 0;
        double error;
        Orientation angles;

        turn( targetAngle, maxError );
        
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setTargetPosition(target);
        leftDrive.setTargetPosition(target);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while(rightDrive.isBusy() )  // && rightDrive.isBusy()) 
        {
            angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = (targetAngle - angles.firstAngle);
            sumError += error;
            
            double errorPower = (error * P) + (sumError * I);
            if (target < 0) {
                errorPower *= -1;
            }
            leftDrive.setPower(power-errorPower);
            rightDrive.setPower(power+errorPower);
            
            telemetry.addData("IMU", "Angle: %s", formatDegrees(angles.firstAngle));
            telemetry.addData("ErrorPower", "%f", errorPower);
            telemetry.addData("Error", "%f", error);
            telemetry.update();
        }
        
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // leftDrive.setPower(0);
        // rightDrive.setPower(0);
    }

    void turn(double targetAngle, double maxError)
    {
        turn(targetAngle, MAX_TURN_SPEED, maxError);
    }

    void turn(double targetAngle, double speed, double maxError)
    {
        ElapsedTime runtime = new ElapsedTime();

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double P = 0.005;
        double I = 0.0000;
        double D = 0.0000;

        double error = getAngleError(targetAngle);
        double rotationSpeed = getRotationSpeed();
        double sumError = error;
        double prevError = error;
        double power = 0.0;
        double prevPower = 0.0;

        while( (Math.abs(error) > maxError || Math.abs(rotationSpeed) > 0.1) && runtime.seconds() < 10 )
        {
            power = (error * P) + Math.signum(error) * 0.2 + (sumError * I) + ((error-prevError) * D);
            if (Math.abs(power) > speed ) {
                power = Math.signum(power) * speed;
            }
//            if (Math.abs(power-prevPower) > 0.005 ) {
//                power = prevPower + Math.signum(power) * 0.005;
//            }
            prevPower = power;

            leftDrive.setPower(-power);
            rightDrive.setPower(power);
            
            telemetry.addData("Power", "%f", power);
            telemetry.addData("Rotation", "%f", rotationSpeed);
            telemetry.addData("Error", "%f", error);
            telemetry.update();

            prevError = error;            
            error = getAngleError(targetAngle);
            rotationSpeed = getRotationSpeed();
            sumError += error;
        }
        
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        telemetry.addData("Error", "%f", error);
        telemetry.addData("Rotation", "%f", rotationSpeed);
        telemetry.update();
    }

    double getAngleError(double target) 
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        double error = (target - angles.firstAngle);
        
        while (error > 180)  error -= 360;
        while (error <= -180) error += 360;        
        
        return error;
    }

    double getRotationSpeed()
    {
        AngularVelocity v = imu.getAngularVelocity();
        return v.zRotationRate;
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
