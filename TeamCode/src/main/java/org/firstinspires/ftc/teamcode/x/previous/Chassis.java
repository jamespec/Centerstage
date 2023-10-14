package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;


public class Chassis 
{
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private BNO055IMU imu = null;
    private Telemetry telemetry = null;

    Chassis( DcMotor leftDrive, DcMotor rightDrive, BNO055IMU imu, Telemetry telemetry ) 
    {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.imu = imu;
        this.telemetry = telemetry;
        
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
        turn(targetAngle, 0.4, maxError);
    }

    void turn(double targetAngle, double speed, double maxError)
    {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        double P = 0.003;
        double I = 0.0001;
        double MP = 0.5;
        double sumError = 0;
        Orientation angles;

        double error = getAngleError(targetAngle);
        double prevError = error;
        while(Math.abs(error) > maxError)
        {
            sumError += error;
            double power = (error * P) + Math.signum(error) * 0.2 + (sumError * I);
            
            double maxSpeed = 1.0 - Math.min( (Math.abs(error - prevError) / error) * MP, 1.0 );
            maxSpeed = Math.min(maxSpeed, speed);
            
            if (Math.abs(power) > maxSpeed ) {
                power = Math.signum(power) * maxSpeed;
            }
            
            leftDrive.setPower(-power);
            rightDrive.setPower(power);
            
            //telemetry.addData("IMU", "Angle: %s", formatDegrees(angles.firstAngle));
            telemetry.addData("Power", "%f", power);
            telemetry.addData("Error", "%f", error);
            telemetry.update();

            prevError = error;            
            error = getAngleError(targetAngle);
        }
        
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        telemetry.addData("Error", "%f", error);
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

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
