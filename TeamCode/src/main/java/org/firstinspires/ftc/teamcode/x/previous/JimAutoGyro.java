package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;

@Autonomous(name="JimAutoGyro")
@Disabled
public class JimAutoGyro extends LinearOpMode 
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    // The IMU sensor object
    private BNO055IMU imu;
        
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";

    @Override
    public void runOpMode() 
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");

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

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Wait for the game to start (driver presses PLAY)
        
        waitForStart();
        runtime.reset();
        
        move( 3000, 3000, 0.15 );
        //turn(90);
        sleep(3000);
    }

    private void move( double targetAngle, int target, double power )
    {
        double P = 0.0004;
        double I = 0.00001;  // 0.000001;
        double sumError = 0;
        double error;
        Orientation angles;

        turn( targetAngle );
        
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
    

    private void turn(double targetAngle)
    {
        double P = 0.004;
        double I = 0.000001;
        double MAX_ERROR = 0.325;
        double sumError = 0;
        Orientation angles;
        
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = (targetAngle - angles.firstAngle);

        while(Math.abs(error) > MAX_ERROR) 
        {
            angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = (targetAngle - angles.firstAngle);
            sumError += error;
            
            double power = (error * P) + Math.signum(error) * 0.2 + (sumError * I);
            leftDrive.setPower(-power);
            rightDrive.setPower(power);

            telemetry.addData("IMU", "Angle: %s", formatDegrees(angles.firstAngle));
            telemetry.addData("Power", "%f", power);
            telemetry.addData("Error", "%f", error);
            telemetry.update();
        }    
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

