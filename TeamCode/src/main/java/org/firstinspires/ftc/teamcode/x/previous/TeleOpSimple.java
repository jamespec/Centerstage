package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class TeleOpSimple extends OpMode
{
    TwoMotorChassis twoMotorChassis = null;
    
    private BNO055IMU imu;
    private Orientation angles;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor wheel = null;
    private CRServo elevator = null;
    private CRServo intake = null;
    //private DistanceSensor distance = null;
    private ColorSensor color = null;
    private TouchSensor touch = null;
    private DigitalChannel red = null;
    private DigitalChannel green = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        wheel      = hardwareMap.get(DcMotor.class, "wheel");
        elevator   = hardwareMap.get(CRServo.class, "elevator");
        intake     = hardwareMap.get(CRServo.class, "intake");
        // distance   = hardwareMap.get(DistanceSensor.class, "distance");
        // color      = hardwareMap.get(ColorSensor.class, "color");
        touch      = hardwareMap.get(TouchSensor.class, "touch");
        red        = hardwareMap.get(DigitalChannel.class, "red");
        green      = hardwareMap.get(DigitalChannel.class, "green");

        red.setMode(DigitalChannel.Mode.OUTPUT);
        red.setState(true);
        
        green.setMode(DigitalChannel.Mode.OUTPUT);
        green.setState(true);
    
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        twoMotorChassis = new TwoMotorChassis( hardwareMap, telemetry );
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
       
       
       
       
       
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
