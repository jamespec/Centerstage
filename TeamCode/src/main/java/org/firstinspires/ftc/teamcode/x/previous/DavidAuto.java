package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

@Autonomous(name="DavidAuto")
@Disabled
public class DavidAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor wheel = null;
    private CRServo elevator = null;
    private CRServo intake   = null;
    private DistanceSensor leftDistanceSensor = null;
    private DistanceSensor rightDistanceSensor = null;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck"
      // , "Marker"
    };

private static final String VUFORIA_KEY =
            "AROHLt//////AAABmRqtKHXnCETrrKry+MJXVcdqPPGOE6f4kvj9Kh5mhFwXKquVVjlrr+9T0G3ckUn7PEtQocoVIAwVfdY7y1cqkibeTI3IxRhf1taeZO+ovnE8T++Udbtqy8Y9sN9IwHbXus5AXTLp1s1jEZGB5thPT7rLACUDOouus47BeF8Ygyj5ygGSDIGEh0hWdwtF3G4zI1HUjPNtVakFVYQMqIaaUmv0FtTRUJP+2aBeEtETU5dmuq6eLZ76sHWarETv+lzUe1rOZM7fTNKfRGT/M6TZZXmnbOB2w45TM6nS5Z15vUjzuvX+L+Nxn+BeaAjmPpWk87gecXYnTyQ5bz+oOJtld5EkIMgu3DSyFEg46O374Ytr";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    //private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
     
    private TFObjectDetector tfod;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "DO NOT RUN");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        wheel      = hardwareMap.get(DcMotor.class, "wheel");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right_distance");
        elevator   = hardwareMap.get(CRServo.class, "elevator");
        intake     = hardwareMap.get(CRServo.class, "intake");
        int level = 1;
        
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        // Wait for the game to start (driver presses PLAY)
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            //tfod.setZoom(1.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start Autonomous");
        telemetry.update();
        waitForStart();
        runtime.reset();
        
        
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
              telemetry.addData("# Object Detected", updatedRecognitions.size());
              // step through the list of recognitions and display boundary info.
              int i = 0;
              for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                i++;
                if (recognition.getLeft() > 280) {
                    level = 3;
                }
                else {
                    level = 2;
                }
            }
          }
        }
   
        telemetry.addData("level", "(%d)", level);
        telemetry.update();
        
        
        //Supposed to know what level duck is going to
        // Setup a variable for each drive wheel to save power level for telemetry
        
        double leftPower = 0.2;
        double rightPower = 0.2;
        double wheelPower = .75;
        
        //first step
        move( leftDrive, rightDrive, -900, -900, 0.2 );

        // raise elevator - slow
        if (level == 3) {
        elevator.setPower(-0.125);
        }
        
        wheel.setPower(wheelPower);
        
        sleep(3000);//3 second wait
        wheel.setPower(0);
       
        
        //backup against wall
        move( leftDrive, rightDrive, 2175, 2175, 0.2 );
        
        // TURN 90 degrees right
        move( leftDrive, rightDrive, 360, -360, 0.2 );
        
        //Back  into wall
        move( leftDrive, rightDrive, -200, -200, 0.2 );
    
        if (level == 1) {
            elevator.setPower(-0.3);
            sleep(1500);
            elevator.setPower(-0.06);
            // Move to Hub
        move( leftDrive, rightDrive, 1155, 1155, 0.2 );
        }
        
        if (level == 2) {
            elevator.setPower(-0.3);
            sleep(2000);
            elevator.setPower(-0.06);
            // Move to Hub
        move( leftDrive, rightDrive, 1150, 1150, 0.2 );
        }
        
        if (level == 3) {
        move( leftDrive, rightDrive, 1120, 1120, 0.2 );
        }
    
        intake.setPower(0.15);
        sleep(1500);
        intake.setPower(0.0);
        
        // Backup            
        move( leftDrive, rightDrive, -1000, -1000, 0.2 );
        elevator.setPower(0.2);

        // Turn left
        move( leftDrive, rightDrive, -375, 375, 0.2 );
        elevator.setPower(0.1);

        // Drive into warehouse
        move( leftDrive, rightDrive, 3000, 3000, 0.5 );
        
        elevator.setPower(0.0);
        
        // double leftDistanceCM = leftDistanceSensor.getDistance(DistanceUnit.CM);
        // double rightDistanceCM = rightDistanceSensor.getDistance(DistanceUnit.CM);

        // while(true) {
        // // Show the elapsed game time and wheel power.
        // leftDistanceCM = leftDistanceSensor.getDistance(DistanceUnit.CM);
        // rightDistanceCM = rightDistanceSensor.getDistance(DistanceUnit.CM);
        // telemetry.addData("Status", "Run Time: " + runtime.toString());
        // telemetry.addData("LeftDistanceCM", "%f", leftDistanceCM);
        // telemetry.addData("RightDistanceCM", "%f", rightDistanceCM);
        // telemetry.update();
        // }
//        }
    }
    
    private void move( DcMotor leftDrive, DcMotor rightDrive, int leftTarget, int rightTarget, double power )
    {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setTargetPosition(leftTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while(leftDrive.isBusy() || rightDrive.isBusy()) {
            int rightPos = rightDrive.getCurrentPosition();
            int leftPos  = leftDrive.getCurrentPosition();
            telemetry.addData("LeftPos", "%d", leftPos);
            telemetry.addData("RightPos", "%d", rightPos);
            telemetry.update();
        }
        
        //leftDrive.setPower(0);
        //rightDrive.setPower(0);
    }
    
    private void turn( DcMotor leftDrive, DcMotor rightDrive, int degrees, double power )
    {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; //Doesn't work for either extrinsic or intrinsic.

        // rightDrive.setTargetPosition(rightTarget);
        // leftDrive.setTargetPosition(leftTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // leftDrive.setPower(power);
        // rightDrive.setPower(power);

        // while(leftDrive.isBusy() || rightDrive.isBusy()) {
        //     int rightPos = rightDrive.getCurrentPosition();
        //     int leftPos  = leftDrive.getCurrentPosition();
        //     telemetry.addData("LeftPos", "%d", leftPos);
        //     telemetry.addData("RightPos", "%d", rightPos);
        //     telemetry.update();
        // }
        
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        
        
        
    }
    
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//       tfodParameters.minResultConfidence = 0.8f;
//       tfodParameters.isModelTensorFlow2 = true;
//       tfodParameters.inputSize = 320;
//       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    
}

