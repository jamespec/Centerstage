package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="DistanceAuto")
@Disabled
public class DistanceTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor wheel = null;
    private DistanceSensor leftDistanceSensor = null;
    private DistanceSensor rightDistanceSensor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        wheel      = hardwareMap.get(DcMotor.class, "wheel");
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right_distance");
           
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
            double leftDistanceCM;
            double rightDistanceCM;
            
            while(true) {
            // Show the elapsed game time and wheel power.
            leftDistanceCM = leftDistanceSensor.getDistance(DistanceUnit.CM);
            rightDistanceCM = rightDistanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("LeftDistanceCM", "%f", leftDistanceCM);
            telemetry.addData("RightDistanceCM", "%f", rightDistanceCM);
            telemetry.update();
            }
//        }
    }
}



