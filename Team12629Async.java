/*
 * Robot design for 2023-2024 FTC - Asynchronous period
 * 
 * Hardware:
 * 4 motors, for wheels
 * 1 motor and 2 servos, for claws
 * 1 servo, for claws
 * 
 +-+-+ PLAN FOR AUTONOMOUS PERIOD +-+-+
1. Drive forward to arms distance from each spike mark
2. Rotate the camera 360 degrees to scan the apriltag on the backboard
3. Program decides whether to drive the long or short distance
4. Turn in the direction of the apriltag.
5. Place down the purple pixel.
6. Drive forward to backboard.
7. Place yellow pixel on backboard.
8. Stay parked in backboard area.
*/

// FIRST Robotics imports
package org.firstinspires.ftc.teamcode;

import java.util.List;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
// Motors and Servos
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
// Webcam and April Tags
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class Test extends LinearOpMode {
    // Distance conversion
    Float degrees = 8667f/90f;
    Float feet = 7555f;
    // Wheel Init
    private DcMotor[] drivingMotors = new DcMotor[4];
    // Claw Init
    private DcMotor clawMotor;
    private Servo clawRotServo;
    private CRServo clawCloseServo;

    /*
    // Webcam Servo Init
    private CRServo camServo;
    // April Tag Init
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    // Misc Init
    */
    private ElapsedTime time = new ElapsedTime();

    private void halt() {
        for (DcMotor i : drivingMotors){
            i.setPower(0);
        }
    }

    private void forward(float distance){
        drivingMotors[0].setDirection(DcMotor.Direction.REVERSE);
        drivingMotors[3].setDirection(DcMotor.Direction.REVERSE);

        Float computedDistance = distance * feet;

        while (computedDistance > 1) {
            for (DcMotor i : drivingMotors){
                i.setPower(1);
            }
            computedDistance -= 1;
        }
        halt();
    }

    private void rotate(float angle){
        drivingMotors[0].setDirection(DcMotor.Direction.FORWARD);
        drivingMotors[3].setDirection(DcMotor.Direction.FORWARD);

        Float computedAngle = angle * degrees;

        while (computedAngle > 1) {
            for (DcMotor i : drivingMotors){
                i.setPower(1);
            }
            computedAngle -= 1;
        }
        for (DcMotor i : drivingMotors){
            i.setPower(computedAngle);
        }
        halt();
    }

    /*
     * Positions:
     * 1: rest
     * 2: arm up
     * 3: arm down
     */
    private Float armPos(float lastPos, float goalPos) {
        Float current;
        Float goal;
        Float clawTarget;

        // Sets current arm rotation
        if (lastPos == 1) {}
        else if (lastPos == 2) {}
        else if (lastPos == 3) {}

        // Sets goal arm rotation and target claw rotation
        if (goalPos == 1) {}
        else if (goalPos == 2) {}
        else if (goalPos == 3) {}

        // Computes the amount to move
        current -= goal;
        current *= -1;

        // Rotating the arm
        while (current < -1) {
            clawMotor.setPower(-1);
            current += 1;
        }
        while (current > 1) {
            clawMotor.setPower(1);
            current -= 1;
        }
        clawMotor.setPower(current);
        current = 0f;
        clawMotor.setPower(current);

        // Rotates the claw
        clawRotServo.setPosition(clawTarget);

        // If picking up or dropping, opens the claw
        if ((clawTarget == 2) || (clawTarget == 3)) {
            Double end = time.seconds() + 5;
            while (time.seconds() <= end) {
                clawCloseServo.setPower(1);
            }
            clawCloseServo.setPower(-1);
        }
    }

    // Code Running ------------------------------------------------------
    @Override
    public void runOpMode(){
        // Mapping
        drivingMotors[0] = hardwareMap.get(DcMotor.class, "motor_fl");
        drivingMotors[1] = hardwareMap.get(DcMotor.class, "motor_fr");
        drivingMotors[2] = hardwareMap.get(DcMotor.class, "motor_bl");
        drivingMotors[3] = hardwareMap.get(DcMotor.class, "motor_br");

        clawMotor = hardwareMap.get(DcMotor.class, "motor_arm");
        clawRotServo = hardwareMap.get(Servo.class, "claw_rot");
        clawCloseServo = hardwareMap.get(CRServo.class, "claw_close");

        /*
        camServo = hardwareMap.get(CRServo.class, "camera");
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        */

        // Setting motor and servo settings ------------------------------
        // Universal wheel properties
        for (DcMotor i : drivingMotors){
            // Tells motors to run based on power level, not velocity
            i.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Attempts to turn motor not resisted
            i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Claw properties
        clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        clawRotServo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawRotServo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawCloseServo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BREAK);
        clawCloseServo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

        // Set to starting positions -------------------------------------

        // Ready to start ------------------------------------------------
        Float factor = 0.5f;

        telemetry.addData(">", "Ready to start");
        waitForStart();
        time.reset();

        while (opModeIsActive()){

            // Output any debug or other info to controller
            telemetry.addData("Time", time.toString());
            telemetry.update();
        }
    }
}
