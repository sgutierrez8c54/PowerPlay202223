/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class CamAndPark extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double COUNTS_PER_MOTOR_REV = 384.5;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION*1.266) /
            (WHEEL_DIAMETER_INCHES * 3.14159265359);
    static final double DRIVE_SPEED = 0.4;
    static final double SAFE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

   //Tag IDs of sleeve
    int one_dot = 1;
    int two_dot = 2;
    int three_dot = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == one_dot || tag.id == two_dot || tag.id == three_dot)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
       if( tagOfInterest.id == one_dot) {
           robot.claw.setDirection(CRServo.Direction.FORWARD);
           robot.claw.setPower(-0.2);
           encoderDrive(DRIVE_SPEED,75,75,75,75,15);
           sleep(200);
           encoderDrive(DRIVE_SPEED,-11,-11,-11,-11,7);
           sleep(200);
           encoderDrive(DRIVE_SPEED,-16,16,16,-16 ,7); //prev 13
           sleep(200);
           robot.arm.setPower(0.10);
           sleep(3650);
           robot.arm.setPower(0.0);
           sleep(200);
           encoderDrive(DRIVE_SPEED,-4,4,4,-4 ,7);
           sleep(200);
           robot.claw.setPower(0.25);
           sleep(400);
           robot.claw.setPower(0);
           sleep(200);
           encoderDrive(DRIVE_SPEED,-4,-4,-4,-4 ,7);
           sleep(200);
           encoderDrive(DRIVE_SPEED,49,-49,-49,49 ,20);
           sleep(200);

       }else if (tagOfInterest.id == two_dot || tagOfInterest == null) {
           robot.claw.setDirection(CRServo.Direction.FORWARD);
           robot.claw.setPower(-0.2);
           encoderDrive(DRIVE_SPEED,75,75,75,75,15);
           sleep(200);
           encoderDrive(DRIVE_SPEED,-11,-11,-11,-11,7);
           //sleep(200);
           encoderDrive(DRIVE_SPEED,-16,16,16,-16 ,7); //prev 13
           sleep(200);
           robot.arm.setPower(0.10);
           sleep(3650);
           robot.arm.setPower(0.0);
           sleep(200);
           encoderDrive(DRIVE_SPEED,-4,4,4,-4 ,7);
           sleep(200);
           robot.claw.setPower(0.25);
           sleep(400);
           robot.claw.setPower(0);
           sleep(200);
           encoderDrive(DRIVE_SPEED,-4,-4,-4,-4 ,7);
           sleep(200);
           encoderDrive(DRIVE_SPEED,20,-20,-20,20 ,7);
           sleep(200);


       }else if (tagOfInterest.id == three_dot ) {
           robot.claw.setDirection(CRServo.Direction.FORWARD);
           robot.claw.setPower(-0.2);
           encoderDrive(DRIVE_SPEED,75,75,75,75,15);
           //sleep(200);
           encoderDrive(DRIVE_SPEED,-11,-11,-11,-11,7);
           //sleep(200);
           encoderDrive(DRIVE_SPEED,-16,16,16,-16 ,7); //prev 13
           sleep(200);
           robot.arm.setPower(0.10);
           sleep(3650);
           robot.arm.setPower(0.0);
           sleep(200);
           encoderDrive(DRIVE_SPEED,-4,4,4,-4 ,7);
           sleep(200);
           robot.claw.setPower(0.25);
           sleep(400);
           robot.claw.setPower(0);
           sleep(200);
           encoderDrive(DRIVE_SPEED,-4,-4,-4,-4 ,7);
           sleep(200);
           encoderDrive(DRIVE_SPEED,-18,18,18,-18 ,7);

       }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}

        /* PREVIOUS PATH WORKING
           robot.claw.setDirection(CRServo.Direction.FORWARD);
           robot.claw.setPower(-0.2);
           encoderDrive(DRIVE_SPEED,34,34,34,34,7);
           sleep(500);
           encoderDrive(DRIVE_SPEED,-13,13,13,-13 ,7);
           sleep(500);
           robot.arm.setPower(0.10);
           sleep(2800);
           robot.arm.setPower(0.0);
           sleep(500);
           encoderDrive(DRIVE_SPEED,-6,6,6,-6 ,7);
           sleep(500);
           robot.claw.setPower(0.25);
           sleep(200);
           robot.claw.setPower(0);
           sleep(500);
           encoderDrive(DRIVE_SPEED,16,-16,-16,16,7);
           sleep(500);
           robot.arm.setPower(-0.10);
           sleep(1850);
           robot.arm.setPower(0.0);
           encoderDrive(DRIVE_SPEED,30,30,30,30,7);
           sleep(500);
           encoderDrive(DRIVE_SPEED,-4,-4,-4,-4,7);
           sleep(500);
           encoderDrive(DRIVE_SPEED,20,-20,20,-20,7);
           sleep(500);
           encoderDrive(DRIVE_SPEED,28,24,24,30,7);
           sleep(500);
           robot.claw.setPower(-0.2);
           sleep(200);*/
    }

    public void encoderDrive(double speed,
                             double leftFinches,
                             double rightFinches,
                             double leftBinches,
                             double rightBinches,
                             double timeoutS) {
        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newLeftFTarget = robot.frontRight.getCurrentPosition() -  (int) (rightFinches * COUNTS_PER_INCH);
            newRightFTarget = robot.frontLeft.getCurrentPosition() - (int) (leftFinches * COUNTS_PER_INCH);
            newLeftBTarget = robot.backRight.getCurrentPosition() - (int) (rightBinches * COUNTS_PER_INCH);
            newRightBTarget = robot.backLeft.getCurrentPosition() - (int) (leftBinches * COUNTS_PER_INCH);

            robot.frontRight.setTargetPosition(newRightFTarget);
            robot.frontLeft.setTargetPosition(newLeftFTarget);
            robot.backRight.setTargetPosition(newRightBTarget);
            robot.backLeft.setTargetPosition(newLeftBTarget);

            // Turn On RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontRight.setPower(Math.abs(speed));
            robot.frontLeft.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFTarget, newRightFTarget, newLeftBTarget, newRightBTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(),
                        robot.backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(350);   // optional pause after each move. was 250 before
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}