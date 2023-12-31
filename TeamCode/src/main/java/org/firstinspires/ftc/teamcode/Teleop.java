/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="PowerPlay: Teleop", group="Linear Opmode")
//@Disabled
public class Teleop extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    // Declare OpMode mem bers.
    //private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;



    //@Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();




        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double Speed = -gamepad1.left_stick_y;
            double Turn = -gamepad1.left_stick_x;
            double Strafe = -gamepad1.right_stick_x;
            double Slide = gamepad2.right_stick_y;
            double Grab = gamepad2.left_stick_x;
            double MAX_SPEED = 1.0;



            double numFl = 0.70*Range.clip((+Speed - Turn - Strafe), -1, +1);
            double numFr = 0.70*Range.clip((+Speed + Turn + Strafe), -1, +1);
            double numBl = 0.70*Range.clip((+Speed + Turn - Strafe), -1, +1);
            double numBr = 0.70*Range.clip((+Speed - Turn + Strafe), -1, +1);
            double numUp = 0.40*Range.clip((-Slide), -1, +1);
            double numGrab = 2*Range.clip((+Grab), -1, +1);



           /* if (gamepad2.b && rotation<2824)
                robot.arm.setTargetPosition(1000);
                robot.arm.getCurrentPosition();
                if (robot.arm.)
                robot.arm.setPower(.5);*/
            //Fabian Bafoonery

            //rotation values for height
            // small:negative -1875
            // medium: -3150
            // tall height: -4200




            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());

            double frontLeftPower = robot.frontLeft.getPower();
            double frontRightPower = robot.frontRight.getPower();
            double backLeftPower = robot.backLeft.getPower();
            double backRightPower = robot.backRight.getPower();
            double armPower = robot.arm.getPower();
            telemetry.addData("Arm height:", robot.arm.getCurrentPosition());
            telemetry.addData("Motors Power", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f), arm (%.2f)", frontLeftPower, frontRightPower, backLeftPower,backRightPower, armPower);
            telemetry.update();



            robot.arm.setPower(numUp - MAX_SPEED + MAX_SPEED);
            //robot.armB.setPower(numUp - MAX_SPEED + MAX_SPEED);
            robot.frontLeft.setPower(numFl - MAX_SPEED + MAX_SPEED);
            robot.frontRight.setPower(numFr - MAX_SPEED + MAX_SPEED);
            robot.backLeft.setPower(numBl - MAX_SPEED + MAX_SPEED);
            robot.backRight.setPower(numBr - MAX_SPEED + MAX_SPEED);
            robot.claw.setPower(numGrab - MAX_SPEED  + MAX_SPEED);

            /*//OPEN
            if (gamepad2.dpad_left){
                robot.clawL.setPosition(0.47);
                robot.clawR.setPosition(0.80);
            }

            //CLOSE
            if (gamepad2.dpad_right){
                robot.clawL.setPosition(0.52);
                robot.clawR.setPosition(0.48);
            }*/




            boolean vroom = true;
            if(vroom == true)
            {
                while(gamepad1.dpad_down)
                {
                    robot.frontLeft.setPower(1);
                    robot.frontRight.setPower(1);
                    robot.backLeft.setPower(1);
                    robot.backRight.setPower(1);
                    vroom = false;
                }
                if(vroom == false)
                {
                    robot.frontLeft.setPower(0);
                    robot.frontRight.setPower(0);
                    robot.backLeft.setPower(0);
                    robot.backRight.setPower(0);
                }
            }
            boolean vroom2 = true;
            if(vroom2 == true)
            {
                while(gamepad1.dpad_up)
                {
                    robot.frontLeft.setPower(-1);
                    robot.frontRight.setPower(-1);
                    robot.backLeft.setPower(-1);
                    robot.backRight.setPower(-1);
                    vroom2 = false;
                }
                if(vroom2 == false)
                {
                    robot.frontLeft.setPower(0);
                    robot.frontRight.setPower(0);
                    robot.backLeft.setPower(0);
                    robot.backRight.setPower(0);
                }
            }




            telemetry.update();
        }
    }
}
