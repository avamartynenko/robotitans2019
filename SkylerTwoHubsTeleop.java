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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SkylerTwoHubsTeleop", group="Pushbot")
//@Disabled
public class SkylerTwoHubsTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    CompetitionHardware robot = new CompetitionHardware();   // Use a Pushbot's hardware
    double right_stick_speed;
    double left_stick_speed;
    boolean elevatorLock = false;
    boolean initialized = false;


    @Override
    public void runOpMode() {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, false, false, false);
        robot.initTeleopModules();


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

          // BleftDriveSpeed,  BrightDriveSpeed,  FleftDriveSpeed,  FrightDriveSpeed
        robot.setPower4WDrive(gamepad1.left_stick_y,gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_y );

        if (gamepad1.right_trigger > 0){

            telemetry.addData("Right trigger pressed.", gamepad1.right_trigger);

            // BleftDriveSpeed,  BrightDriveSpeed,  FleftDriveSpeed,  FrightDriveSpeed
            robot.setPower4WDrive(gamepad1.right_trigger, -gamepad1.right_trigger, -gamepad1.right_trigger, gamepad1.right_trigger );


        }
        else if (gamepad1.left_trigger > 0 ) {

            telemetry.addData("Left trigger pressed.", gamepad1.left_trigger);

            // BleftDriveSpeed,  BrightDriveSpeed,  FleftDriveSpeed,  FrightDriveSpeed
            robot.setPower4WDrive(-gamepad1.left_trigger, gamepad1.left_trigger, gamepad1.left_trigger, -gamepad1.left_trigger);

        }


            //gamepad 1 ends here



            //gamepad 2 starts here

            //hooks
            if(gamepad2.dpad_up){


                robot.hookLatch.latch();
                telemetry.addData("dPad Up", " " + gamepad2.dpad_up);
                telemetry.update();


            } else if (gamepad2.dpad_down){

                robot.hookLatch.release();
                telemetry.addData("dPad Down", " " + gamepad2.dpad_down);
                telemetry.update();

            } else{

                //robot.hookLatch.stop();
                telemetry.addData(" both down ", " " + gamepad2.dpad_down);
                telemetry.addData(" both up", " " + gamepad2.dpad_up);
                telemetry.update();
            }



            //front side auton arm
            if(gamepad2.y){

                robot.frontArm.liftUp(0.4);
            }
            else if(gamepad2.a){

                robot.frontArm.goDown(0.4);
            }
            else {

                robot.frontArm.stop();
            }


            if(gamepad2.x){

                robot.frontArm.releaseStone(0.4);
            }
            else if(gamepad2.b){

                robot.frontArm.latchStone(0.4);
            }
            else{

                robot.frontArm.stop();
            }


            //intake motors
            if (gamepad2.left_trigger > 0){
                robot.intakeMech.run(gamepad2.left_trigger);
            } else if(gamepad2.right_trigger > 0) {
                robot.intakeMech.run(-gamepad2.right_trigger);
            } else{

                robot.intakeMech.stop();
            }



            //elevator
            robot.liftMech.runElevator(-gamepad2.left_stick_y);

            if(gamepad2.back){
                if(elevatorLock) {
                    robot.liftMech.runElevator(0.2);
                } else {
                    elevatorLock = false;
                }

            }


            //grabber
            if (gamepad2.left_bumper) {

                robot.liftMech.grabber.setPosition(robot.liftMech.GRABBER_RELEASE_POSITION); //0.15
                telemetry.addData("grabStone leftB pushing to 0.0", " " + gamepad2.left_bumper);
                telemetry.update();

            } else if (gamepad2.right_bumper){

                robot.liftMech.grabber.setPosition(robot.liftMech.GRABBER_LOCK_POSITION);
                telemetry.addData("grabStone rightB pushing to 1.0", " " + gamepad2.right_bumper);
                telemetry.update();

            }


            //Twist
            if (gamepad2.right_stick_x < 0)
            {
                robot.liftMech.twister.setPosition(robot.liftMech.TWISTER_HOME_POSITION);
                robot.liftMech.grabber.setPosition(robot.liftMech.GRABBER_RELEASE_POSITION); //0.15
                //robot.liftMech.grabber.setPosition(1.0);
            } else if(gamepad2.right_stick_x > 0 ){
                robot.liftMech.twister.setPosition(robot.liftMech.TWISTER_DELIVER_POSITION);
            }

            //Grab
            //if (gamepad2.right_stick_y > 0) robot.liftMech.grabber.setPosition(0.90);//0.85
            //if(gamepad2.right_stick_y < 0 ) robot.liftMech.grabber.setPosition(0.08); //0.15

            telemetry.addData("twistLift rightStickX", " " + gamepad2.right_stick_x);
            telemetry.update();

        }



        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        }
    }
