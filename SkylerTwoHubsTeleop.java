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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


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

@TeleOp(name="SkylerTwoHubsTeleop", group="Competition")
//@Disabled
public class SkylerTwoHubsTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    CompetitionHardware robot = new CompetitionHardware();   // Use a Pushbot's hardware
    private double right_stick_speed;
    private double left_stick_speed;
    private boolean elevatorLock = false;
    private boolean hooksLatched = false;
    private double SENSITIVITY_DRIVE = 0.8;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, false, false, false);
        robot.initTeleopModules();
        robot.backArm.moveToHomePosition();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // BleftDriveSpeed,  BrightDriveSpeed,  FleftDriveSpeed,  FrightDriveSpeed
            if (gamepad1.left_stick_y > 0){
                left_stick_speed = gamepad1.left_stick_y * gamepad1.left_stick_y;
            }else if (gamepad1.left_stick_y < 0){
                left_stick_speed = -(gamepad1.left_stick_y * gamepad1.left_stick_y);
            }else{
                left_stick_speed = 0;
            }

            if(!gamepad1.left_stick_button)
                left_stick_speed = left_stick_speed * SENSITIVITY_DRIVE;

            if (gamepad1.right_stick_y > 0){
                right_stick_speed = gamepad1.right_stick_y * gamepad1.right_stick_y;
            } else if(gamepad1.right_stick_y < 0){
                right_stick_speed = -(gamepad1.right_stick_y * gamepad1.right_stick_y);
            } else{
                right_stick_speed = 0;
            }

            if(!gamepad1.right_stick_button)
                right_stick_speed = right_stick_speed * SENSITIVITY_DRIVE;

            robot.setPower4WDrive(-left_stick_speed, -right_stick_speed, -left_stick_speed, -right_stick_speed);

            double slowspeed = 0.4;
            if(gamepad1.dpad_down){
                robot.setPower4WDrive(-slowspeed, -slowspeed, -slowspeed, -slowspeed);
            }
            if(gamepad1.dpad_up){
                robot.setPower4WDrive(slowspeed, slowspeed, slowspeed, slowspeed);
            }
            if (gamepad1.dpad_left){
                robot.setPower4WDrive(slowspeed, -slowspeed, -slowspeed, slowspeed);
            }
            if (gamepad1.dpad_right){
                robot.setPower4WDrive(-slowspeed, slowspeed, slowspeed, -slowspeed );
            }

            if (gamepad1.right_trigger > 0){

                telemetry.addData("Right trigger pressed.", gamepad1.right_trigger);

                // BleftDriveSpeed,  BrightDriveSpeed,  FleftDriveSpeed,  FrightDriveSpeed
                //robot.setPower4WDrive(gamepad1.right_trigger, -gamepad1.right_trigger, -gamepad1.right_trigger, gamepad1.right_trigger ); // ORIG
                robot.setPower4WDrive(-gamepad1.right_trigger, gamepad1.right_trigger, gamepad1.right_trigger, -gamepad1.right_trigger );
            }
            else if (gamepad1.left_trigger > 0 ) {

                telemetry.addData("Left trigger pressed.", gamepad1.left_trigger);

                // BleftDriveSpeed,  BrightDriveSpeed,  FleftDriveSpeed,  FrightDriveSpeed
                //robot.setPower4WDrive(-gamepad1.left_trigger, gamepad1.left_trigger, gamepad1.left_trigger, -gamepad1.left_trigger); // ORIG
                robot.setPower4WDrive(gamepad1.left_trigger, -gamepad1.left_trigger, -gamepad1.left_trigger, gamepad1.left_trigger);
            }



            //intake motors
            if (gamepad1.left_bumper || hooksLatched) {

                hooksLatched = true;
                robot.hookLatch.latch();

            }

            if (gamepad1.right_bumper || !hooksLatched){

                hooksLatched = false;
                robot.hookLatch.release();

            }

            //gamepad 1 ends here

            //gamepad 2 starts here


            //front side auton arm
            if(gamepad2.y){

                robot.frontArm.liftUp(0.8);
            }
            else if(gamepad2.a){

                robot.frontArm.goDown(0.8);
            }
            else {

                robot.frontArm.stop();
            }


            if(gamepad2.x){

                robot.frontArm.releaseStone(0.8);
            }
            else if(gamepad2.b){

                robot.frontArm.latchStone(0.8);
            }
            else{

                robot.frontArm.stop();
            }

            //intake motors
            if (gamepad2.left_trigger > 0){
                robot.intakeMech.run(-robot.intakeMech.INTAKE_SPEED);
            } else if(gamepad2.right_trigger > 0) {
                robot.intakeMech.run(robot.intakeMech.INTAKE_SPEED);
            } else{

                robot.intakeMech.stop();
            }

            //elevator
            robot.liftMech.runElevator(-gamepad2.left_stick_y);

            if(gamepad2.left_stick_y>0 || gamepad2.left_stick_y<0){
                telemetry.addData("Lift Ele", " " + -gamepad2.left_stick_y);
                telemetry.update();
            }

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

            //telemetry.addData("twistLift rightStickX", " " + gamepad2.right_stick_x);
            //telemetry.update();

            //telemetry.addData("current Position", "L: "+robot.hookLatch.hookLeft.getPosition()+" R:"+robot.hookLatch.hookRight.getPosition());    //
            //telemetry.update();

        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        }
    }
