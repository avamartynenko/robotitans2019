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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

@TeleOp(name="TeleopSylerOnlyWheels", group="Pushbot")
//@Disabled
public class TeleopSylerOnlyWheels extends LinearOpMode {

    /* Declare OpMode members. */
    //CompetitionHardware robot           = new CompetitionHardware();   // Use a Pushbot's hardware

    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;


    public void setPower4WDrive(double BleftDriveSpeed, double BrightDriveSpeed, double FleftDriveSpeed, double FrightDriveSpeed ){
        backLeft.setPower(BleftDriveSpeed*BleftDriveSpeed);
        backRight.setPower(BrightDriveSpeed*BrightDriveSpeed);
        frontLeft.setPower(-FleftDriveSpeed*FleftDriveSpeed);
        frontRight.setPower(-FrightDriveSpeed*FrightDriveSpeed);
    }


    @Override
    public void runOpMode() {
        /*
        double left;
        double right;
        double drive;
        double turn;
        double max;

         */

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap, false, false, false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

          // BleftDriveSpeed,  BrightDriveSpeed,  FleftDriveSpeed,  FrightDriveSpeed
        setPower4WDrive(gamepad1.left_stick_y,gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_y );

        if (gamepad1.right_trigger > 0){

            telemetry.addData("Right trigger pressed.", gamepad1.right_trigger);

            // BleftDriveSpeed,  BrightDriveSpeed,  FleftDriveSpeed,  FrightDriveSpeed
            setPower4WDrive(gamepad1.right_trigger, -gamepad1.right_trigger, -gamepad1.right_trigger, gamepad1.right_trigger );


        }
        else if (gamepad1.left_trigger > 0 ) {

            telemetry.addData("Left trigger pressed.", gamepad1.left_trigger);

            // BleftDriveSpeed,  BrightDriveSpeed,  FleftDriveSpeed,  FrightDriveSpeed
             setPower4WDrive(-gamepad1.left_trigger, gamepad1.left_trigger, gamepad1.left_trigger, -gamepad1.left_trigger);

        }


        }
    }
}
