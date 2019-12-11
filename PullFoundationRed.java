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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.GYRO_RIGHT;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.REVERSE;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="RED_PullFoundation", group="Pushbot")
//@Disabled
public class PullFoundationRed extends BasicAuton {

    @Override
    public void runOpMode() {

        // refresh competition hardware, arm. hook
        // code is in servotest class
        super.initialize();
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap, true, false, true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "init AutonTestEx");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        if (opModeIsActive())
        {
            reOrient();
            //robot.linearMove(robot.LEFT,0.6,11);
            robot.linearMove(REVERSE,0.5,32);
            //robot.linearMove(CompetitionHardware.Direction.REVERSE,0.1,2);
            robot.hookLatch.latch();
            sleep(1000);
            robot.gyroMove(GYRO_RIGHT,0.2,13);
            //sleep(1000);
            robot.linearMove(FORWARD,0.8,25);
            robot.gyroMove(GYRO_RIGHT,0.5,80);
            robot.linearMove(REVERSE,0.8,5);
            robot.hookLatch.release();
            robot.linearMove(LEFT,0.8,10);
            //robot.linearMove(robot.LEFT,0.5,8);
            robot.linearMove(FORWARD,0.8,40);
            //sleep(20000);
        }
    }
}
