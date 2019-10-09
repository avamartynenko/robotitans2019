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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

@Autonomous(name="AutonTest", group="Pushbot")
//@Disabled
public class AutonTest extends LinearOpMode {

    /* Declare OpMode members. */
    CompetitionHardware robot = new CompetitionHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    static final int MOVE_PAUSE = 100; // delay between linear moves
    static final double WALL_OVERRUN = 1; // home much father we run into the wall
    static final double WALL_RECOIL = .5; // how far we pull back from the wall
    static final double START_SPEED = .5; // highest possible speed with no slippage
    static final double MAX_SPEED = 1;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true, false, true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "init");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive())
        {
            // get cube
            linearMoveWrapper(robot.FORWARD, 27, true); // runinto the wall
            linearMoveWrapper(robot.RIGHT, 27, false);

            // TODO: put get cube code here

            // place cube
            linearMoveWrapper(robot.LEFT, 28, true);
            linearMoveWrapper(robot.REVERSE, 115, true);
            sleep(100);

            linearMoveWrapper(robot.RIGHT, 29, false);
            linearMoveWrapper(robot.FORWARD,10, false);

            // TODO: put place cube code here

            // pullback and rotate
            linearMoveWrapper(robot.LEFT, 4, false);
            robot.linearMove(robot.GYRO_LEFT, MAX_SPEED, 21);
            linearMoveWrapper(robot.REVERSE, 4, false);

            // TODO: put grab platform code here

            // pull platform back
            linearMoveWrapper(robot.FORWARD, 32, false);

            // push platform to the corner
            linearMoveWrapper(robot.RIGHT, 27, false);
            linearMoveWrapper(robot.REVERSE, 19, false);
            linearMoveWrapper(robot.LEFT, 6, true);

            // retreat and park under the bridge

            linearMoveWrapper(robot.FORWARD, 19, true);
            linearMoveWrapper(robot.RIGHT, 26, false);
/*

            robot.linearMove(robot.FORWARD, 0.4, 18);

            sleep(5000);

            telemetry.addData("Path", "went forward");
            telemetry.update();

            robot.linearMove(robot.RIGHT, 0.4, 73);

            sleep(5000);

            telemetry.addData("Path", "strafed right");
            telemetry.update();

            //robot.linearMove(robot.FORWARD, 0.4, 15);

            //sleep(10000);

            robot.linearMove(robot.REVERSE, 0.4, 15);

            sleep(5000);

            telemetry.addData("Path", "went back");
            telemetry.update();

            robot.linearMove(robot.LEFT, 0.4, 35);

            sleep(5000);

            telemetry.addData("Path", "went left");
            telemetry.update();*/

            //robot.linearMove(robot.FORWARD, 0.4, 15);

            //sleep(10000);

//            String strPath = "FORWARD .4:18~RIGHT  .4:73~REVERSE .4:15~LEFT .4:35";

  //          followLinearPath(strPath, 5000);

            telemetry.addData("Path", "Complete :))))");
            telemetry.update();
        }
    }

    public void linearMoveWrapper(int Direction, double Distance, boolean bHitWall)
    {
        // accelerate to max speed for first 5", drive at max speed, slow down

        if(bHitWall)
            Distance += WALL_OVERRUN;

        robot.linearMove(Direction, MAX_SPEED, Distance);
        sleep(bHitWall ? 0 : MOVE_PAUSE);
        telemetry.addData("Path", "went left " + decodeDirection(Direction));
    }

    public void linearMoveWrapper2(int Direction, double Distance, boolean bHitWall)
    {
        // accelerate to max speed for first 5", drive at max speed, slow down

        if(bHitWall)
            Distance += WALL_OVERRUN;

        // assuming 5 step acceleration
        int accelerateSteps = (int)((Distance / 2) >= 5 ? 5 : (Distance / 2));

        // accelerate
        for(int i=0; i<accelerateSteps; i++)
            robot.linearMove(Direction, START_SPEED + i*.1, 1);

        // drive
        if(Distance > 2*accelerateSteps)
            robot.linearMove(Direction, MAX_SPEED, Distance - 2*accelerateSteps);

        // slow down
        for(int i=accelerateSteps; i>0; i--)
            robot.linearMove(Direction, START_SPEED + i*.1, 1);

        robot.linearMove(Direction, MAX_SPEED, Distance);

        sleep(bHitWall ? 0 : MOVE_PAUSE);
        telemetry.addData("Path", "went left " + decodeDirection(Direction));

        if(bHitWall)
        {
            robot.linearMove(reverseDirection(Direction), MAX_SPEED/2, WALL_RECOIL);
            sleep(MOVE_PAUSE/2);
            telemetry.addData("Path", "aligned against the wall ");
        }
    }

    public int reverseDirection(int Direction)
    {
        if(Direction == robot.LEFT)
            return robot.RIGHT;
        else if(Direction == robot.FORWARD)
            return robot.REVERSE;
        else if(Direction == robot.RIGHT)
            return robot.LEFT;
        else
            return robot.FORWARD;
    }

    public String decodeDirection(int Direction)
    {
        if(Direction == robot.LEFT)
            return "right";
        else if(Direction == robot.FORWARD)
            return "reverse";
        else if(Direction == robot.RIGHT)
            return "right";
        else
            return "forward";
    }

    // strPath format:
    // Command Param|Command Param
    public void followLinearPath(String strPath, int sleepInterval)
    {
        String[] strCommands = strPath.split("~");

        for(int i=0; i<strCommands.length; i++)
        {
            String[] strCommand = strCommands[i].split(" ");
            switch (strCommand[0])
            {
                case "FORWARD":
                    linearMoveWrapper(robot.FORWARD, strCommand[1], sleepInterval);
                    break;
                case "REVERSE":
                    linearMoveWrapper(robot.REVERSE, strCommand[1], sleepInterval);
                    break;
                case "LEFT":
                    linearMoveWrapper(robot.LEFT, strCommand[1], sleepInterval);
                    break;
                case "RIGHT":
                    linearMoveWrapper(robot.RIGHT, strCommand[1], sleepInterval);
                    break;
                default:
                    //throw new Exception("Command not fouNd");
                    break;
            }
        }
    }

    public void linearMoveWrapper(int intDirection, String strParams, int sleepInterval)
    {
        String[] strParam = strParams.split(":");
        robot.linearMove(intDirection, Double.parseDouble(strParam[0]), Double.parseDouble(strParam[1]));

        sleep(sleepInterval);

        String strMessage;
        strMessage = "Uninitialized direction. Check your path declaration";

        if(intDirection == robot.FORWARD)
            strMessage = "moved forward";
        else if(intDirection == robot.LEFT)
            strMessage = "moved left";
        else if(intDirection == robot.RIGHT)
            strMessage = "moved right";
        else if(intDirection == robot.REVERSE)
            strMessage = "moved back";

        telemetry.addData("Path", strMessage);
        telemetry.update();
    }
}
