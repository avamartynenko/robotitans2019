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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 *
 */

//@Autonomous(name="BasicAuton", group="Pushbot")
//@Disabled
public class BasicAuton extends LinearOpMode {

    /* Declare OpMode members. */
    CompetitionHardware robot;
    Arm choiceOfArm;
    private ElapsedTime runtime = new ElapsedTime();

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    public static final int SKYSTONE_LEFT = 100;
    public static final int SKYSTONE_CENTER = 200;
    public static final int SKYSTONE_RIGHT = 300;
    public static final int GAME_ALLIANCE_RED = 1000;
    public static final int GAME_ALLIANCE_BLUE = 2000;


    public int allinceColor = GAME_ALLIANCE_RED;



    static final int MOVE_PAUSE = 100; // delay between linear moves
    static final double WALL_OVERRUN = 1; // home much father we run into the wall
    static final double WALL_RECOIL = .5; // how far we pull back from the wall
    static final double START_SPEED = .5; // highest possible speed with no slippage
    static final double MAX_SPEED = .75;



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap, true, false, true);
        initialize();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "init");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {

            telemetry.addData("Path", "Complete :))))");
            telemetry.update();
        }

    }


    public void initialize(){


        switch (allinceColor){

            case GAME_ALLIANCE_BLUE:
                robot = new ComeptitionHardwareForBlue();
                break;

                default:
                        robot = new CompetitionHardware();
                    break;

        }

    }


    public int detectSkyStone() {

        return SKYSTONE_CENTER;
    }

    public void goToSkyStone(int skyStonePosition) {

        switch (skyStonePosition) {
            case SKYSTONE_LEFT:

                break;

            case SKYSTONE_CENTER:

                break;

            case SKYSTONE_RIGHT:

                break;


            default:
                break;
        }

    }

    public void setChoiceOfArm(int gameAllianceColor){

        switch (gameAllianceColor) {
            case GAME_ALLIANCE_RED:
                choiceOfArm = robot.frontArm;
                break;

            case GAME_ALLIANCE_BLUE:
                choiceOfArm = robot.backArm;
                break;

            default:
                break;
        }

    }


    public void pickUpSkyStone () {

        choiceOfArm.goDown(0.1);
        sleep(3000);

        choiceOfArm.latchStone(0.5);
        sleep(3000);

        choiceOfArm.liftUp(0.1);
        sleep(3000);


    }

    public void moveToFoundation(int skyStonePosition){

        switch (skyStonePosition) {
            case SKYSTONE_LEFT:

                break;

            case SKYSTONE_CENTER:

                break;

            case SKYSTONE_RIGHT:

                break;


            default:
                break;
        }

    }

    public void placeSkyStoneOnFoundation(){

        choiceOfArm.goDown(0.1);
        sleep(3000);

        choiceOfArm.releaseStone(0.5);
        sleep(3000);

        choiceOfArm.liftUp(0.1);
        sleep(3000);

    }

    public void moveFoundationToBuildZone(){


    }

    public void getCube(Arm armToTest){

        telemetry.addData("Status", "Start arm motions...");
        telemetry.update();

        armToTest.goDown(0.5);
        sleep(1500);

        armToTest.stop();
        sleep(500);


        armToTest.latchStone(0.5);
        sleep(1500);

        armToTest.liftUp(0.5);
        sleep(1500);

        //armToTest.collectServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //telemetry.addData("Path 4",armToTest.collectServo.getDirection());
        //armToTest.collectServo.setPower(0.075);
        //sleep(3000);

        // release all motors
        armToTest.dropServo.setPower(0);
        //sleep(1000);

        telemetry.addData("Status", "Arm motions complete");
        telemetry.update();
    }

    public void dropCube(Arm armToTest){

        telemetry.addData("Status", "Start arm motions...");
        telemetry.update();

        armToTest.goDown(0.5);
        sleep(2500);

        armToTest.stop();
        sleep(1000);


        armToTest.releaseStone(0.5);
        sleep(2500);

        armToTest.liftUp(0.5);
        sleep(2500);

        //armToTest.collectServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //telemetry.addData("Path 4",armToTest.collectServo.getDirection());
        //armToTest.collectServo.setPower(0.075);
        //sleep(3000);

        // release all motors
        armToTest.dropServo.setPower(0);
        sleep(1000);

        telemetry.addData("Status", "Arm motions complete");
        telemetry.update();
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

    int linearMoveWrapper(int direction, double speed, double distance)
    {
        return robot.linearMove(direction, speed, distance);
    }

    public void setAllinceColor(int allinceColor) {
        this.allinceColor = allinceColor;
    }

}

