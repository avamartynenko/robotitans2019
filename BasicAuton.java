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


    public int allianceColor = GAME_ALLIANCE_RED;

    /*
    public BasicAuton(){
        initialize();
        telemetry.addData("Status Basic Auton", "init complete"+robot.toString());    //
        telemetry.update();
    }
     */

    static final int MOVE_PAUSE = 100; // delay between linear moves
    static final double WALL_OVERRUN = 1; // home much father we run into the wall
    static final double WALL_RECOIL = .5; // how far we pull back from the wall
    static final double START_SPEED = .5; // highest possible speed with no slippage
    static final double MAX_SPEED = .75;



    @Override
    public void runOpMode() {
    }


    public void initialize(){

        telemetry.addData("Status", "initialize BasicAuton"+ allianceColor);    //
        telemetry.update();

        switch (allianceColor){

            case GAME_ALLIANCE_BLUE:
                robot = new ComeptitionHardwareForBlue();
                break;

            default:
                robot = new CompetitionHardware();
                break;

        }

        robot.init(hardwareMap, true, false, true);

        setChoiceOfArm();

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

    public void setChoiceOfArm(){

        switch (allianceColor) {
            case GAME_ALLIANCE_RED:
                choiceOfArm = robot.frontArm;
                telemetry.addData("ChoiceOfArm", " frontArm");
                telemetry.update();
                break;

            case GAME_ALLIANCE_BLUE:
                choiceOfArm = robot.backArm;
                telemetry.addData("ChoiceOfArm", " backArm");
                telemetry.update();
                break;

            default:
                break;
        }





    }


    public void pickUpSkyStone () {

        choiceOfArm.latchStone(0.9);
        choiceOfArm.goDown(0.9);

        sleep(800);

        //sleep(1000);

        choiceOfArm.liftUp(0.9);
        sleep(800);


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

    public void getCube(){

        telemetry.addData("Status", "Start arm motions...");
        telemetry.update();

        choiceOfArm.goDown(0.5);
        sleep(1500);

        choiceOfArm.stop();
        //sleep(500);


        choiceOfArm.latchStone(0.5);
        sleep(1500);

        choiceOfArm.liftUp(0.5);
        sleep(1500);

        //armToTest.collectServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //telemetry.addData("Path 4",armToTest.collectServo.getDirection());
        //armToTest.collectServo.setPower(0.075);
        //sleep(3000);

        // release all motors
        choiceOfArm.dropServo.setPower(0);
        //sleep(1000);

        telemetry.addData("Status", "Arm motions complete");
        telemetry.update();
    }

    public void dropCube(){

        telemetry.addData("Status", "Start arm motions...");
        telemetry.update();

        choiceOfArm.goDown(0.5);
        sleep(2500);

        choiceOfArm.stop();
        //sleep(1000);


        choiceOfArm.releaseStone(0.5);
        sleep(2500);

        choiceOfArm.liftUp(0.5);
        sleep(2500);

        //armToTest.collectServo.setDirection(DcMotorSimple.Direction.FORWARD);
        //telemetry.addData("Path 4",armToTest.collectServo.getDirection());
        //armToTest.collectServo.setPower(0.075);
        //sleep(3000);

        // release all motors
        choiceOfArm.dropServo.setPower(0);
        //sleep(1000);

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

    public void setAllianceColor(int allianceColor) {
        this.allianceColor = allianceColor;
    }


    public void reOrient(){

        if (allianceColor == GAME_ALLIANCE_BLUE){

            ComeptitionHardwareForBlue.setCurrentOrtientation(ComeptitionHardwareForBlue.ORIENTATION_TWO);

        }

    }

}

