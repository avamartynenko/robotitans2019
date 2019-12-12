/* Copyright (c) 2019 FIRST. All rights reserved.
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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.RIGHT;

// Use fast skystone detection method by comparing brightness of the stone instead of vuforia target recognition
@Autonomous(name="2 SkStns Dlvr - Blue", group ="Competition")
//@Disabled
public class TwoStoneBlueD extends BasicAutonEx {

    @Override public void runOpMode() {
        telemetry.addLine("Initializing robot. Please wait...");
        super.initialize();

        allianceColor = GAME_ALLIANCE_BLUE;
        setChoiceOfArm();

        telemetry.addLine("Init Completed. Detecting Skystone :)");
        telemetry.update();

        int iStonePos = -1;

        while(!opModeIsActive()) {
            iStonePos = getSkyStonePosition();
            opmodeRunTime.reset();
            if (iStonePos == -1) {
                telemetry.log().add("Unable to locate skystone");
                telemetry.update();
                iStonePos = 2;
            }
            else {
                // robot width 17.5 in
                // arm reach 5.5 in
                telemetry.addLine("Press (>) to Start when ready ;)");
                telemetry.update();
            }
        }

        telemetry.log().add("Starting Auton at: " + String.format("%.1f", opmodeRunTime.seconds()));

        robot.opStartHeading = robot.getActualHeading();


        robot.activateSpeedProfile = true;
        telemetry.log().add("Starting move to the left", "");
        robot.linearMove(RIGHT, 1, 25, this);

        //reverse counts for stones collection
        switch (iStonePos) {
            case 2:
                iStonePos = 0;
                break;
            case 0:
                iStonePos = 2;
                break;
                default:
                    break;
        }

        double FIRST_STONE_CORRECTION = 0;
        switch (iStonePos) {
            case 0:
                FIRST_STONE_CORRECTION = 1.5;
                break;
            case 1:
                FIRST_STONE_CORRECTION = 3.5;
                break;
            case 2:
                FIRST_STONE_CORRECTION = 2.5;
                break;
        }

        double distanceFromBWall = robot.sensorTimeOfFlightB.getDistance(INCH); // store distance

        telemetry.log().add("Adjusted stone pos " + iStonePos);
        telemetry.log().add("Distance to Back Wall (in)" + String.format("%.1f", distanceFromBWall));
        telemetry.update();

        double offsetToFirstSkyStone = distanceFromBWall - iStonePos*STONE_LENGTH - FIRST_STONE_CORRECTION;
        telemetry.log().add("requested offset " + offsetToFirstSkyStone);
        robot.linearMove(REVERSE, .9, offsetToFirstSkyStone, this);
        telemetry.log().add("In position for 1st stone: " + String.format("%.1f", opmodeRunTime.seconds()));
        telemetry.log().add("Distance to Back Wall (in) " + String.format("%.1f", robot.sensorTimeOfFlightB.getDistance(INCH)));

        correctGain();
        // grab skystone
        pickUpSkyStone();

        telemetry.log().add("1st stone collected: " + String.format("%.1f", opmodeRunTime.seconds()));
        telemetry.update();

        //robot.setHeading(0, this);

        // adjusting drop point to allow alliance robot to move foundation
        FIRST_STONE_DROP -= 32;

        // drop first skystone
        distanceFromBWall = robot.sensorTimeOfFlightB.getDistance(INCH) - 15;
        telemetry.log().add("Distance from back wall at collection: " + String.format("%.1f", distanceFromBWall));
        robot.linearMove(FORWARD, 1, FIRST_STONE_DROP - distanceFromBWall, this);


        telemetry.log().add("In position to drop 1st stone: " + String.format("%.1f", opmodeRunTime.seconds()));
        telemetry.update();
/*
        // gain correction not required for drop off program
        correctGain();
*/

        placeSkyStoneOnFoundation();
        telemetry.log().add("1st stone on the platform: " + String.format("%.1f", opmodeRunTime.seconds()));
        telemetry.update();


        //align and collect 2nd stone
        double distanceFromFWall = robot.sensorTimeOfFlightF.getDistance(INCH);

        double secondSkyStoneCorrection = 0;
        switch (iStonePos) {
            case 0:
                secondSkyStoneCorrection = 6.5;
                break;
            case 1:
                secondSkyStoneCorrection = 5.5;
                break;
            case 2:
                secondSkyStoneCorrection = 5;
                break;
        }


        double sendStonePick = FIELD_WIDTH - distanceFromFWall - 8 * (3 + iStonePos) - ROBOT_LENGTH + secondSkyStoneCorrection;
        telemetry.log().add("2nd stone pick. distance from Front wall " + String.format("%.1f", distanceFromBWall) + ". Second stone pic: " + String.format("%.1f", sendStonePick));
        robot.linearMove(REVERSE, 1, sendStonePick, this);


        // robot needs to idle a little, otherwise sensor report unreliable data
        sleep(50);
        correctGain();

        // TODO: add front wall correction

        telemetry.log().add("In position for 2nd stone: " + String.format("%.1f", opmodeRunTime.seconds()));

        // grab 2nd skystone
        pickUpSkyStone(true);
        telemetry.log().add("2nd stone collected: " + String.format("%.1f", opmodeRunTime.seconds()));
        telemetry.update();

        // adjust drop pint for second stone
        FIRST_STONE_DROP += 8;

        // drop second skystone
        distanceFromBWall = robot.sensorTimeOfFlightB.getDistance(INCH);
        telemetry.log().add("Distance from wall at collection: " + String.format("%.1f", distanceFromBWall));
        robot.linearMove(FORWARD, 1, FIRST_STONE_DROP - distanceFromBWall, this);
        telemetry.log().add("In position to drop 2nd stone: " + String.format("%.1f", opmodeRunTime.seconds()));
        telemetry.update();

/*
        // gain correction not required for drop only program
        correctGain();
*/
        placeSkyStoneOnFoundation();

        telemetry.log().add("2nd stone on the platform: " + String.format("%.1f", opmodeRunTime.seconds()));
        telemetry.update();

        robot.linearMove(REVERSE, 1, 20, this);

        telemetry.log().add("Under the bridge " + String.format("%.1f", opmodeRunTime.seconds()));
        telemetry.update();


        sleep(10000);
    }

    public void correctGain() {
        // correct gain
        double distanceFromLWall = robot.sensorTimeOfFlightL.getDistance(INCH); // measurements are off by ~2"
        boolean bTimeOut = robot.sensorTimeOfFlightL.didTimeoutOccur();
        telemetry.log().add("Distance to Left Wall (in) " + String.format("%.1f", distanceFromLWall) + (bTimeOut ? " TIMEOUT!!!" : ""));
        telemetry.update();

        if (distanceFromLWall < 27) {
            robot.linearMove(RIGHT, .7, 27 - distanceFromLWall - .5, this);
            telemetry.log().add("Correction Right (in) " + String.format("%.1f", 27 - distanceFromLWall));
        }

        if (distanceFromLWall > 28) {
            robot.linearMove(LEFT, .7, distanceFromLWall - 28 + .5, this);
            telemetry.log().add("Correction Left (in) " + String.format("%.1f", distanceFromLWall - 28.5 + .5));
        }

        telemetry.update();
    }
}
