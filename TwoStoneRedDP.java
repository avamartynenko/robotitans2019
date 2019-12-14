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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.GYRO_LEFT;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.GYRO_RIGHT;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.CompetitionHardware.Direction.RIGHT;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

// Use fast skystone detection method by comparing brightness of the stone instead of vuforia target recognition
@Autonomous(name="Red - 2 SkyStones Deliver & Place", group ="Competition")
//@Disabled
public class TwoStoneRedDP extends BasicAutonEx {

    private int targetPostion = 0;

    private double safeDistanceOffset = 3;
    private double dropZoneOffset = 90;
    private double slowMoSpeed = .4;
    private int sleepTime = 100;
    private int detectionWaitTime = 2000;
    private int latchTime = 1250;


    @Override public void runOpMode() {
        telemetry.addLine("Initializing robot. Please wait...");
        super.initialize();

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

        //waitForStart();
        robot.opStartHeading = robot.getActualHeading();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        //telemetry.addData("Offset ", "%.0f", SkyStoneOffset);

        robot.activateSpeedProfile = true;
        telemetry.log().add("Adjusted stone pos " + iStonePos);
        telemetry.log().add("Starting move to the right");
        robot.linearMove(RIGHT, 1, 25, this);

        double FIRST_STONE_CORRECTION = 0;
        switch (iStonePos) {
            case 0:
                FIRST_STONE_CORRECTION = 1.5;
                break;
            case 1:
                FIRST_STONE_CORRECTION = 1.5;
                break;
            case 2:
                FIRST_STONE_CORRECTION = 1.5;
                break;
        }

        double distanceFromFWall = getSensorValue(robot.sensorTimeOfFlightF, INCH); // store distance

        telemetry.log().add("Distance to Front Wall (in)" + String.format("%.1f", distanceFromFWall));
        telemetry.update();

        double offsetToFirstSkyStone = distanceFromFWall - iStonePos*STONE_LENGTH - FIRST_STONE_CORRECTION;
        robot.linearMove(FORWARD, .9, offsetToFirstSkyStone, this);
        telemetry.log().add("In position for 1st stone: " + String.format("%.1f", opmodeRunTime.seconds()));
        telemetry.log().add("Distance to Front Wall (in) " + String.format("%.1f", robot.sensorTimeOfFlightF.getDistance(INCH)));

    correctGain(false);
    // grab skystone
    pickUpSkyStone();
    telemetry.log().add("1st stone collected: " + String.format("%.1f", opmodeRunTime.seconds()));
    telemetry.update();
    //robot.setHeading(0, this);

    // drop first skystone
    distanceFromFWall = robot.sensorTimeOfFlightF.getDistance(INCH) - 15;
    telemetry.log().add("Distance from front wall at collection: " + String.format("%.1f", distanceFromFWall));
    robot.linearMove(REVERSE, 1, FIRST_STONE_DROP - distanceFromFWall, this);
    //robot.linearMoveEncoder(REVERSE, 1, FIRST_STONE_DROP - distanceFromFWall, this);

    telemetry.log().add("In position to drop 1st stone: " + String.format("%.1f", opmodeRunTime.seconds()));
    telemetry.update();

    // correct gain
    correctGain(true);

    placeSkyStoneOnFoundation();
    telemetry.log().add("1st stone on the platform: " + String.format("%.1f", opmodeRunTime.seconds()));
    telemetry.update();

    robot.setHeading(robot.opStartHeading, this);

    //align and collect 2nd stone
    double distanceFromBWall = robot.sensorTimeOfFlightB.getDistance(INCH);

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


    double sendStonePick = FIELD_WIDTH - distanceFromBWall - 8 * (3 + iStonePos) - ROBOT_LENGTH + secondSkyStoneCorrection;
    telemetry.log().add("2nd stone pick. distance from Back wall " + String.format("%.1f", distanceFromBWall) + ". Second stone pic: " + String.format("%.1f", sendStonePick));
    robot.linearMove(FORWARD, 1, sendStonePick, this);

    // robot needs to idle a little, otherwise sensor report unreliable data
    sleep(50);
    correctGain(false);

    // TODO: add front wall correction

    telemetry.log().add("In position for 2nd stone: " + String.format("%.1f", opmodeRunTime.seconds()));

    // grab 2nd skystone
    pickUpSkyStone(true);
    telemetry.log().add("2nd stone collected: " + String.format("%.1f", opmodeRunTime.seconds()));
    telemetry.update();


    // drop second skystone
    distanceFromFWall = robot.sensorTimeOfFlightF.getDistance(INCH);
    telemetry.log().add("Distance from wall at collection: " + String.format("%.1f", distanceFromFWall));
    robot.linearMove(REVERSE, 1, FIRST_STONE_DROP - distanceFromFWall, this);
    telemetry.log().add("In position to drop 2nd stone: " + String.format("%.1f", opmodeRunTime.seconds()));
    telemetry.update();

    // correct gain
    correctGain(true);

    placeSkyStoneOnFoundation();
    telemetry.log().add("2nd stone on the platform: " + String.format("%.1f", opmodeRunTime.seconds()));
    telemetry.update();

    robot.linearMove(FORWARD, 1, 35, this);


//    sleep(10000);
    }
}
