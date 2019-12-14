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

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.BasicAutonEx;
import org.firstinspires.ftc.teamcode.CompetitionHardware;

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
@TeleOp(name="Diag: Move Test", group ="Util")
@Disabled
public class MovesTest extends BasicAutonEx {

    private int targetPostion = 0;

    private double safeDistanceOffset = 3;
    private double dropZoneOffset = 90;
    private double slowMoSpeed = .4;
    private int sleepTime = 100;
    private int detectionWaitTime = 2000;
    private int latchTime = 1250;


    @Override public void runOpMode() {
        super.initialize();

        telemetry.addLine("Init Completed. Detecting Skystone :)");
        telemetry.update();

        int iStonePos = -1;

        while(!opModeIsActive()) {
        }


        //waitForStart();
        //robot.opStartHeading = robot.getActualHeading();
        robot.verboseTelemetry = true;

        robot.linearMove(FORWARD, .75, 72, this);
        sleep(2000);
/*        robot.linearMove(LEFT, .75, 20, this);
        sleep(2000);*/
        robot.linearMove(REVERSE, .75, 72, this);
        sleep(2000);
/*        robot.linearMove(RIGHT, .75, 20, this);
        sleep(2000);*/

/*        robot.gyroMove90(GYRO_LEFT, telemetry);
        sleep(5000);

        robot.gyroMove90(GYRO_RIGHT, telemetry);
        sleep(5000);

        robot.gyroMoveByOffset(GYRO_LEFT, 1, 90, this);
        sleep(5000);

        robot.gyroMoveByOffset(GYRO_RIGHT, 1, 90, this);
        sleep(5000);*/
    }

}
