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

package org.firstinspires.ftc.teamcode.POC;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Vec2F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name="POC: TensorFlow Object Detection", group="POC")
//@TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")
@Disabled
public class ConceptTensorFlowObjectDetectionv1 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AYFREzX/////AAABmcPnFDqISEdopgXOzlcGHJt80ccNfKjcnQKVKS26WHgE3ehNGdF0oZsKGUtXr7sKpnbD/tq8zOnTDgIKOLDh4hsDolTJrjjqYOOMoRw2fG40nw8kIEC70ttSJJ2B8POSgOA5itYlC06rvBRT8GWK9+kxR8kIKqryIb7k3/cA+eXX2q3bS49AnxB21ZpLsBpoh4OebPoJcY8qxs+7uNi6p/Fqb3ShFX1O/Vuq0LOITiMjQW/7eTXPp+ZUMfx183wQnZKH3jnEpqFfp1jUAw/pKGGojD7O6Pj7ZXf0EmD9LN58V8PiVv8qizUyHQTOnuo04HrhzbKs/j5SzLsXf1CsOnGb9ayhEcFJi62bH013P0Ad";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector[] tfod = new TFObjectDetector[1];

    @Override
    public void runOpMode() {
		// The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
		// first.
		initVuforia();

        double startConfidence = .7;
        double confidenceStep = 0.5;

		if (ClassFactory.getInstance().canCreateTFObjectDetector()) {

		    for(int i=0; i<tfod.length; i++) {
                tfod[i] = initTfod(startConfidence + confidenceStep*i);
                tfod[i].activate();
            }
		} else {
			telemetry.addData("Sorry!", "This device is not compatible with TFOD");
		}

		/**
		 * Activate TensorFlow Object Detection before we wait for the start command.
		 * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
		 **/
/*        for(int k=0; k<tfod.length; k++) {
            if (tfod[k] != null) {
                tfod[k].activate();
            }
        }*/

		/** Wait for the game to begin */
		telemetry.addData(">", "Press Play to start op mode");
		telemetry.update();
		waitForStart();

		if (opModeIsActive()) {
			int iAttemptNo = 0;

			double stoneATop = 0;
			double stoneABottom = 0;
			double stoneBTop = 0;
			double stoneBBottom = 0;
			double stoneCTop = 0;
			double stoneCBottom = 0;
            double stoneskyTop = 0;
            double stoneskyBottom = 0;
			double skystoneCenter = 0;


			int stoneACount = 0;
            int stoneBCount = 0;
            int stoneCCount = 0;

			int recognitionsCount = 0;
            int recognitionsSkyCount = 0;
			double currentConfidence = startConfidence;
			int j = 0;

//            tfod[j] = initTfod(currentConfidence);
//            tfod[j].activate();

			while (opModeIsActive()) {

				for(j=0; j<tfod.length; j++) {
//				    tfod[j].activate();

                    currentConfidence = currentConfidence + confidenceStep;
                    telemetry.addLine("Current Confidence: " + j + " Recognitions: " + recognitionsCount);
                    //telemetry.addData("  left, right (%d)", "%.03f , %.03f",
                   //         (stoneABottom + stoneBTop) /2 , (stoneBBottom + stoneCTop) /2);
                    telemetry.addData("  A", "%d %.03f , %.03f",stoneACount, stoneATop , stoneABottom);
                    telemetry.addData("  B", "%d %.03f , %.03f",stoneBCount, stoneBTop , stoneBBottom);
                    telemetry.addData("  C", "%d %.03f , %.03f",stoneCCount, stoneCTop , stoneCBottom);
                    telemetry.addData("  Skystone: ", "%d %.03f , %.03f",recognitionsSkyCount, stoneskyTop , stoneskyBottom);
                    telemetry.update();


                    iAttemptNo++;
                    while((iAttemptNo % 2000000) != 0) {
                        iAttemptNo++;

//                        telemetry.addLine("Attempt: " + iAttemptNo + " Recognitions " + recognitionsCount);
  //                      telemetry.update();
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod[j].getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
//                            telemetry.addData("# Object Detected", updatedRecognitions.size() + " Attempt #: " + iAttemptNo);

                            // Vec2F size = vuforia.getCameraCalibration().getSize();
                            // telemetry.addLine("Size: 0=" + size.getData()[0] + " 1=" + size.getData()[1]);
                            int screenWidth = 1280;
                            // int stoneWidth = 320;
                            int screenMiddle = 640;
                            int stoneOffset = 60;

                            // step through the list of recognitions and display boundary info.
                            for (Recognition recognition : updatedRecognitions) {
                                if(recognition.getHeight() > screenWidth/3)
                                    continue;

                                recognitionsCount++;

                                // classify stone
                                if (Math.max(recognition.getBottom(), recognition.getTop()) < screenMiddle) // stone A
                                {
                                    stoneACount++;
                                    stoneATop = (stoneATop*(stoneACount-1) + Math.min(recognition.getBottom(), recognition.getTop())) / stoneACount;
                                    stoneABottom = (stoneABottom*(stoneACount-1) + Math.max(recognition.getBottom(), recognition.getTop())) / stoneACount;
//                                    stoneBTop = stoneABottom;
                                }

                                if (Math.min(recognition.getBottom(), recognition.getTop()) > screenMiddle) // stone C
                                {
                                    stoneCCount++;
                                    stoneCTop = (stoneCTop*(stoneCCount-1) + Math.min(recognition.getBottom(), recognition.getTop())) / stoneCCount;
                                    stoneCBottom = (stoneCBottom*(stoneCCount-1) + Math.max(recognition.getBottom(), recognition.getTop())) / stoneCCount;
  //                                  stoneBBottom = stoneCTop;
                                }

                                if(Math.min(recognition.getBottom(), recognition.getTop()) < screenMiddle && Math.max(recognition.getBottom(), recognition.getTop()) > screenMiddle) // stone B
                                {
                                    stoneBCount++;
                                    stoneBTop = (stoneBTop*(stoneBCount-1) + Math.min(recognition.getBottom(), recognition.getTop())) / stoneBCount;
                                    stoneBBottom = (stoneBBottom*(stoneBCount-1) + Math.max(recognition.getBottom(), recognition.getTop())) / stoneBCount;
                                }

//                                telemetry.addLine("Attempt: " + iAttemptNo + " Recognitions " + recognitionsCount + " Confidence: " + currentConfidence);
//                                telemetry.addData("  left, right (%d)", "%.03f , %.03f",
//                                        stoneBTop, stoneBBottom);

                                if(recognition.getLabel() == "Skystone") {
                                    recognitionsSkyCount++;
                                    stoneskyTop = (stoneskyTop*(recognitionsSkyCount-1) + Math.min(recognition.getBottom(), recognition.getTop())) / recognitionsSkyCount;
                                    stoneskyBottom = (stoneskyBottom*(recognitionsSkyCount-1) + Math.max(recognition.getBottom(), recognition.getTop())) / recognitionsSkyCount;
                                }

  //                              telemetry.addData("  Skystone: ", "%.03f", skystoneCenter);

    //                            telemetry.update();

					        /*float stoneMiddle = (recognition.getBottom() - recognition.getTop()) / 2 + recognition.getTop();
						    String stonePosition = "B";
					        if(stoneMiddle < 640 - stoneOffset)
							    stonePosition = "A";
						    else if(stoneMiddle > 640 + stoneOffset)
							    stonePosition = "C";

						    telemetry.addLine("Stone Position: " + stonePosition + " (Stone center: " +  stoneMiddle + ", Attempt: " + iAttemptNo + ")");

						    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
						    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
							recognition.getLeft(), recognition.getTop());
						    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
							    recognition.getRight(), recognition.getBottom());
						    telemetry.addData(String.format("  confidence,angle (%d)", i), "%.03f , %.03f",
							    recognition.getConfidence(), recognition.estimateAngleToObject(AngleUnit.DEGREES));*/
                            }
                        }
                    }

//                    tfod[j].deactivate();
  //                  tfod[j].shutdown();
  //                  tfod[j] = null;
  //                  tfod[j] = initTfod(currentConfidence);
                }

                if(currentConfidence > .85) {
                    currentConfidence = startConfidence;
                }
            }
		}

		for(int i=0; i<tfod.length; i++) {
             if (tfod[i] != null) {
                tfod[i].shutdown();
             }
		}
	}

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        //parameters.cameraDirection = CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private TFObjectDetector initTfod(double minConfidence) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.useObjectTracker = false;
        tfodParameters.minimumConfidence = minConfidence;
        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        return tfod;
    }
}
