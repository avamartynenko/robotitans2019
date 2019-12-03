package org.firstinspires.ftc.teamcode.POC;

/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.CvType.CV_32F;
import static org.opencv.core.CvType.CV_8UC1;
import static org.opencv.imgproc.Imgproc.COLORMAP_AUTUMN;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2GRAY;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HLS;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.COLOR_HLS2BGR;
import static org.opencv.imgproc.Imgproc.COLOR_HSV2BGR;
import static org.opencv.imgproc.Imgproc.COLOR_HSV2RGB;
import static org.opencv.imgproc.Imgproc.COLOR_HSV2RGB_FULL;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2GRAY;

@TeleOp
@Disabled
public class InternalCameraExample extends LinearOpMode
{
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(new SamplePipeline());

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                phoneCam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * The "if" statements below will pause the viewport if the "X" button on gamepad1 is pressed,
             * and resume the viewport if the "Y" button on gamepad1 is pressed.
             */
            else if(gamepad1.x)
            {
                phoneCam.pauseViewport();
            }
            else if(gamepad1.y)
            {
                phoneCam.resumeViewport();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
/*            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);*/

            /* Remove blue channel - results are ok, but not great

            List<Mat> channels = new ArrayList<>();
            Core.split(input, channels);
            channels.get(2).setTo(Scalar.all(0));
            Core.merge(channels, input);

            end remove blue channel */


            // select yellow only in HSV - works little better than removing blue


            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, COLOR_BGR2HSV);
            Mat mask = new Mat();

            Core.inRange(hsv, new Scalar(85, 127, 127),
                    new Scalar(115, 255, 255), mask);

            Mat result = new Mat();
            Core.bitwise_and(input, input, result, mask);

            hsv.release();

            // equalize histogram
            //Mat eq = new Mat();
            //Mat bgr = new Mat();
            //Imgproc.cvtColor(result, bgr, COLOR_HSV2BGR);
            //Imgproc.equalizeHist(bgr, eq);

            return result;

/*            List<Mat> channels = new ArrayList<>();
            Core.split(result, channels);

            Mat product = new Mat();
            Core.add(channels.get(1), channels.get(2), product);

            channels.set(2, product);
            Mat merge = new Mat();
            Core.merge(channels, merge);

            return merge;
            //Mat emboss =  emboss(result);*/

            /* conviert to greyscale
            List<Mat> channels = new ArrayList<>();
            Core.split(result, channels);

            Mat greyscale = channels.get(1);
            Mat hcGs = new Mat();
            greyscale.convertTo(hcGs, -1,2, 0);*/
            /*return hcGs;*/
            /*return  greyscale;
            end select yellow*/



//            return findContours(input);

            //return edges;

            //Medges = cv2.Canny(image,100,200)

            //Imgproc.Canny(emboss, emboss, 100, 200);
            //Imgproc.GaussianBlur(greyscale, greyscale, new  org.opencv.core.Size(5, 5), 5); */


            //return emboss;

            //return doBackgroundRemoval(input);

//            return input;
        }

        private Mat emboss(Mat src)
        {
            Mat grey = new Mat();
            Imgproc.cvtColor(src, grey, Imgproc.COLOR_BGR2GRAY);
            Mat sobelx = new Mat();
            Imgproc.Sobel(grey, sobelx, CvType.CV_32F, 1, 0);
            Core.MinMaxLocResult res = Core.minMaxLoc(sobelx); // find minimum and maximum intensities
            Mat draw = new Mat();
            double maxVal = res.maxVal, minVal = res.minVal;
            maxVal = 127;
            sobelx.convertTo(draw, CvType.CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
            return  draw;
        }

        private Mat findContours(Mat src)
        {
            Imgproc.cvtColor(src, src, Imgproc.COLOR_BGR2HSV);
            Mat dest = new Mat();
            // Mat dest = new Mat(src.width(), src.height(), src.type());
            Core.inRange(src, new Scalar(85, 127, 127),
                    new Scalar(115, 255, 255), dest);

            Mat erode = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
            Mat dilate = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));
            Imgproc.erode(dest, dest, erode);
            Imgproc.erode(dest, dest, erode);

            Imgproc.dilate(dest, dest, dilate);
            Imgproc.dilate(dest, dest, dilate);

            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(dest, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(dest, contours, -1, new Scalar(255,255,0));

            return  dest;
        }


        /**
         * Perform the operations needed for removing a uniform background
         *
         * @param frame
         *            the current frame
         * @return an image with only foreground objects
         */
        private Mat doBackgroundRemoval(Mat frame)
        {
            // init
            Mat hsvImg = new Mat();
            List<Mat> hsvPlanes = new ArrayList<>();
            Mat thresholdImg = new Mat();

            // threshold the image with the histogram average value
            hsvImg.create(frame.size(), CvType.CV_8U);
            Imgproc.cvtColor(frame, hsvImg, Imgproc.COLOR_BGR2HSV);
            Core.split(hsvImg, hsvPlanes);

            double threshValue = this.getHistAverage(hsvImg, hsvPlanes.get(0));

            boolean inverse = false;

            if (inverse)
                Imgproc.threshold(hsvPlanes.get(0), thresholdImg, threshValue, 179.0, Imgproc.THRESH_BINARY_INV);
            else
                Imgproc.threshold(hsvPlanes.get(0), thresholdImg, threshValue, 179.0, Imgproc.THRESH_BINARY);

            Imgproc.blur(thresholdImg, thresholdImg, new Size(5, 5));

            // dilate to fill gaps, erode to smooth edges
            Imgproc.dilate(thresholdImg, thresholdImg, new Mat(), new Point(-1, 1), 6);
            Imgproc.erode(thresholdImg, thresholdImg, new Mat(), new Point(-1, 1), 6);

            Imgproc.threshold(thresholdImg, thresholdImg, threshValue, 179.0, Imgproc.THRESH_BINARY);

            // create the new image
            Mat foreground = new Mat(frame.size(), CvType.CV_8UC3, new Scalar(255, 255, 255));
            frame.copyTo(foreground, thresholdImg);

            return foreground;
        }

        /**
         * Get the average value of the histogram representing the image Hue
         * component
         *
         * @param hsvImg
         *            the current frame in HSV
         * @param hueValues
         *            the Hue component of the current frame
         * @return the average value
         */
        private double getHistAverage(Mat hsvImg, Mat hueValues)
        {
            // init
            double average = 0.0;
            Mat hist_hue = new Mat();
            MatOfInt histSize = new MatOfInt(180);
            List<Mat> hue = new ArrayList<>();
            hue.add(hueValues);

            // compute the histogram
            Imgproc.calcHist(hue, new MatOfInt(0), new Mat(), hist_hue, histSize, new MatOfFloat(0, 179));

            // get the average for each bin
            for (int h = 0; h < 180; h++)
            {
                average += (hist_hue.get(h, 0)[0] * h);
            }

            return average = average / hsvImg.size().height / hsvImg.size().width;
        }

    }
}