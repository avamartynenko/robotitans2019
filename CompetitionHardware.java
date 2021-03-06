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

import android.graphics.Color;
import android.support.annotation.Nullable;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static java.lang.Thread.sleep;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class CompetitionHardware
{
    /* Public OpMode members. */
    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    // oritnal values (p=1.169998 i=0.117004 d=0.000000 f=11.699997 alg=PIDF)
    public static final double NEW_P = 1.5;
    public static final double NEW_I = 0.117;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 11.699997;

    public ColorSensor colorsense = null;
    private boolean hasHook = true;
    private boolean hasArm = true;
    private boolean hasFrogTongue = true;
    public boolean activateSpeedProfile = false;
    public Hooks hookLatch = null;
    //public ServoHooks hookLatch = null;
    public IntakeMech intakeMech = null;
    public LiftMech liftMech = null;
    public Arm frontArm = null;
    public Arm backArm = null;
    public FrogTongue frogTongue = null;


    public static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.66 ;     // This is < 1.0 if geared UP
    public final double     COUNTS_PER_MOTOR_REV_WITH_GEAR    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ;     // For figuring circumference
    public final double     WHEEL_CIRCUMFRENCE      = WHEEL_DIAMETER_INCHES * 3.1415;

    // defines if robot stops motors in the end of the move
    public boolean linkMoves = false;

    public final double  MECHANUM_CPI_RATIO  = 0.5; //since it is a 4 wheel drive, you have to half the CPI
    public  final double   COUNTS_PER_INCH   = (COUNTS_PER_MOTOR_REV_WITH_GEAR/ WHEEL_CIRCUMFRENCE) * MECHANUM_CPI_RATIO;

    public enum Direction {
        FORWARD,
        REVERSE,
        RIGHT,
        LEFT,
        GYRO_RIGHT,
        GYRO_LEFT,
        DIAGONAL_RIGHT_UP,
        DIAGONAL_RIGHT_DOWN,
        DIAGONAL_LEFT_UP,
        DIAGONAL_LEFT_DOWN
    }

    public final double MAX_SPEED = 1;          // defines max robot speed. set to 1 for normal operation, .5 for debugging to observer robot moves in slow motion
    public final double TURN_ERROR = 43;        // Robot over rotates by approximaterly 45 degrees if directed to turn 90 degrees at full speed
    public final double NO_ERROR_SPEED = .2;   // max speed at which robot turns correctly, robot may not move any any speed less than .14
    public final double STOP_GAP = 2;           // even at smallest possible speed there is still lag in processing wich results in overrun in case if power is killed once the angle is reach
                                                // we need to kill power 3 degrees ahead of target angle to achive accurate turn angle
                                                // STPO_GAP is dependent on turn speed and robot configuration and will need to be reset in case if robot configuraiton is changed
    public final long OVERRAN_SLEEP_TIME = 50;  // how long robot should sleep for in uncontrolled spin before checking if it is stabilized
    public final double PRECISION_TURN_ZONE = TURN_ERROR/2;    // skip fast turn, turn at slowest speed instead

    public final double STOP_THRESHOLD = .25;   // min XY accell below which we consider that robot is stationary

    // variables for using gyro
    protected boolean gyroInitialized = false;
    Orientation angles;
    public BNO055IMU        imu;
    protected Orientation   lastAngles = new Orientation();
    protected double        globalAngle, rotation;
    private PIDController   pidRotate, pidDrive;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public CompetitionHardware(){
        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hMap, boolean needEncoder,boolean needColorSensor, boolean needGyro) {
        this.hwMap=hMap;
        // Define and Initialize Motors
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");

        // Fix for 5.3 firmware
        changePIDCoefficients(RUN_USING_ENCODER, 10, 3, 0);
        changePIDCoefficients(RUN_TO_POSITION , 10, 0.05, 0);

        if (hasHook) hookLatch = new Hooks(hwMap);
        if (hasArm) frontArm = new Arm(hwMap,Arm.FRONT_ARM);
        if (hasArm) backArm = new Arm(hwMap, Arm.BACK_ARM);
        if (hasFrogTongue) frogTongue = new FrogTongue(hwMap);


        //Sets direction of motors
        backLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        setPower4WDrive(0);

        setEncoder(needEncoder);

        if(needColorSensor)
            enableColorSensor();

        if(needGyro)
            enableGyro();

        /// Keep the attachments to home position
        hookLatch.release();
       // frontArm.moveToHomePosition();
       // backArm.moveToHomePosition();

    }

    public void initTeleopModules() {
        intakeMech = new IntakeMech(hwMap);
        liftMech = new LiftMech(hwMap);
    }

    public void setEncoder(boolean needEncoder){

        if (!needEncoder) {
            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.

            backLeft.setMode(RUN_WITHOUT_ENCODER);
            frontLeft.setMode(RUN_WITHOUT_ENCODER);
            frontRight.setMode(RUN_WITHOUT_ENCODER);
            backRight.setMode(RUN_WITHOUT_ENCODER);

        }
        else {
            /// Stop and Reset ENCODER
            backLeft.setMode(STOP_AND_RESET_ENCODER);
            frontLeft.setMode(STOP_AND_RESET_ENCODER);
            frontRight.setMode(STOP_AND_RESET_ENCODER);
            backRight.setMode(STOP_AND_RESET_ENCODER);

            //RUN USING ENCODER
            backLeft.setMode(RUN_USING_ENCODER);
            frontLeft.setMode(RUN_USING_ENCODER);
            frontRight.setMode(RUN_USING_ENCODER);
            backRight.setMode(RUN_USING_ENCODER);

            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void enableGyro (){
        if(!gyroInitialized) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = getREVName() + "_AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            while(!imu.isGyroCalibrated()){
                try {
                    sleep(50);
                }
                catch (Exception e) {
                    Log.e("IMU", "Calibration terminated");
                }
            }

            gyroInitialized = true;
        }

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
        //imu.stopAccelerationIntegration();
    }

    public void enableColorSensor(){
        // Gets Sensors
        colorsense = hwMap.get(ColorSensor.class, "color");
        colorsense.enableLed(true);
    }

    public int linearMove (Direction direction, double speed, double distance) {
        return linearMove(direction, speed, distance, null);
    }

    public int linearMove (Direction direction, double speed, double distance, @Nullable LinearOpMode opMode) {
        int newBleftTarget;
        int newBrightTarget;
        int newFleftTarget;
        int newFrightTarget;

        // encoder position does not get reset between program starts
        // motors can start in random positions if robot was not restarted after completion of previous program
        if(activateSpeedProfile) {
            setEncoder(true);
            //setMotorsMode(RUN_TO_POSITION);
            setZeroPowerMode(BRAKE);
        }

        // Determine new target position, and pass to motor controller
        newBleftTarget = backLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newBrightTarget = backRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newFleftTarget = frontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newFrightTarget = frontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        if(opMode != null) {
            opMode.telemetry.log().add("Linear move direction: " + direction2string(direction) + ". Distance: " + distance + " inches");
            //opMode.telemetry.log().add("Speed profile " + (activateSpeedProfile ? "ON" : "OFF"));
        }

        setDirection(direction);

/*        backLeft.setTargetPosition(newBleftTarget);
        backRight.setTargetPosition(newBrightTarget);
        frontLeft.setTargetPosition(newFleftTarget);
        frontRight.setTargetPosition(newFrightTarget);*/

        // Turn On RUN_TO_POSITION
/*        backLeft.setMode(RUN_TO_POSITION);
        backRight.setMode(RUN_TO_POSITION);
        frontLeft.setMode(RUN_TO_POSITION);
        frontRight.setMode(RUN_TO_POSITION);*/

        if (!activateSpeedProfile) {
            //Setting the power
            setPower4WDrive(Math.abs(speed));

            while (
                newBleftTarget > backLeft.getCurrentPosition() &&
                newBrightTarget > backRight.getCurrentPosition() &&
                newFleftTarget > frontLeft.getCurrentPosition() &&
                newFrightTarget > frontRight.getCurrentPosition()
            )
            {
                // just waiting when motors are busy
                 // Thread.yield();
            }
        } else {
            set4WDriveWithSpeedProfile(direction, Math.abs(speed), newBleftTarget, backLeft, opMode);
        }



        // Stop all motion - TODO test if it actually works - encoder run to position might actually stop motors
        if(!linkMoves) {
            setZeroPowerMode(BRAKE);
            setPower4WDrive(0);
            setEncoder(true);
        }
        else
            setZeroPowerMode(DcMotor.ZeroPowerBehavior.FLOAT);

        return newBleftTarget;
    }

    public void set4WDriveWithSpeedProfile(Direction direction, double speed, int target, DcMotor motor, LinearOpMode opMode) {
        boolean original_logic = false;

        if(original_logic) {
            double profileSpeed = 0.0;
            double speedIncrement = 0.007;
            int curPostion = motor.getCurrentPosition();

            while (curPostion <= target) {

                if (curPostion < target / 2) {
                    profileSpeed = profileSpeed + speedIncrement;
                } else {
                    profileSpeed = profileSpeed - speedIncrement;
                }

                if (profileSpeed > speed) profileSpeed = speed;
                if (profileSpeed < 0.0) profileSpeed = 0.0;

                setPower4WDrive(profileSpeed);
                curPostion = motor.getCurrentPosition();
            }
        }
        else {
            double accelerationTime = 350;
            double breakDistance = 0;
            double currentSpeed = 0;
            double minBreakSpeed = 0;
            double correctedTarget = target;
            double posIncrement = 0;
            double initialHeading = getAbsoluteHeading();

            // set min break speed and break distance
            switch (direction) {
                case FORWARD:
                case REVERSE:
                    minBreakSpeed = .25;
                    breakDistance = 450;
                    break;
                case LEFT:
                case RIGHT:
                    minBreakSpeed = .25;
                    breakDistance = 250;
                    break;
            }

            // set postion increment for straif
            switch (direction) {
                case LEFT:
                    posIncrement = (backLeft.getTargetPosition() - backLeft.getCurrentPosition())*.40;
                    correctedTarget += posIncrement;
                    break;
                case RIGHT:
                    posIncrement = (backLeft.getTargetPosition() - backLeft.getCurrentPosition())*.40;
                    correctedTarget += posIncrement;
                    break;
                default:
                   break;
            }

            backLeft.setTargetPosition(backLeft.getTargetPosition() + (int)posIncrement);
            backRight.setTargetPosition(backRight.getTargetPosition() + (int)posIncrement);
            frontLeft.setTargetPosition(frontLeft.getTargetPosition() + (int)posIncrement);
            frontRight.setTargetPosition(frontRight.getTargetPosition() + (int)posIncrement);

            resetAngle();
            PIDController pidDrive = new PIDController(.20, 0, 0);
            // Set up parameters for driving in a straight line.

            pidDrive.setSetpoint(getAngle());
            pidDrive.setOutputRange(0, speed);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();

            double correction = 0;

            if(opMode != null && false) {
                opMode.telemetry.log().add("Current Positions bl " + backLeft.getCurrentPosition() + " br " + backRight.getCurrentPosition() + " fl " + frontLeft.getCurrentPosition() + " fr " + frontRight.getCurrentPosition());
                opMode.telemetry.log().add("Heading " + getActualHeading() + ". Angle: " + getAngle());
                opMode.telemetry.log().add("Target original/corrected " + target + "/" + correctedTarget);
            }

            int curPostion = motor.getCurrentPosition();
            ElapsedTime et = new ElapsedTime();
            while (curPostion <= Math.max(correctedTarget - breakDistance, correctedTarget/2) && opMode.opModeIsActive()) {
                // Use PID with imu input to drive in a straight line.
                correction = 0;

                curPostion = motor.getCurrentPosition();
                currentSpeed = Range.clip(Math.min(accelerationTime, et.milliseconds()) / accelerationTime, .20, 1);

                if(et.milliseconds() >= accelerationTime)
                    correction = pidDrive.performPID(getAngle());

                if(direction == Direction.REVERSE || direction == Direction.RIGHT)
                    correction *= -1;

                setPower4WDrive(  Range.clip(currentSpeed - correction, -1, 1),
                        Range.clip(currentSpeed + correction, -1, 1),
                        Range.clip(currentSpeed - correction, -1, 1),
                        Range.clip(currentSpeed + correction, -1, 1));
            }

            if(opMode != null && false) {
                opMode.telemetry.log().add("Final correction " + correction);
                opMode.telemetry.log().add("Pre break heading " + getActualHeading());
                opMode.telemetry.log().add("Prebreak position bl " + backLeft.getCurrentPosition() + " br " + backRight.getCurrentPosition() + " fl " + frontLeft.getCurrentPosition() + " fr " + frontRight.getCurrentPosition());
            }

            double maxTravelSpeed = currentSpeed; // capture current speed

            // stop braking as soon as at least one motor will reach target position
            while (!oneMotorAtTarget() && opMode.opModeIsActive()) {
                double slowDownCorrection = 0;
                // Use PID with imu input to drive in a straight line.
                currentSpeed = Math.max((correctedTarget-curPostion)/breakDistance, minBreakSpeed);

                if(direction == Direction.FORWARD || direction == Direction.REVERSE)
                    slowDownCorrection = pidDrive.performPID(getAngle());
                else
                    slowDownCorrection = correction*currentSpeed;

                curPostion = motor.getCurrentPosition();
                if(direction == Direction.REVERSE || direction == Direction.RIGHT)
                    slowDownCorrection *= -1;

                setPower4WDrive(  Range.clip(currentSpeed - slowDownCorrection, -1, 1),
                        Range.clip(currentSpeed + slowDownCorrection, -1, 1),
                        Range.clip(currentSpeed - slowDownCorrection, -1, 1),
                        Range.clip(currentSpeed + slowDownCorrection, -1, 1));
            }

            // stop
            setPower4WDrive(0);
            waitToStop();

            double correctionAngle = calcTurnAngleD(initialHeading, getAbsoluteHeading());

            if(opMode != null && false) {
                opMode.telemetry.log().add("Positions bl " + backLeft.getCurrentPosition() + " br " + backRight.getCurrentPosition() + " fl " + frontLeft.getCurrentPosition() + " fr " + frontRight.getCurrentPosition());
                opMode.telemetry.log().add("Final heading/correction angle " + getActualHeading() + "/" + correctionAngle);
            }

            correctHeading(initialHeading, opMode);
        }

        ElapsedTime et = new ElapsedTime();
        setPower4WDrive(0);
        setEncoder(true);
    }

    protected boolean oneMotorAtTarget() {
        return  backLeft.getCurrentPosition() >= backLeft.getTargetPosition() ||
                backRight.getCurrentPosition() >= backRight.getTargetPosition() ||
                frontLeft.getCurrentPosition() >= frontLeft.getTargetPosition() ||
                frontRight.getCurrentPosition() >= frontRight.getTargetPosition();
    }


    protected boolean allMotorsAtTarget() {
        return  backLeft.getCurrentPosition() >= backLeft.getTargetPosition() &&
                backRight.getCurrentPosition() >= backRight.getTargetPosition() &&
                frontLeft.getCurrentPosition() >= frontLeft.getTargetPosition() &&
                frontRight.getCurrentPosition() >= frontRight.getTargetPosition();
    }

    public void correctHeading(double initialHeading, LinearOpMode opMode) {
        double correctionAngle = calcTurnAngleD(initialHeading, getActualHeading());
        opMode.telemetry.log().add("correctHeading correction Angle: " + correctionAngle);
        opMode.telemetry.update();
        if(correctionAngle != 0) {
            Direction correctionAngleDirection = (correctionAngle < 0) ? Direction.GYRO_LEFT : Direction.GYRO_RIGHT;
            gyroMoveByOffset(correctionAngleDirection, 1, Math.abs(correctionAngle), opMode);
        }
    }

    public void setPower4WDrive(double speed){
        setPower4WDrive(speed,speed,speed,speed);
    }

    public void gyroMoveByOffset(Direction direction, double speed, double degrees, LinearOpMode opMode) {
        if(degrees < STOP_GAP)  // robot has issue correcting very small angles like 0.06 degrees :)
            return;

        setDirection(direction);
        setEncoder(true);

        int positionIncrement = (int) (degrees * 1000 / 60); // 60 degress ~ 1000 steps
        //opMode.telemetry.log().add("Actual heading/tics/direction " + getActualHeading() + "/" + positionIncrement + "/" + direction);

        frontLeft.setTargetPosition(positionIncrement);
        frontRight.setTargetPosition(positionIncrement);
        backLeft.setTargetPosition(positionIncrement);
        backRight.setTargetPosition(positionIncrement);

        setMotorsMode(RUN_TO_POSITION);

        setPower4WDrive(speed);

        while (!oneMotorAtTarget()) {
            // just waiting when motors are busy
            try {
                Thread.sleep(10);
            }
            catch (InterruptedException ex) {
                opMode.telemetry.log().add("Failed in sleep");
            }
        }

        setZeroPowerMode(BRAKE);
        setMotorsMode(STOP_AND_RESET_ENCODER);
        setPower4WDrive(0);
        waitToStop();

        //opMode.telemetry.log().add("Final heading: " + getActualHeading());
    }

    // BleftDriveSpeed,  BrightDriveSpeed,  FleftDriveSpeed,  FrightDriveSpeed
    public void setPower4WDrive(double BleftDriveSpeed, double BrightDriveSpeed, double FleftDriveSpeed, double FrightDriveSpeed ){
        backLeft.setPower(BleftDriveSpeed);
        backRight.setPower(BrightDriveSpeed);
        frontLeft.setPower(FleftDriveSpeed);
        frontRight.setPower(FrightDriveSpeed);
    }

    /*
     we are setting what needs to be done(forward,backward,right,left)
     when we put in each thing(forward,backward,right,left) it will come here and look for that
     */
    public void setDirection(Direction direction){

        switch (direction){
            case FORWARD:
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case REVERSE:
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                break;

            case RIGHT:
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                break;

            case LEFT:
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE); // INSTEAD OF REVERSE WE WENT FORWARD
                break;
            case GYRO_LEFT:
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            case GYRO_RIGHT:
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                break;

            default:
                break;
        }
    }

    // WARNING: due to processing lag and robot inertia actual angle will usaully be different
    // from requested angle except for very small speeds
    // eg 90 degrees turn exected at full speed will result in approximately 45 degrees overrun
    // use gyroMove2 in case if precision is required
    public void gyroMove(Direction direction,double speed, double angle) {

        enableGyro();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        setDirection(direction);

        //Setting the power
        while(getAbsoluteHeading() < angle) {
            setPower4WDrive(Math.abs(speed));
        }
        setPower4WDrive(0.0);

        imu.stopAccelerationIntegration();
    }

    // DO NOT USE TO ROTATE THE PLATFORM!!!
    public void gyroMove90(Direction direction, Telemetry telemetry) {
        gyroMove2(direction, 90, telemetry);
    }

    // DO NOT USE TO ROTATE THE PLATFORM!!!
    public void gyroMove2(Direction direction, double angle, double speed, Telemetry telemetry) {
        double startAngle = getActualHeading();
        telemetry.addLine("Gyro turn " + direction2string(direction) + " for " + angle + " degrees");
        telemetry.addData("Start heading: ", "%.1f", startAngle);

        if(!(direction == Direction.GYRO_LEFT || direction == Direction.GYRO_RIGHT))
            throw new IllegalArgumentException("Direction should be either GYRO_LEFT or GYRO_RIGHT");

        enableGyro();
        setDirection(direction);

        double currentTurnAngle;

        // sckip fast turn zone if requested angle inside of precision turn zone
        if(angle > PRECISION_TURN_ZONE) {
            setPower4WDrive(MAX_SPEED);

            do {
                currentTurnAngle = calcTurnAngleD(startAngle, getActualHeading());
            }
            while (currentTurnAngle < (angle - TURN_ERROR));
            setPower4WDrive(0.0);

            telemetry.addData("Uncorrected heading, currentTurnAngle: ", "%.1f %.1f", getAbsoluteHeading(), currentTurnAngle);
            //wait for the robot to stop spinning after power is cut off
            double spinAngle;
            do {
                spinAngle = (int) getActualHeading();
                try {
                    sleep(OVERRAN_SLEEP_TIME);
                } catch (Exception e) {
                    Log.i("IMU", "Rotation spin stabilization failure");
                }
            }
            while (spinAngle != (int) getActualHeading());
        }

        currentTurnAngle = calcTurnAngleD(startAngle, getActualHeading());
        setPower4WDrive(NO_ERROR_SPEED * Math.signum(angle - currentTurnAngle));
        double correctionAngle = Math.abs(angle - currentTurnAngle);
        double correctionStart = getActualHeading();

        telemetry.addData("Heading stabilized, correction angle: ", "%.1f %.1f", getAbsoluteHeading(), correctionAngle);
        do {
            currentTurnAngle = calcTurnAngleD(correctionStart, getActualHeading());
        }
        while (currentTurnAngle < (correctionAngle - STOP_GAP));
        setPower4WDrive(0.0);

        telemetry.addData("Corrected heading, completed turn: ", "%.1f %.1f", getAbsoluteHeading(), currentTurnAngle);
    }

    // DO NOT USE TO ROTATE THE PLATFORM!!!
    public void gyroMove2(Direction direction, double angle, Telemetry telemetry) {
        gyroMove2(direction, angle, MAX_SPEED, telemetry);
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void gyroMovePID(int degrees, double power, LinearOpMode opMode)
    {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        setDirection(Direction.FORWARD);
        setZeroPowerMode(BRAKE);

        // rotate until turn is completed.
        try {
            if (degrees < 0) {
                // On right turn we have to get off zero first.
                while (opMode.opModeIsActive() && getAngle() == 0) {
                    setPower4WDrive(power, -power, power, -power);
                    sleep(100);
                }

                do {
                    power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                    setPower4WDrive(-power, power, -power, power);
                } while (opMode.opModeIsActive() && !pidRotate.onTarget());
            } else    // left turn.
                do {
                    power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                    setPower4WDrive(-power, power, -power, power);
                } while (opMode.opModeIsActive() && !pidRotate.onTarget());

            // turn the motors off.
            setPower4WDrive(0);

            rotation = getAngle();

            // wait for rotation to stop.
            sleep(500);
        }
        catch (Exception e) {
            Log.e("IMU", "Failed to complete turn");
        }

        // reset angle tracking on new heading.
        resetAngle();
    }

    /**
     * Calculate _absolute_ angle values between two directions in degrees
     * @param startAngle the initial heading
     * @param currentAngle current heading
     * @return different between initial and current heading
     */
    protected double calcTurnAngleD(double startAngle, double currentAngle) {

        double turnAngle = Math.abs(currentAngle - startAngle);
        turnAngle = (turnAngle > 360) ? turnAngle - 360 : turnAngle;
        turnAngle = (turnAngle > 180) ? 360 - turnAngle : turnAngle;

        return turnAngle;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    protected void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public int diagonalMove (Direction direction,double speed, double distance) {
        int wheel1Target;
        int wheel2Target;
        DcMotor motor1 = null;
        DcMotor motor2 = null;

        switch (direction){
            case DIAGONAL_RIGHT_UP:
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                motor1 = frontLeft;
                motor2 = backRight;
                break;
            case DIAGONAL_RIGHT_DOWN:
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                motor1 = frontLeft;
                motor2 = backRight;
                break;
            case DIAGONAL_LEFT_DOWN:
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                motor1 = backLeft;
                motor2 = frontRight;
                break;
            case DIAGONAL_LEFT_UP:
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                motor1 = backLeft;
                motor2 = frontRight;
                break;

            default:
                break;
        }

        // Determine new target position, and pass to motor controller
        wheel1Target = motor1.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        wheel2Target = motor2.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        motor1.setTargetPosition(wheel1Target);
        motor2.setTargetPosition(wheel2Target);

        // Turn On RUN_TO_POSITION
        motor1.setMode(RUN_TO_POSITION);
        motor2.setMode(RUN_TO_POSITION);

        //Setting the power
        motor1.setPower(speed);
        motor2.setPower(speed);

        while (motor1.isBusy() && motor2.isBusy()) {
            // just waiting when motors are busy
            Thread.yield();
        }

        // Stop all motion
        motor1.setPower(0);
        motor2.setPower(0);

        setEncoder(true);

        return wheel1Target;
    }

    //GYRO TURNING
    //this will always give postitive angle
    public double getAbsoluteHeading() {
        return Math.abs(getActualHeading());
    }

    //GYRO TURNING
    //this will give the actual angle(+/-)
    public double getActualHeading() {
        return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public boolean isItGold() {
        // enabling colorSensor
        //enableColorSensor();

        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;
        Color.RGBToHSV((int) (colorsense.red() * SCALE_FACTOR), (int) (colorsense.green() * SCALE_FACTOR), (int) (colorsense.blue() * SCALE_FACTOR), hsvValues);

        return (hsvValues[0] > 20 && hsvValues[0] < 120); // Detected a non-white color. (Probably yellow lol)
    }

    public int linearMoveOne (DcMotor testMotor, int direction,double speed, double distance) {

        int testTarget;
        testMotor.setMode(RUN_WITHOUT_ENCODER);

        // Determine new target position, and pass to motor controller
        testTarget = testMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        testMotor.setTargetPosition(testTarget);

        // Turn On RUN_TO_POSITION
        testMotor.setMode(RUN_TO_POSITION);

        if (!activateSpeedProfile) {

            //Setting the power
            testMotor.setPower(Math.abs(speed));

            while (testMotor.isBusy() ) {
                // just waiting when motors are busy
                Thread.yield();
            }
        }

        // Stop all motion
        testMotor.setPower(0.0);
        //setEncoder(true);

        return testTarget;
    }

    public void setZeroPowerMode(DcMotor.ZeroPowerBehavior behavior) {
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
    }

    public void setMotorsMode(RunMode mode) {
        frontRight.setMode(mode);
        frontLeft.setMode(mode);
        backRight.setMode(mode);
        backLeft.setMode(mode);
    }

    protected String direction2string(Direction direction) {
        String result = "UNKNOWN";

        switch (direction)
        {
            case LEFT:
                result = "LEFT";
                break;
            case RIGHT:
                result = "RIGHT";
                break;
            case REVERSE:
                result = "REVERSE";
                break;
            case FORWARD:
                result = "FORWAR";
                break;
            case GYRO_LEFT:
                result = "GYRO-LEFT";
                break;
            case GYRO_RIGHT:
                result = "GYRO-RIGHT";
                break;

                default:
                    break;
        }

        return result;
    }

    protected String getREVName() {
        String sResult = "";

        for(Object o: hwMap) {
            HardwareDevice dev = (HardwareDevice)o;
            if(dev.getDeviceName().contains("REV Expansion Hub IMU")) {
                sResult = dev.getConnectionInfo().split(";")[0].split(" ")[1];
                break;
            }
        }

        return sResult;
    }

    protected void waitToStop() {
        while (Math.sqrt(Math.pow(imu.getLinearAcceleration().xAccel, 2) + Math.pow(imu.getLinearAcceleration().yAccel,2)) > STOP_THRESHOLD) {
            Thread.yield();
        }
    }

    // PID coefficients section

    protected void changePIDCoefficients(RunMode runMode, double NEW_P, double NEW_I, double NEW_D) {
        changePIDCoefficients(frontLeft, runMode, NEW_P, NEW_I, NEW_D);
        changePIDCoefficients(frontRight, runMode, NEW_P, NEW_I, NEW_D);
        changePIDCoefficients(backLeft, runMode, NEW_P, NEW_I, NEW_D);
        changePIDCoefficients(backRight, runMode, NEW_P, NEW_I, NEW_D);
    }

    protected void changePIDCoefficients(DcMotor dcMotor, RunMode runMode, double NEW_P, double NEW_I, double NEW_D) {
        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx motorControllerEx = (DcMotorControllerEx)dcMotor.getController();

        // get the port number of our configured motor.
        int motorIndex = ((DcMotorEx)dcMotor).getPortNumber();

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        //PIDCoefficients pidOrig = motorControllerEx.getPIDCoefficients(motorIndex, runMode);

        // change coefficients.
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        motorControllerEx.setPIDCoefficients(motorIndex, runMode, pidNew);
    }

    // End PID coefficients section

    // helper class to track Motor postions
    private class PositionTracker {
        int backLeftStart, backRightStart, frontLeftStart, frontRightStart;

        public PositionTracker() {
            reset();
        }

        public void reset() {
            setMotorsMode(RUN_WITHOUT_ENCODER);

            backLeftStart = backLeft.getCurrentPosition();
            backRightStart = backRight.getCurrentPosition();
            frontLeftStart = frontLeft.getCurrentPosition();
            frontRightStart = frontRight.getCurrentPosition();
        }

        public int getMaxGain() {
            return Math.max(Math.max(backLeft.getCurrentPosition() - backLeftStart, backRight.getCurrentPosition() - backRightStart),
                    Math.max(frontLeft.getCurrentPosition() - frontLeftStart, frontRight.getCurrentPosition() - frontRightStart));
        }

        public int getMinGain() {
            return Math.min(Math.min(backLeft.getCurrentPosition() - backLeftStart, backRight.getCurrentPosition() - backRightStart),
                    Math.min(frontLeft.getCurrentPosition() - frontLeftStart, frontRight.getCurrentPosition() - frontRightStart));
        }
    }
}
