package org.firstinspires.ftc.teamcode;

public class BasicAutonEx extends BasicAuton {
    public CompetitionHardwareEx robot;

    // all below dimensions are in Inches
    protected static final double FRONT_ARM_OFFSET = 5; //distance from from ARM to Front sensor
    protected static final double STONE_LENGTH = 8; // lenght of skystone
    protected static final double FIRST_STONE_DROP = 105;
    protected static final double FIELD_WIDTH = 46+46+45;
    protected static final double ROBOT_LENGTH = 17;
    protected static final double ROBOT_WIDTH = 17.5;
    protected static final double DROP_POSITION = 27.5;

    @Override
    public void initialize() {
        telemetry.addData("Status", "Initialize BasicAutonEx " + allianceColor);
        telemetry.update();

        robot = new CompetitionHardwareEx();
        robot.init(hardwareMap, true, false, true);
        super.robot = robot;

        setChoiceOfArm();

        initVuforia();
    }

    public void pickUpSkyStone(boolean delay) {

        choiceOfArm.latchStone(0.9);
        choiceOfArm.goDown(0.9);

        sleep(1000);
        if(delay)
            sleep(500);

        //sleep(1000);

        choiceOfArm.liftUp(0.9);
        sleep(1000);

    }
}
