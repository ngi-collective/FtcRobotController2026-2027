package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;

@Autonomous(name = "Example Auto", group = "Examples")
public class TestAuto extends LinearOpMode {

    private Follower follower;
    private final Pose startPose = new Pose(22, 122, Math.toRadians(324)); // Start Pose of our robot. This is against the goal facing AWAY
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(135)); // Scoring Pose of our robot.
    private final Pose pickup1Pose = new Pose(17, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(12, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(12, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose endPose = new Pose (60, 105); // Final Pose of our robot, off the starting line
    //defining our PathChains
    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, leave;

    public void buildPaths() {

    }
    public Command autoRoutine() {
        return sequential(
                /* Go To Score Command*/

        );
    }
    @Override
    public void runOpMode() {
        //These will run when the OpMode is initiated
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();
        //We schedule all our commands when we start the OpMode
        schedule(autoRoutine());
        while (opModeIsActive()) {
            //Update the follower and execute the scheduler every loop
            follower.update();
            Scheduler.execute();

            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }}