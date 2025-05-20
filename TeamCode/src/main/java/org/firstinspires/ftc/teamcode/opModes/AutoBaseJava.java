package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstantsBasket;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstantsBasket;
import org.firstinspires.ftc.teamcode.subsystems.MegiddoOpModeAuto;
import org.firstinspires.ftc.teamcode.subsystems.followerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.linearSlides;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;

public class AutoBaseJava extends MegiddoOpMode {
    private Telemetry telemetryA;

    public enum Side{
        basket,
        chamber
    }
    public Pose startingPoseChamber = new Pose(-62, -8.5, Math.toRadians(180));
    public Pose startingPoseBasket = new Pose(-62, 8.5+24, 0);
    Side side;
    /**
     * Constructs an AutoBaseJava instance with the specified starting side.
     *
     * @param side the starting side of the robot, either basket or chamber
     */
    public AutoBaseJava(Side side){
        this.side = side;
    }
    protected Follower follower;
    /**
     * Initializes telemetry and the path follower, setting the follower's starting pose based on the selected side.
     *
     * This method prepares the autonomous mode by configuring combined telemetry output and instantiating the follower with hardware and constant classes. The follower's pose is set to either the basket or chamber starting pose, depending on the configured side.
     */
    @Override
    public void preInit(){
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        switch(side){
            case basket:
                follower.poseUpdater.setPose(startingPoseBasket);
//                follower.poseUpdater.setCurrentPoseWithOffset(startingPoseBasket);
//                follower = new Follower(hardwareMap);
                break;
            case chamber:
//                Constants.setConstants(FConstants.class, LConstants.class);
                follower.poseUpdater.setPose(startingPoseChamber);
//                follower.poseUpdater.setCurrentPoseWithOffset(startingPoseChamber);

        }
    }
    /**
     * Updates the starting poses of the follower and linear slides subsystems to their current poses.
     *
     * This method is typically called to record the robot's current state as the new starting reference for subsequent operations.
     */
    @Override
    public void myFullStop(){
        followerSubsystem.setStartingPose(follower.getPose());
        linearSlides.setStartingPose(linearSlides.getPose());
    }
    boolean flag = false;
    public Lambda runFollower = new Lambda("update Follower")
            .setFinish(()->false)
            .setInit(()->{
                switch(side){
                    case basket:
                        follower.poseUpdater.setPose(startingPoseBasket);
//                follower.poseUpdater.setCurrentPoseWithOffset(startingPoseBasket);
//                follower = new Follower(hardwareMap);
                        break;
                    case chamber:
//                Constants.setConstants(FConstants.class, LConstants.class);
                        follower.poseUpdater.setPose(startingPoseChamber);
//                follower.poseUpdater.setCurrentPoseWithOffset(startingPoseChamber);

                }            })
            .setExecute(()->{
//                if (!flag){
//                    follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//                    switch(side){
//                        case basket:
//                            follower.setCurrentPoseWithOffset(startingPoseBasket);
////                follower = new Follower(hardwareMap);
//                            break;
//                        case chamber:
////                Constants.setConstants(FConstants.class, LConstants.class);
//                            follower.setCurrentPoseWithOffset(startingPoseChamber);
//                    }
//                }
//                flag = true;
                follower.update();
                follower.telemetryDebug(telemetryA);
            });
    public Lambda finishAuto = new Lambda("finishAuto")
        .setInit(()-> {
            follower.breakFollowing();
            follower.update();
            linearSlides.setStartingPose(linearSlides.getPose());

        });


    /**
     * Creates a Lambda command that starts following the specified path chain using the follower and schedules continuous path updates.
     *
     * The command finishes when the follower is no longer busy or the robot is detected as stuck. If interrupted, it stops the follower's path following.
     *
     * @param chain the path chain to follow
     * @return a Lambda command for executing the path following routine
     */
    public Lambda followPath(PathChain chain){
        return new Lambda("follow-path-chain")
            .setInit(() -> {
                follower.followPath(chain, true);
//                if (!Mercurial.isScheduled(runFollower)){
//                    follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
                    runFollower.schedule();
//                }
            })
//            .setExecute(() ->{
//
//
//            })
            .setFinish(()->  (!follower.isBusy() || follower.isRobotStuck()))
            .setEnd((interrupted) -> {
                if (interrupted) follower.breakFollowing();
            });
    }

    /**
     in radians
     **/
//    public void turnTo(double degrees) { // if you want to turn right, use negative degrees
//        Pose temp = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(degrees));
//        follower.holdPoint(temp);
/**
     * Creates a Lambda command that turns the robot to an absolute heading in degrees.
     *
     * The command initializes the turn, continuously updates the follower and telemetry during execution,
     * and finishes when the turn is complete or the robot is detected as stuck. If interrupted, it stops the follower.
     *
     * @param angle the absolute heading in degrees to turn to
     * @return a Lambda command for executing the turn
     */
    public Lambda turnTo(double angle){
        return new Lambda("follow-path-chain")
                .setInit(() -> follower.turnToDegrees(angle))
                .setExecute(() ->{
                    follower.update();
                    follower.telemetryDebug(telemetryA);

                })
                .setFinish(()-> ((!follower.isBusy() ) || follower.isRobotStuck()))
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }
    /**
     * Creates a Lambda command to turn the robot by a specified number of degrees relative to its current heading.
     *
     * The command initializes the turn, continuously updates the follower and telemetry during execution,
     * and finishes when the turn is complete or the robot is detected as stuck. If interrupted, it stops the follower.
     *
     * @param degrees the relative angle in degrees to turn the robot
     * @return a Lambda command that performs the relative turn
     */
    public Lambda turn(double degrees){
        return new Lambda("follow-path-chain")
                .setInit(() -> follower.turnDegrees(degrees, false))
                .setExecute(() ->{
                    follower.update();
                    follower.telemetryDebug(telemetryA);

                })
                .setFinish(()-> ((!follower.isBusy() && !follower.isTurning()) || follower.isRobotStuck()))
                .setEnd((interrupted) -> {
                    if (interrupted) follower.breakFollowing();
                });
    }
    static PathChain slowPath;
    public Lambda slowX = new Lambda("slowF")
            .setEnd((interrupted)-> {
                follower.breakFollowing();
                follower.setMaxPower(1.0);
            })
            .setInit(()-> {
                follower.setMaxPower(1);
                follower.followPath(makeLinePath(follower.getPose(),
                 new Pose(follower.getPose().getX()-50, follower.getPose().getY(),
                0)));
            })
            .setExecute(() ->{
                follower.update();
                follower.telemetryDebug(telemetryA);

            })
            .setFinish(()-> ((!follower.isBusy() ) || follower.isRobotStuck()));

    /**
     * Creates a linear path between two poses with linear heading interpolation.
     *
     * @param startingPose the initial pose of the path
     * @param endingPose the final pose of the path
     * @return a PathChain representing a straight-line trajectory with heading smoothly interpolated from start to end
     */
    public PathChain makeLinePath(Pose startingPose, Pose endingPose){
        return new PathBuilder().addPath(
            new BezierLine(new Point(startingPose), new Point(endingPose)))
                .setLinearHeadingInterpolation(startingPose.getHeading(), endingPose.getHeading())
                .build();
    }
    /**
     * Creates a two-segment path from a starting pose to an ending pose, with a heading that interpolates linearly to the midpoint and remains constant for the second half.
     *
     * The path consists of two straight segments: the first from the starting pose to the midpoint, using linear heading interpolation between the start and end headings; the second from the midpoint to the ending pose, maintaining the ending pose's heading.
     *
     * @param startingPose the initial pose of the path
     * @param endingPose the final pose of the path
     * @return a PathChain representing the constructed two-segment path
     */
    public PathChain makeSpinHalfWayPath(Pose startingPose, Pose endingPose){
        return new PathBuilder().addPath(
                        new BezierLine(new Point(startingPose), new Point((startingPose.getX() + endingPose.getX())/2,
                            (startingPose.getY() + endingPose.getY())/2)))
                .setLinearHeadingInterpolation(startingPose.getHeading(), endingPose.getHeading())
                .addPath(new BezierLine(new Point((startingPose.getX() + endingPose.getX())/2,
                        (startingPose.getY() + endingPose.getY())/2), new Point(endingPose)))
                .setConstantHeadingInterpolation(endingPose.getHeading())
                .build();
    }
    /**
     * Creates a linear path between two poses with a constant heading equal to the ending pose's heading.
     *
     * @param startingPose the initial pose of the path
     * @param endingPose the final pose of the path, whose heading is used throughout the path
     * @return a PathChain representing the constructed linear path with constant heading
     */
    public PathChain makeLinePathConst(Pose startingPose, Pose endingPose){
        return new PathBuilder().addPath(
            new BezierLine(new Point(startingPose), new Point(endingPose)))
                .setConstantHeadingInterpolation(endingPose.getHeading())
                .build();
    }
    /**
     * Creates a path chain representing a linear movement between two poses, optionally inserting a spin point at a specified percentage along the path.
     *
     * If {@code spinPercentage} is between 0 and 1, the path is split at that percentage: the first segment uses tangent heading interpolation up to the spin point, and the second segment uses linear heading interpolation from the spin point to the ending pose. Otherwise, a single linear path with linear heading interpolation is created.
     *
     * @param startingPose the initial pose of the path
     * @param endingPose the final pose of the path
     * @param spinPercentage the fraction (0–1) along the path where a spin point is inserted; values outside this range result in a direct linear path
     * @return a PathChain representing the constructed path
     */
    public PathChain makeLinePath(Pose startingPose, Pose endingPose, double spinPercentage) {
        if (spinPercentage > 0 && spinPercentage < 1) {
            double spinPointX = startingPose.getX() + (endingPose.getX() - startingPose.getX()) * spinPercentage;
            double spinPointY = startingPose.getY() + (endingPose.getY() - startingPose.getY()) * spinPercentage;
            Pose spinPose = new Pose(spinPointX, spinPointY, startingPose.getHeading());
            return new PathBuilder().addPath(
                            new BezierLine(new Point(startingPose), new Point(spinPose))
                    ).setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(new Point(spinPose), new Point(endingPose))
                    ).setLinearHeadingInterpolation(startingPose.getHeading(), endingPose.getHeading())
                    .build();
        }
        return new PathBuilder().addPath(
                        new BezierLine(new Point(startingPose), new Point(endingPose))
                ).setLinearHeadingInterpolation(startingPose.getHeading(), endingPose.getHeading())
                .build();
    }

    /**
     * Creates a Bezier curve path through the given sequence of poses with linear heading interpolation from the first to the last pose.
     *
     * @param poses the sequence of poses defining the curve
     * @return a PathChain representing the constructed Bezier curve path
     */
    public PathChain makeCurvePath(Pose... poses) {
        ArrayList<Point> arr = new ArrayList<>();
//        poses.forEach { arr.add(Point(it)) }
        for(Pose pose: poses){
            arr.add(new Point(pose));
        }
        return new PathBuilder().addPath(
                new BezierCurve(arr)
        ).setLinearHeadingInterpolation(poses[0].getHeading(), poses[poses.length-1].getHeading()).build();
    }

}
