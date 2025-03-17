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
    public AutoBaseJava(Side side){
        this.side = side;
    }
    protected Follower follower;
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
    @Override
    public void myFullStop(){
        followerSubsystem.setStartingPose(follower.getPose());
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
//    }
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
                follower.setMaxPower(0.9);
                follower.followPath(makeLinePath(follower.getPose(),
                 new Pose(follower.getPose().getX()-50, follower.getPose().getY(),
                0)));
            })
            .setExecute(() ->{
                follower.update();
                follower.telemetryDebug(telemetryA);

            })
            .setFinish(()-> ((!follower.isBusy() ) || follower.isRobotStuck()));

    public PathChain makeLinePath(Pose startingPose, Pose endingPose){
        return new PathBuilder().addPath(
            new BezierLine(new Point(startingPose), new Point(endingPose)))
                .setLinearHeadingInterpolation(startingPose.getHeading(), endingPose.getHeading())
                .build();
    }
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
    public PathChain makeLinePathConst(Pose startingPose, Pose endingPose){
        return new PathBuilder().addPath(
            new BezierLine(new Point(startingPose), new Point(endingPose)))
                .setConstantHeadingInterpolation(endingPose.getHeading())
                .build();
    }
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
    public PathChain makeBezierByAngle(Pose startingPose, Pose endingPose) {
        // Calculate distance between start and end points
        double distance = Math.hypot(endingPose.getX() - startingPose.getX(), endingPose.getY() - startingPose.getY());

        // Compute angle difference properly
        double angleDifference = Math.abs(Math.atan2(
                Math.sin(endingPose.getHeading() - startingPose.getHeading()),
                Math.cos(endingPose.getHeading() - startingPose.getHeading())
        ));

        // **Dynamically Adjust Control Point Distance (Optimized for Shorter Paths)**
        double baseFactor = 0.1 + 0.1 * Math.sqrt(distance / 50); // More aggressive shortening
        double controlPointDistance = distance * baseFactor;

        if (angleDifference > Math.PI / 2) {
            controlPointDistance *= 1.2; // Slightly longer for sharp turns
        } else if (angleDifference < Math.PI / 6) {
            controlPointDistance *= 0.75; // Even shorter for small turns
        }

        // Ensure a **minimum control distance** to prevent zero-length paths
        controlPointDistance = Math.max(controlPointDistance, distance * 0.1);

        // Determine turn direction (left or right turn)
        double turnDirection = Math.signum(
                Math.sin(endingPose.getHeading() - startingPose.getHeading())
        );
        controlPointDistance *= turnDirection; // Flip control point side for left turns

        // Compute control points
        double startX = startingPose.getX() + Math.cos(startingPose.getHeading()) * controlPointDistance;
        double startY = startingPose.getY() + Math.sin(startingPose.getHeading()) * controlPointDistance;
        Pose startControlPose = new Pose(startX, startY, startingPose.getHeading());

        double endX = endingPose.getX() - Math.cos(endingPose.getHeading()) * controlPointDistance;
        double endY = endingPose.getY() - Math.sin(endingPose.getHeading()) * controlPointDistance;
        Pose endControlPose = new Pose(endX, endY, endingPose.getHeading());

        // **Step 1: Check movement direction (dot product)**
        double dx = endingPose.getX() - startingPose.getX();
        double dy = endingPose.getY() - startingPose.getY();
        double movementDirection = dx * Math.cos(startingPose.getHeading()) + dy * Math.sin(startingPose.getHeading());
        boolean reverseByMovement = movementDirection < 0; // If moving backward, reverse

        // **Step 2: Check if heading suggests backward motion**
        boolean reverseByAngle = angleDifference > Math.PI / 2; // Large angle difference may need reversing

        // **Step 3: Apply reversal if either condition is met**
        boolean rev = reverseByMovement || reverseByAngle;

        // Create the Bézier curve
        BezierCurve bezierCurve = new BezierCurve(
                new Point(startingPose),
                new Point(startControlPose),
                new Point(endControlPose),
                new Point(endingPose)
        );

        return new PathBuilder()
                .addPath(bezierCurve)
                .setTangentHeadingInterpolation()
                .setReversed(rev).build(); // Fully dynamic reversing!
    }
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
