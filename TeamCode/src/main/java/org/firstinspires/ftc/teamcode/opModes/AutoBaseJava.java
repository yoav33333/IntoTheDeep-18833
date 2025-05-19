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
