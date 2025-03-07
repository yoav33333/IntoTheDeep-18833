package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
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
import org.firstinspires.ftc.teamcode.util.FollowerInstance;


import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;

public class AutoBaseJava extends MegiddoOpMode {
    private static Telemetry telemetryA;
    private Follower follower;

    public enum Side{
        basket,
        chamber
        }
    public static Pose startingPoseChamber = new Pose(-62, -8.5, Math.toRadians(0));
    public static Pose startingPoseBasket = new Pose(-62, 8.5+24, 0);
    Side side;
    public AutoBaseJava(Side side){
        this.side = side;
    }
    @Override
    final public void preInit() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.setConstants(FConstants.class, LConstants.class);

        switch (side) {
            case basket:
                FollowerInstance.reset(FeatureRegistrar.getActiveOpMode().hardwareMap);
                follower = FollowerInstance.getInstance(FeatureRegistrar.getActiveOpMode().hardwareMap);
                follower.setCurrentPoseWithOffset(startingPoseBasket);
                break;
            case chamber:
                FollowerInstance.reset(FeatureRegistrar.getActiveOpMode().hardwareMap);
                follower = FollowerInstance.getInstance(FeatureRegistrar.getActiveOpMode().hardwareMap);
                follower.setCurrentPoseWithOffset(startingPoseChamber);
        }
    }
    public Lambda runFollower = new Lambda("update Follower")
            .setFinish(()->false)
            .setExecute(()->{
                follower.update();
                follower.telemetryDebug(telemetryA);
            });
    public Lambda finishAuto = new Lambda("finishAuto")
        .setInit(()-> {
            follower.breakFollowing();
            follower.update();
            linearSlides.setStartingPose(linearSlides.getPose());

        });

    public Follower getFollower() {
        return follower;
    }

    public static Lambda instantCommand(Runnable runnable){
        return new Lambda("instant command")
                .setInit(runnable);
    }
    public Lambda followPath(PathChain chain){
        return new Lambda("follow-path-chain")
            .setInit(() -> {
                follower.followPath(chain, true);
                runFollower.schedule();
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
    public static Lambda waitUntil(BooleanSupplier supplier){
        return new Lambda("Wait until")
            .setFinish(supplier::getAsBoolean);
    }
    public static Lambda runNonBlocking(Command... commands){
        return new Lambda("Wait until")
            .setInit(()->{for(Command command : commands){command.schedule();}});
    }

    public Lambda setHeadingPID(double p, double d){
        return new Lambda("setHeadingPID")
            .setInit(()-> {
                FollowerConstants.headingPIDFCoefficients.D = d;
                FollowerConstants.headingPIDFCoefficients.P = p;
            });
    }
    static double openExD = 0.18;
    static double openExP = 1.4;
    static double closeExD = 0.18;
    static double closeExP = 1.4;

    public Lambda setOpenExPid = setHeadingPID(openExP, openExD);
    public Lambda setCloseExPid = setHeadingPID(closeExP, closeExD);
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
                .setFinish(()-> ((!follower.isBusy() ) || follower.isRobotStuck()))
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
                follower.setMaxPower(0.5);
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
        return follower.pathBuilder().addPath(
            new BezierLine(new Point(startingPose), new Point(endingPose)))
                .setLinearHeadingInterpolation(startingPose.getHeading(), endingPose.getHeading())
                .build();
    }
    public PathChain makeCurvePath(Pose... poses) {
        ArrayList<Point> arr = new ArrayList<>();
//        poses.forEach { arr.add(Point(it)) }
        for(Pose pose: poses){
            arr.add(new Point(pose));
        }
        return follower.pathBuilder().addPath(
                new BezierCurve(arr)
        ).setLinearHeadingInterpolation(poses[0].getHeading(), poses[poses.length-1].getHeading()).build();
    }

}
