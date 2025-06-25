package org.firstinspires.ftc.teamcode.opModes.auto;


import static org.firstinspires.ftc.teamcode.subsystems.arm.ArmCommands.getAvoidBasket;
import static org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.getCloseDepositClaw;
import static org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.getCloseWhenSampleInPlace;
import static org.firstinspires.ftc.teamcode.subsystems.depositClaw.DepositClawCommands.getOpenDepositClaw;
import static org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.getCloseIntakeClaw;
import static org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.getResetAngleClaw;
import static org.firstinspires.ftc.teamcode.subsystems.intakeClaw.IntakeClawCommands.getTurnLeft;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getCloseSlides;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getEnablePID;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getGoToHighChamber;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftCommands.getGoToHighChamberUp;
import static org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware.getPose;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getChamberSeq;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getCloseNoTransfer;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getOpenCommand;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getOpenPushCommand;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getReset;
import static org.firstinspires.ftc.teamcode.subsystems.robot.RobotCommands.getWallSeq;
import static org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.getArmUp;
import static org.firstinspires.ftc.teamcode.subsystems.v4b.V4bCommands.getExtendoPush;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftHardware;
import org.firstinspires.ftc.teamcode.subsystems.lift.LiftVariables;
import org.firstinspires.ftc.teamcode.subsystems.robot.BulkReads;
import org.firstinspires.ftc.teamcode.commands.util.InstantCommand;
import org.firstinspires.ftc.teamcode.commands.util.RunNonBlocking;
import org.firstinspires.ftc.teamcode.commands.util.WaitUntil;

import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
@Config
@Autonomous
public class java5spec extends AutoBaseJava {
    public java5spec() {super(Side.chamber);}
    public static double offset = -0.6;
    public MultipleTelemetry tele = new MultipleTelemetry(telemetry,
            FtcDashboard.getInstance().getTelemetry());
    @Override
    public void myLoop(){
        tele.addData("current Control", BulkReads.modules.get(0).getCurrent(CurrentUnit.AMPS));
        tele.addData("current Expansion", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));
        tele.addData("current sum", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS)+BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));

        tele.update();
    }
    @Override
    public void myInitLoop() {
        tele.addData("current Control", BulkReads.modules.get(0).getCurrent(CurrentUnit.AMPS));
        tele.addData("current Expansion", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));
        tele.addData("current sum", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS)+BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));

        tele.update();
    }
    public Pose startPose = this.startingPoseChamber;
    public Pose chamberPose1 = new Pose(-35.5, -3, Math.toRadians(180));
    public Pose chamberPose2 = new Pose(-32.0+offset, -5+0.8, Math.toRadians(180));
    public Pose chamberPose3 = new Pose(-32.0+offset, -9+0.8, Math.toRadians(180));
    public Pose chamberPose4 = new Pose(-32.0+offset, -12+0.8, Math.toRadians(180));
    public Pose chamberPose5 = new Pose(-32.0+offset, -14+0.8, Math.toRadians(180));
    public Pose dragPose1 = new Pose(-35, -22.0, Math.toRadians(-62));
    public Pose dragPose2 = new Pose(-37, -31.4, Math.toRadians(-60));
    public Pose dragPose3 = new Pose(-37, -40.7, Math.toRadians(-60));
    public Pose dragPoseTurn1 = new Pose(-40.2, -24, Math.toRadians(-65-75));
    public Pose dragPoseTurn2 = new Pose(-40.3, -35.2, Math.toRadians(-65-75));
    public Pose dragPoseTurn3 = new Pose(-45, -42.1, Math.toRadians(-65-65));
    public Pose pickup1Pose = new Pose(-57-2.6, -39-2, Math.toRadians(180));
    public Pose pickup2Pose = new Pose(-55-2.6, -40-2, Math.toRadians(180));
    public Pose pickup3Pose = new Pose(-53-2.6, -41-2, Math.toRadians(180));
    public Pose pickup4Pose = new Pose(-51-2.6, -42-2, Math.toRadians(180));
    public Pose pickup12Pose = new Pose(-59, -42, Math.toRadians(180));
    public Pose parkPose = new Pose(-47, -29, Math.toRadians(180+40));

    static PathChain scorePreload;
    static PathChain getToDrag1;
    static PathChain turnDrag1;
    static PathChain getToDrag2;
    static PathChain turnDrag2;
    static PathChain getToDrag3;
    static PathChain turnDrag3;
    static PathChain specialHPIntake;
    static PathChain specialHPIntake1;
    static PathChain specialHPIntake1score;
    static PathChain specialHPIntake2;
    static PathChain specialHPIntake2score;
    static PathChain dragToHP;
    static PathChain scorePickup1;
    static PathChain scorePickup2;
    static PathChain scorePickup3;
    static PathChain scorePickup4;
    static PathChain pickup1;
    static PathChain pickup2;
    static PathChain pickup3;
    static PathChain pickup4;
    static PathChain park;
    public static double power = 1.0;
    @Override
    public void myInit() {

        scorePreload = makeLinePath(startPose, chamberPose1);
        getToDrag1 = makeLinePath(chamberPose1, dragPose1);
        turnDrag1 = makeLinePath(dragPose1, dragPoseTurn1);
        getToDrag2 = makeLinePath(dragPoseTurn1, dragPose2);
        turnDrag2 = makeLinePath(dragPose2, dragPoseTurn2);
        getToDrag3 = makeLinePath(dragPoseTurn2, dragPose3);
        turnDrag3 = makeLinePath(dragPose3, dragPoseTurn3);
//        specialHPIntake = makeLinePath(dragPoseTurn3, pickup1Pose);
        specialHPIntake = makeSpinHalfWayPath(dragPoseTurn3, pickup1Pose);
        specialHPIntake1 = makeLinePath(chamberPose2, pickup12Pose);
        park = makeLinePath(chamberPose5, parkPose);
        specialHPIntake1score = makeLinePath(pickup12Pose, chamberPose2);
//        dragToHP = makeCurvePath(chamberPose1, dragPose1, dragPoseTurn1, dragPose2, dragPoseTurn2, dragPose3, dragPoseTurn3,pickup1Pose);
        scorePickup1 = makeSpinHalfWayPath(pickup1Pose, chamberPose2);
        scorePickup2 = makeSpinHalfWayPath(pickup2Pose, chamberPose3);
        scorePickup3 = makeSpinHalfWayPath(pickup3Pose, chamberPose4);
        scorePickup4 = makeSpinHalfWayPath(pickup4Pose, chamberPose5);
        pickup1 = makeSpinHalfWayPath(chamberPose2, pickup2Pose);
        pickup2 = makeSpinHalfWayPath(chamberPose3, pickup3Pose);
        pickup3 = makeSpinHalfWayPath(chamberPose4, pickup4Pose);
//        pickup4 = makeLinePath(chamberPose5, pickup12Pose);
//        park = makeLinePath(basketScore, parkPose);
//        clawSubsystem.getClawRotationServo().setPosition(0.5);
        new Parallel(
            getResetAngleClaw(),
            getCloseIntakeClaw(),
            getAvoidBasket()
        ).schedule();
        follower.setMaxPower(power);
    }

    @Override
    public void myStart() {
        LiftHardware.setPose(0);

        new Sequential(
            getEnablePID(),
            getReset(),
            getGoToHighChamber(),
            followPath(scorePreload).with(
                new Sequential(
                    new WaitUntil(()->getPose()>500),
                    getChamberSeq()
                )
            ),
            getOpenDepositClaw(),
            new Parallel(
                new RunNonBlocking(getCloseSlides()),
                new Sequential(
                    new WaitUntil(()->follower.getCurrentTValue()>0.6),
                    getOpenPushCommand(),
                    getArmUp(),
                    getTurnLeft()
                ),
                new RunNonBlocking(
                    new Sequential(
                        new WaitUntil(()->follower.getCurrentTValue()>0.99),
                        getExtendoPush()
                    )
                ),
                followPath(getToDrag1)
            ),
            followPath(turnDrag1),
            getArmUp(),
            new RunNonBlocking(new Sequential(
                new WaitUntil(()->follower.getCurrentTValue()>0.99),
                getExtendoPush()
            )),
            followPath(getToDrag2),
            followPath(turnDrag2),
            getArmUp(),
            new RunNonBlocking(new Sequential(
                new WaitUntil(()->follower.getCurrentTValue()>0.99),
                getExtendoPush()
            )),
            followPath(getToDrag3),
            followPath(turnDrag3),
            new RunNonBlocking(
                getCloseNoTransfer()
            ),
            new Wait(0.05),
            new Parallel(
                new Sequential(
                    new Wait(0.2),
                    getCloseSlides()
                ),
                followPath(specialHPIntake).raceWith(
                    new Sequential(new Wait(0.15)),
                    getCloseWhenSampleInPlace()
                ),
                new Sequential(
                    new Wait(0.4),
                    getWallSeq(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCloseWhenSampleInPlace().raceWith(
                        slowX,
                        new Sequential(
                            new Wait(0.4)
                        )
                    ),
                    new RunNonBlocking(getChamberSeq())
                )
            ),
            getGoToHighChamber(),
            followPath(scorePickup1),
            getOpenDepositClaw(),
            new Parallel(
                new Sequential(
                    new Wait(0.2),
                    getCloseSlides()
                ),
                followPath(pickup1).raceWith(
                    new Sequential(new Wait(0.15)),
                    getCloseWhenSampleInPlace()
                ),
                new Sequential(
                    new Wait(0.4),
                    getWallSeq(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCloseWhenSampleInPlace().raceWith(
                        slowX,
                        new Sequential(
                            new Wait(0.4)
                        )
                    ),
                    new RunNonBlocking(getChamberSeq())
                )
            ),
            getGoToHighChamber(),
            followPath(scorePickup2),
            getOpenDepositClaw(),
            new Parallel(
                new Sequential(
                    new Wait(0.2),
                    getCloseSlides()
                ),
                followPath(pickup2).raceWith(
                    new Sequential(new Wait(0.15)),
                    getCloseWhenSampleInPlace()
                ),
                new Sequential(
                    new Wait(0.4),
                    getWallSeq(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCloseWhenSampleInPlace().raceWith(
                        slowX,
                        new Sequential(
                            new Wait(0.4)
                        )
                    ),
                    new RunNonBlocking(getChamberSeq())
                )
            ),
            getGoToHighChamber(),
            followPath(scorePickup3),
            getOpenDepositClaw(),
            new Parallel(
                new Sequential(
                    new Wait(0.2),
                    getCloseSlides()
                ),
                followPath(pickup3).raceWith(
                    new Sequential(new Wait(0.15)),
                    getCloseWhenSampleInPlace()
                ),
                new Sequential(
                    new Wait(0.4),
                    getWallSeq(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCloseWhenSampleInPlace().raceWith(
                        slowX,
                        new Sequential(
                            new Wait(0.4)
                        )
                    ),
                    new RunNonBlocking(getChamberSeq())
                )
            ),
            getGoToHighChamber(),
            followPath(scorePickup4),
            getOpenDepositClaw(),

            new Parallel(
                new Wait(0.2).then(getOpenCommand()),
                followPath(park)
            ),
            finishAuto
        ).schedule();

    }
    @Override
    public void myStop(){
        LiftVariables.startingPose = getPose();
    }
}
