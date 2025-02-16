package org.firstinspires.ftc.teamcode.opModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import dev.frozenmilk.dairy.core.FeatureRegistrar.activeOpMode
import dev.frozenmilk.mercurial.commands.Lambda
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants
import java.util.function.BooleanSupplier

open class idk(var side: Side) :
    MegiddoOpMode() {
    enum class Side {
        basket,
        chamber
    }

    override fun preInit() {
        telemetryA = MultipleTelemetry(
            this.telemetry, FtcDashboard.getInstance().telemetry
        )

        when (side) {
            Side.basket -> {
                Constants.setConstants(FConstants::class.java, LConstants::class.java)
                follower = Follower(activeOpMode.hardwareMap)
                follower!!.setCurrentPoseWithOffset(startingPoseBasket)
            }

            Side.chamber -> {
                Constants.setConstants(FConstants::class.java, LConstants::class.java)
                follower = Follower(activeOpMode.hardwareMap)
                follower!!.setCurrentPoseWithOffset(startingPoseChamber)
            }
        }
    }

    companion object {
        private var telemetryA: Telemetry? = null

        var startingPoseChamber: Pose = Pose(-62.0, -8.5, Math.toRadians(0.0))
        var startingPoseBasket: Pose = Pose(-62.0, 8.5 + 24, 0.0)
        var follower: Follower? = null
        var runFollower: Lambda = Lambda("update Follower")
            .setFinish { false }
            .setExecute {
                follower!!.update()
                follower!!.telemetryDebug(
                    telemetryA
                )
            }
        var finishAuto: Lambda = Lambda("finishAuto")
            .setInit {
                follower!!.breakFollowing()
                follower!!.update()
                runFollower.cancel()
            }

        fun followPath(chain: PathChain?): Lambda {
            return Lambda("follow-path-chain")
                .setInit {
                    follower!!.followPath(
                        chain,
                        true
                    )
                    runFollower.schedule()
                } //            .setExecute(() ->{
                //
                //
                //            })
                .setFinish { (!follower!!.isBusy || follower!!.isRobotStuck) }
                .setEnd { interrupted: Boolean ->
                    if (interrupted) follower!!.breakFollowing()
                }
        }

        fun waitUntil(supplier: BooleanSupplier): Lambda {
            return Lambda("Wait until")
                .setFinish { supplier.asBoolean }
        }


        fun turnTo(angle: Double): Lambda {
            return Lambda("follow-path-chain")
                .setInit {
                    follower!!.turnToDegrees(
                        angle
                    )
                }
                .setExecute {
                    follower!!.update()
                    follower!!.telemetryDebug(
                        telemetryA
                    )
                }
                .setFinish { ((!follower!!.isBusy) || follower!!.isRobotStuck) }
                .setEnd { interrupted: Boolean ->
                    if (interrupted) follower!!.breakFollowing()
                }
        }

        fun turn(angle: Double): Lambda {
            return Lambda("follow-path-chain")
                .setInit {
                    follower!!.turnDegrees(
                        angle,
                        false
                    )
                }
                .setExecute {
                    follower!!.update()
                    follower!!.telemetryDebug(
                        telemetryA
                    )
                }
                .setFinish { ((!follower!!.isBusy) || follower!!.isRobotStuck) }
                .setEnd { interrupted: Boolean ->
                    if (interrupted) follower!!.breakFollowing()
                }
        }

        fun makeLinePath(startingPose: Pose, endingPose: Pose): PathChain {
            return follower!!.pathBuilder().addPath(
                BezierLine(Point(startingPose), Point(endingPose))
            )
                .setLinearHeadingInterpolation(startingPose.heading, endingPose.heading)
                .build()
        }

        fun makeCurvePath(vararg poses: Pose): PathChain {
            val arr = ArrayList<Point>()
            //        poses.forEach { arr.add(Point(it)) }
            for (pose in poses) {
                arr.add(Point(pose))
            }
            return follower!!.pathBuilder().addPath(
                BezierCurve(arr)
            ).setLinearHeadingInterpolation(poses[0].heading, poses[poses.size - 1].heading).build()
        }
    }
}