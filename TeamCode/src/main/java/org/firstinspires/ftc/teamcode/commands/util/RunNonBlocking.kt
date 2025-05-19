package org.firstinspires.ftc.teamcode.commands.util

import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Command

/**
 * a command that runs without blocking the command group
 */
class RunNonBlocking(private val command: Command) : Command {

    /**
     * Schedules the wrapped command to run asynchronously.
     */
    override fun initialise() {
        command.schedule()
    }

    /**
     * Does nothing during command execution.
     */
    override fun execute() {
    }

    /**
     * Does nothing when the command ends or is interrupted.
     *
     * This method is intentionally left empty as no cleanup or additional actions are required.
     */
    override fun end(interrupted: Boolean) {
    }

    /**
     * Indicates that this command finishes immediately after initialization.
     *
     * @return Always returns true.
     */
    override fun finished(): Boolean {
        return true
    }

    override val requirements: Set<Any> = emptySet()
    override val runStates: Set<Wrapper.OpModeState> = setOf(Wrapper.OpModeState.INIT, Wrapper.OpModeState.ACTIVE)
    /**
 * Returns the string representation of this command.
 *
 * @return The string "RunNonBlocking".
 */
override fun toString() = "RunNonBlocking"
}