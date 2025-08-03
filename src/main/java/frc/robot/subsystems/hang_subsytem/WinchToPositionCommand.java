package frc.robot.subsystems.hang_subsytem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WinchToPositionCommand extends Command {
    private HangSubsystem hangingSubsystem;
    private double targetPosition;
    private Timer timer = new Timer();
    private static final double MIN_TIME = 0.5; //required to let the servo unlatch before extension
    private static final double MAX_TIME = 5.0;
    private static final double TOLERANCE = 10;

    protected WinchToPositionCommand(HangSubsystem hangingSubsystem, double targetPosition) {
        this.hangingSubsystem = hangingSubsystem;
        this.targetPosition = targetPosition;
    }

    @Override
    public void initialize() {
        hangingSubsystem.setWinchPosition(targetPosition);
        timer.reset();
        timer.start();
    }

    /** Run for at least the MIN_TIME and wait until it either reaches position within TOLERANCE or MAX_TIME has Elapsed. */
    @Override
    public boolean isFinished() {
        return (Math.abs(hangingSubsystem.getWinchPosition() - targetPosition) < TOLERANCE && timer.hasElapsed(MIN_TIME)) 
        || timer.hasElapsed(MAX_TIME);
    }
}