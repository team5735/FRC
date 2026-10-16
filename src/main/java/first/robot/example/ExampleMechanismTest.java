package first.robot.example;

import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;

import first.robot.Robot;

@Utility(name = "Example Mechanism Test", group = "Testers")
public class ExampleMechanismTest extends PeriodicOpMode {
    ExampleMechanism mechanism;

    ExampleMechanismTest(Robot robot) { robot.port0.a().whileTrue(mechanism.exampleCommand()); }

    @Override
    public void close() {
        Scheduler.getDefault().run();
    }
}
