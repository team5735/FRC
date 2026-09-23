package first.robot.tests.functional;

import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;

import first.robot.Robot;
import first.robot.mechanisms.ExampleMechanism;

@Utility 
public class ExampleMechanismTest extends PeriodicOpMode {
    ExampleMechanism mechanism;

    ExampleMechanismTest(Robot robot) {
        robot.port0.a().whileTrue(mechanism.exampleCommand());
    }

    @Override
    public void close() {
        Scheduler.getDefault().run();
    }
}
