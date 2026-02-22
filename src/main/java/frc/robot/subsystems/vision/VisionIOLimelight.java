package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;

  private final NetworkTable table;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber; // "tl" (ms)
  private final DoubleSubscriber txSubscriber;      // "tx" (deg)
  private final DoubleSubscriber tySubscriber;      // "ty" (deg)
  private final DoubleSubscriber tidSubscriber;     // "tid" (tag id)

  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight (ex: "limelight").
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    this.rotationSupplier = rotationSupplier;

    this.table = NetworkTableInstance.getDefault().getTable(name);

    this.orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();

    this.latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    this.txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    this.tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    this.tidSubscriber = table.getDoubleTopic("tid").subscribe(0.0);

    this.megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    this.megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    long nowUs = RobotController.getFPGATime();
    long lastChangeUs = latencySubscriber.getLastChange();
    inputs.connected = (nowUs - lastChangeUs) < 250_000; 

//latest tag
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()),
            Rotation2d.fromDegrees(tySubscriber.get()),
            tidSubscriber.get());

    //MegaTag2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault().flush();

    //Reads NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;

      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

      poseObservations.add(
          new PoseObservation(
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              parsePose(rawSample.value),
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
              (int) rawSample.value[7],
              rawSample.value[9],
              PoseObservationType.MEGATAG_1));
    }

    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;

      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

      poseObservations.add(
          new PoseObservation(
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              parsePose(rawSample.value),
              0.0,
              (int) rawSample.value[7],
              rawSample.value[9],
              PoseObservationType.MEGATAG_2));
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    inputs.tagIds = new int[tagIds.size()];
    int idx = 0;
    for (int id : tagIds) {
      inputs.tagIds[idx++] = id;
    }
  }

  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}





















/*package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;

  private final NetworkTable table;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber; // "tl" (ms)
  private final DoubleSubscriber tvSubscriber;      // "tv" (0 or 1)
  private final DoubleSubscriber txSubscriber;      // "tx" (deg)
  private final DoubleSubscriber tySubscriber;      // "ty" (deg)
  private final DoubleSubscriber tidSubscriber;     // "tid" (tag id)

  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight (ex: "limelight-tag" or "limelight").
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   *
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    this.rotationSupplier = rotationSupplier;

    this.table = NetworkTableInstance.getDefault().getTable(name);

    this.orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();

    this.latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    this.tvSubscriber = table.getDoubleTopic("tv").subscribe(0.0);
    this.txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    this.tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    this.tidSubscriber = table.getDoubleTopic("tid").subscribe(0.0);

    this.megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    this.megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Connection status: did "tl" update recently?
    // RobotController.getFPGATime() is microseconds, getLastChange() is microseconds.
    long nowUs = RobotController.getFPGATime();
    long lastChangeUs = latencySubscriber.getLastChange();
    inputs.connected = (nowUs - lastChangeUs) < 250_000; // 250ms

    // Limelight valid target flag
    boolean hasTarget = tvSubscriber.get() > 0.5;

    // Latest target observation (gate tx/ty/tid on tv so you never "fake" a target at 0,0)
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(hasTarget ? txSubscriber.get() : 0.0),
            Rotation2d.fromDegrees(hasTarget ? tySubscriber.get() : 0.0),
            hasTarget ? tidSubscriber.get() : 0.0);

    // Orientation for MegaTag2 (Limelight recommends flushing)
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault().flush();

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;

      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

      poseObservations.add(
          new PoseObservation(
              // Timestamp: server publish time minus LL latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              parsePose(rawSample.value),
              // Ambiguity (only meaningful for single tag)
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
              (int) rawSample.value[7],
              rawSample.value[9],
              PoseObservationType.MEGATAG_1));
    }

    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;

      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

      poseObservations.add(
          new PoseObservation(
              // Timestamp: server publish time minus LL latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              parsePose(rawSample.value),
              0.0,
              (int) rawSample.value[7],
              rawSample.value[9],
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    // Save tag IDs (pose-based detection)
    inputs.tagIds = new int[tagIds.size()];
    int idx = 0;
    for (int id : tagIds) {
      inputs.tagIds[idx++] = id;
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. *
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
*/