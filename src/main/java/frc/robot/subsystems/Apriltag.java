package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ApriltagConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;

/** The subsystem for AprilTag vision processing. */
public final class Apriltag extends SubsystemBase {

  /** The singleton instance of the {@link Apriltag} subsystem. */
  private static Apriltag instance = null;

  /** Collection of all cameras on the robot. */
  private final Map<String, Camera> cameras = new HashMap<>();

  /** If we have gotten and applied a global measurement pose */
  private boolean hasReceivedGlobalPose = false;

  /** NetworkTable for publishing apriltag detection state. */
  private final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("Robot").getSubTable("Apriltag");

  /**
   * Gets the singleton instance of the {@link Apriltag} class.
   *
   * @return The instance of the {@link Apriltag} class.
   */
  public static Apriltag getInstance() {
    if (instance == null) {
      instance = new Apriltag();
    }
    return instance;
  }

  /** Constructor for the Apriltag subsystem. */
  private Apriltag() {
    // Initialize cameras from Constants
    for (ApriltagConstants.ApriltagCameraConfig cameraConfig : ApriltagConstants.PHOTON_CAMERAS) {
      Camera camera =
          new Camera(
              cameraConfig.getName(),
              cameraConfig.getTransform(),
              cameraConfig.getStrategy(),
              ApriltagConstants.SINGLE_TAG_STD_DEVS,
              ApriltagConstants.MULTI_TAG_STD_DEVS);

      cameras.put(camera.getName(), camera);

      if (!camera.camera.isConnected()) {
        DriverStation.reportWarning(
            "PhotonCamera " + camera.getName() + " is not connected!", false);
      }

      SmartDashboard.putBoolean(camera.getName() + " Connected", camera.camera.isConnected());
    }
  }

  /**
   * Whether the subsystem has received a global pose from vision.
   *
   * @return True if a global pose has been received, false otherwise
   */
  public boolean hasReceivedGlobalPose() {
    return hasReceivedGlobalPose;
  }

  /** Updates the pose estimation in the Swerve subsystem with vision measurements. */
  @Override
  public void periodic() {
    SwerveDrive swerveDrive = Swerve.getInstance().getSwerveDrive();
    Pose2d currentPose = Swerve.getInstance().getPose();

    int totalTagsDetected = 0;
    boolean anyPoseUpdated = false;

    // Process all cameras
    for (Camera camera : cameras.values()) {
      SmartDashboard.putBoolean(camera.getName() + " Connected", camera.camera.isConnected());
      if (!camera.camera.isConnected()) continue;

      // Get tag count and update camera data
      int cameraTagCount = processCameraResults(camera, currentPose);
      totalTagsDetected += cameraTagCount;

      // Publish camera data to NetworkTables
      updateCameraNetworkTables(camera, cameraTagCount);

      // Process vision measurement if available
      if (camera.estimatedRobotPose.isPresent()) {
        anyPoseUpdated = true;
        EstimatedRobotPose pose = camera.estimatedRobotPose.get();

        hasReceivedGlobalPose = true;

        // Add vision measurement to swerve drive
        swerveDrive.addVisionMeasurement(
            pose.estimatedPose.toPose2d(), pose.timestampSeconds, camera.curStdDevs);

        table
            .getSubTable(camera.getName())
            .getEntry("lastPoseTimestamp")
            .setDouble(pose.timestampSeconds);
      }
    }

    // Update global vision status
    table.getEntry("totalTagsDetected").setInteger(totalTagsDetected);
    table.getEntry("visionUpdatingPose").setBoolean(anyPoseUpdated);
    table.getEntry("hasReceivedGlobalPose").setBoolean(hasReceivedGlobalPose);
  }

  /**
   * Process camera results and return tag count
   *
   * @param camera The camera to process
   * @param currentPose Current robot pose
   * @return Number of tags detected
   */
  private int processCameraResults(Camera camera, Pose2d currentPose) {
    // Update camera results and get estimated pose
    camera.updateUnreadResults();
    camera.getEstimatedGlobalPose(currentPose);

    // Count total detected tags
    int tagCount = 0;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        tagCount = Math.max(tagCount, result.getTargets().size());
      }
    }

    return tagCount;
  }

  /**
   * Update NetworkTables with camera information
   *
   * @param camera The camera to update
   * @param tagCount Number of tags detected
   */
  private void updateCameraNetworkTables(Camera camera, int tagCount) {
    NetworkTable cameraTable = table.getSubTable(camera.getName());
    cameraTable.getEntry("tagsDetected").setInteger(tagCount);
    cameraTable.getEntry("isConnected").setBoolean(camera.camera.isConnected());
  }

  /**
   * Calculate a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag The ID of the AprilTag
   * @param robotOffset The offset to apply to the pose for proper robot positioning
   * @return The target pose for the robot relative to the AprilTag
   */
  public Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    return ApriltagConstants.FIELD_LAYOUT
        .getTagPose(aprilTag)
        .map(pose3d -> pose3d.toPose2d().transformBy(robotOffset))
        .orElseThrow(
            () -> new RuntimeException("Cannot find AprilTag " + aprilTag + " in field layout"));
  }

  /**
   * Get the distance of the robot from a specific AprilTag.
   *
   * @param id AprilTag ID
   * @return Distance in meters, or -1 if tag not found
   */
  public double getDistanceFromAprilTag(int id) {
    Pose2d currentPose = Swerve.getInstance().getPose();
    return ApriltagConstants.FIELD_LAYOUT
        .getTagPose(id)
        .map(pose3d -> PhotonUtils.getDistanceToPose(currentPose, pose3d.toPose2d()))
        .orElse(-1.0);
  }

  /**
   * Get tracked target from a camera for a specific AprilTag ID.
   *
   * @param id AprilTag ID
   * @param cameraName Name of the camera to check
   * @return Tracked target, or null if not found
   */
  public PhotonTrackedTarget getTargetFromId(int id, String cameraName) {
    Camera camera = cameras.get(cameraName);
    if (camera == null) return null;

    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          if (target.getFiducialId() == id) {
            return target;
          }
        }
      }
    }
    return null;
  }

  /**
   * Get all cameras configured in this subsystem.
   *
   * @return Map of camera name to Camera object
   */
  public Map<String, Camera> getCameras() {
    return cameras;
  }

  /** Class representing a camera on the robot. */
  public static class Camera {
    /** Latency alert for high camera latency */
    public final Alert latencyAlert;

    /** Camera instance for communication */
    public final PhotonCamera camera;

    /** Pose estimator for this camera */
    public final PhotonPoseEstimator poseEstimator;

    /** Standard deviation for single tag readings */
    private final Matrix<N3, N1> singleTagStdDevs;

    /** Standard deviation for multi-tag readings */
    private final Matrix<N3, N1> multiTagStdDevs;

    /** Current standard deviations in use */
    public Matrix<N3, N1> curStdDevs;

    /** Latest estimated robot pose */
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /** Results cache to avoid unnecessary queries */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();

    /** Last timestamp when camera was read */
    private double lastReadTimestamp = 0;

    /** Count of times that the odometry thinks we're far from the April Tag. */
    private int longDistancePoseEstimationCount = 0;

    /**
     * Constructs a camera configuration.
     *
     * @param name Camera name in PhotonVision
     * @param transform Camera transform relative to robot
     * @param strategy Pose estimation strategy
     * @param singleTagStdDevs Standard deviations for single tag detection
     * @param multiTagStdDevs Standard deviations for multi-tag detection
     */
    public Camera(
        String name,
        Transform3d transform,
        PoseStrategy strategy,
        Matrix<N3, N1> singleTagStdDevs,
        Matrix<N3, N1> multiTagStdDevs) {
      this.latencyAlert =
          new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);
      this.camera = new PhotonCamera(name);
      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevs;
      this.curStdDevs = singleTagStdDevs;

      this.poseEstimator =
          new PhotonPoseEstimator(ApriltagConstants.FIELD_LAYOUT, strategy, transform);
      this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Get the camera name.
     *
     * @return Camera name
     */
    public String getName() {
      return camera.getName();
    }

    /**
     * Get estimated global robot pose from this camera.
     *
     * @param currentPose Current robot pose for filtering
     * @return Estimated robot pose if available
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d currentPose) {
      if (!estimatedRobotPose.isPresent()) {
        return estimatedRobotPose;
      }

      // Filter poses that are too far from current position
      double distance =
          PhotonUtils.getDistanceToPose(
              currentPose, estimatedRobotPose.get().estimatedPose.toPose2d());

      if (distance > 1.0) {
        longDistancePoseEstimationCount++;
        if (longDistancePoseEstimationCount < 10) {
          return Optional.empty();
        }
      } else {
        longDistancePoseEstimationCount = 0;
      }

      return estimatedRobotPose;
    }

    /** Update cache of unread results from camera. */
    public void updateUnreadResults() {
      double currentTime = NetworkTablesJNI.now() / 1000000.0;
      double mostRecentTimestamp = getMostRecentTimestamp();
      double debounceTime = ApriltagConstants.CAMERA_DEBOUNCE_TIME;

      // Only fetch new results if enough time has passed
      if ((resultsList.isEmpty() || (currentTime - mostRecentTimestamp >= debounceTime))
          && (currentTime - lastReadTimestamp >= debounceTime)) {

        processNewResults(currentTime);
      }
    }

    /**
     * Get the timestamp of the most recent result
     *
     * @return Timestamp in seconds
     */
    private double getMostRecentTimestamp() {
      double mostRecentTimestamp = 0.0;
      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }
      return mostRecentTimestamp;
    }

    /**
     * Process new results from the camera
     *
     * @param currentTime Current time in seconds
     */
    private void processNewResults(double currentTime) {
      // Get all unread results
      List<PhotonPipelineResult> newResults = camera.getAllUnreadResults();

      // Filter and add results with targets
      newResults.stream().filter(PhotonPipelineResult::hasTargets).forEach(resultsList::add);

      if (!resultsList.isEmpty()) {
        // Sort by timestamp (newest first)
        resultsList.sort(
            (a, b) -> Double.compare(b.getTimestampSeconds(), a.getTimestampSeconds()));

        // Trim to max size
        while (resultsList.size() > ApriltagConstants.MAX_CAMERA_RESULTS) {
          resultsList.remove(resultsList.size() - 1);
        }

        updateEstimatedGlobalPose();
      }

      lastReadTimestamp = currentTime;
    }

    /** Update the estimated global pose from camera results. */
    private void updateEstimatedGlobalPose() {
      for (PhotonPipelineResult result : resultsList) {
        if (!result.hasTargets()) continue;

        // Check for acceptable ambiguity
        if (!hasAcceptableAmbiguity(result.getTargets())) {
          continue;
        }

        // Try to get pose estimate
        Optional<EstimatedRobotPose> visionEst = poseEstimator.update(result);

        if (visionEst.isPresent()) {
          updateEstimationStdDevs(visionEst, result.getTargets());
          estimatedRobotPose = visionEst;
          return; // Use first good result
        }
      }
    }

    /**
     * Check if targets have acceptable ambiguity
     *
     * @param targets List of tracked targets
     * @return True if ambiguity is acceptable
     */
    private boolean hasAcceptableAmbiguity(List<PhotonTrackedTarget> targets) {
      double bestAmbiguity = 1.0;
      for (PhotonTrackedTarget target : targets) {
        if (target.getPoseAmbiguity() != -1 && target.getPoseAmbiguity() < bestAmbiguity) {
          bestAmbiguity = target.getPoseAmbiguity();
        }
      }

      return bestAmbiguity <= ApriltagConstants.MAXIMUM_AMBIGUITY;
    }

    /**
     * Update standard deviations based on vision results.
     *
     * @param estimatedPose Estimated pose
     * @param targets Detected targets
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

      if (estimatedPose.isEmpty()) {
        curStdDevs = singleTagStdDevs;
        return;
      }

      // Calculate average distance and number of tags
      double avgDistance = calculateAverageTagDistance(estimatedPose.get(), targets);
      int numTags = countVisibleTags(targets);

      // Select base std dev based on number of tags
      Matrix<N3, N1> baseStdDevs = (numTags > 1) ? multiTagStdDevs : singleTagStdDevs;

      // Scale by distance
      curStdDevs = baseStdDevs.times(1 + (avgDistance * avgDistance / 30.0));
    }

    /**
     * Calculate average distance to tags
     *
     * @param pose Estimated robot pose
     * @param targets Tracked targets
     * @return Average distance to visible tags
     */
    private double calculateAverageTagDistance(
        EstimatedRobotPose pose, List<PhotonTrackedTarget> targets) {

      double totalDistance = 0;
      int tagCount = 0;

      for (PhotonTrackedTarget target : targets) {
        var tagPose = ApriltagConstants.FIELD_LAYOUT.getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) continue;

        tagCount++;
        totalDistance +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(pose.estimatedPose.toPose2d().getTranslation());
      }

      return tagCount > 0 ? totalDistance / tagCount : 1.0;
    }

    /**
     * Count the number of visible tags with valid poses
     *
     * @param targets Tracked targets
     * @return Number of valid tags
     */
    private int countVisibleTags(List<PhotonTrackedTarget> targets) {
      int count = 0;
      for (PhotonTrackedTarget target : targets) {
        if (ApriltagConstants.FIELD_LAYOUT.getTagPose(target.getFiducialId()).isPresent()) {
          count++;
        }
      }
      return count;
    }
  }
}
