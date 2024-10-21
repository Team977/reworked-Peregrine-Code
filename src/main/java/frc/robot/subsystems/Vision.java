/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  private final PhotonCamera aprilTagCameraLeft;
  private final PhotonCamera aprilTagCameraRight;
  private final PhotonCamera noteCamera;

  private final PhotonPoseEstimator PhotonEstimatorLeft;
  private final PhotonPoseEstimator PhotonEstimatorRight;
  private double lastEstTimestamp = 0;

  private static enum ProcessingType{
    AVERAGE,
    TAKEBESTVALUE
  }

  public Vision() {
    aprilTagCameraLeft = new PhotonCamera(kAprilTagCameraNameLeft);
    aprilTagCameraRight = new PhotonCamera(kAprilTagCameraNameRight);
    noteCamera = new PhotonCamera(kNoteCameraName);

    PhotonEstimatorLeft =
        new PhotonPoseEstimator(
            kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            aprilTagCameraLeft,
            kRobotToApriltagCamLeft);
    PhotonEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    PhotonEstimatorRight =
        new PhotonPoseEstimator(
            kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            aprilTagCameraRight,
            kRobotToApriltagCamRight);
    PhotonEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public PhotonPipelineResult getLatestAprilTagResult(PhotonCamera photonCamera) {
    return photonCamera.getLatestResult();
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator photonPoseEstimator, PhotonCamera photonCamera) {
    var visionEst = photonPoseEstimator.update();
    double latestTimestamp = photonCamera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {

    List<Optional<EstimatedRobotPose>> robotPoses = new ArrayList<Optional<EstimatedRobotPose>>();
    robotPoses.add(getEstimatedGlobalPose(PhotonEstimatorLeft, aprilTagCameraLeft));
    robotPoses.add(getEstimatedGlobalPose(PhotonEstimatorRight, aprilTagCameraRight));

    return combinePoses(robotPoses);

    /*Optional<EstimatedRobotPose> RightCamera = getEstimatedGlobalPoseRight();
    Optional<EstimatedRobotPose> LeftCamera = getEstimatedGlobalPoseLeft();

    if (RightCamera.isEmpty() || LeftCamera.isEmpty()) {
      return Optional.ofNullable(null);
    }

    Pose2d leftPosition = LeftCamera.get().estimatedPose.toPose2d();
    Pose2d RightPosition = RightCamera.get().estimatedPose.toPose2d();
    // Translation2d midPoint = new Translation2d();
    // Rotation2d midRotation  = new Rotation2d();

    if (leftPosition.getTranslation().getDistance(RightPosition.getTranslation()) < 1) {
      return LeftCamera;
    } else {
      return Optional.ofNullable(null);
    }
    /*
    if(leftPosition.getTranslation().getDistance(RightPosition.getTranslation()) > 0.5){
      midPoint = leftPosition.getTranslation().interpolate(RightPosition.getTranslation(), 0.5);
      midRotation = new Rotation2d((leftPosition.getRotation().getRadians() + RightPosition.getRotation().getRadians()) / 2);
    }else{
      return null;
    }

    Collection<PhotonTrackedTarget>
    List<PhotonTrackedTarget> RobotTags = getEstimatedGlobalPoseLeft().get().targetsUsed;

    Optional<EstimatedRobotPose> PoseOut = Optional.ofNullable(new EstimatedRobotPose(
      new Pose3d(new Pose2d(midPoint.getX(), midPoint.getY(), midRotation)),
      lastEstTimestamp, getEstimatedGlobalPoseLeft().get().targetsUsed.addAll(new Collection<PhotonTrackedTarget>(getEstimatedGlobalPoseRight().get().targetsUsed)), null));
    return PoseOut;

    new Est*/
  }
  
  private Optional<EstimatedRobotPose> combinePoses(List<Optional<EstimatedRobotPose>> poses){

    boolean hasValues = false;

    Pose3d estamatedAvgPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

    List<PhotonTrackedTarget> photonTrackedTargets = new ArrayList<PhotonTrackedTarget>();

    double lastEstTimestamp = 0;

    int usedSamples = 0;

    for(int i = 0; i < poses.size(); i++){

      if(poses.get(i).isPresent()){

        EstimatedRobotPose estimatedRobotPoseLoop = poses.get(i).get();

        if(i != 0){
          if(estamatedAvgPose.getTranslation().getDistance(estimatedRobotPoseLoop.estimatedPose.getTranslation()) > 1){
            return Optional.empty();
          };
        }

        estamatedAvgPose
          .plus(
              new Transform3d(estimatedRobotPoseLoop.estimatedPose.getTranslation()
                , estimatedRobotPoseLoop.estimatedPose.getRotation())).div(poses.size());

        lastEstTimestamp
           = lastEstTimestamp < estimatedRobotPoseLoop.timestampSeconds? estimatedRobotPoseLoop.timestampSeconds: lastEstTimestamp;
      
          photonTrackedTargets = combinePhotonTrackedTargets(photonTrackedTargets, estimatedRobotPoseLoop.targetsUsed);

          /*
        for(int j = 0; j < estimatedRobotPoseLoop.targetsUsed.size(); j++){

          if(!photonTrackedTargets.contains(estimatedRobotPoseLoop.targetsUsed.get(j))){
            photonTrackedTargets.add(estimatedRobotPoseLoop.targetsUsed.get(j));
          }

        } */

      }else{
        return Optional.empty();
      }

    }

    estamatedAvgPose.div(usedSamples);

    if(!hasValues){
      return Optional.empty();
    }

    return Optional.of(new EstimatedRobotPose(estamatedAvgPose, lastEstTimestamp, photonTrackedTargets, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));

  }

  public List<PhotonTrackedTarget> combinePhotonTrackedTargets(List<PhotonTrackedTarget> photonTargetsAddTo, List<PhotonTrackedTarget> photonTargetsAdd){
        for(int j = 0; j < photonTargetsAdd.size(); j++){

          if(!photonTargetsAddTo.contains(photonTargetsAdd.get(j))){
            photonTargetsAddTo.add(photonTargetsAdd.get(j));
          }

        }

        return photonTargetsAddTo;
  }

  public double getAngle() {
    if (aprilTagCameraRight.getLatestResult().hasTargets()) {
      return aprilTagCameraRight.getLatestResult().getTargets().get(0).getPitch();
    }
    return 0;
  }
  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = kSingleTagStdDevs;
    var targets = combinePhotonTrackedTargets(getLatestAprilTagResult(aprilTagCameraLeft).targets, getLatestAprilTagResult(aprilTagCameraRight).targets);
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = PhotonEstimatorLeft.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public PhotonPipelineResult getLatestNoteResult() {
    return noteCamera.getLatestResult();
  }

  /**
   * Returns the yaw angle to the best note. returns a value of 0 if no targets are in the latest
   * pipeline result
   */
  public double getYawToNote() {
    PhotonPipelineResult NoteDet = getLatestNoteResult();

    if (NoteDet.hasTargets()) {
      return -NoteDet.getBestTarget().getYaw() + 10;
    } else return 0.0;
  }

  public boolean cameraSeesNote() {
    return getLatestNoteResult().hasTargets();
  }
}
