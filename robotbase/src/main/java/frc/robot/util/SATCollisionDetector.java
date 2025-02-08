// MIT License

// Copyright (c) 2018 Joel Eager

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package frc.robot.util;

import static frc.robot.Constants.ROBOT_CONFIGURATION.*;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SATCollisionDetector {

  public static class SATVector {

    final double x, y;

    public SATVector(double x, double y) {
      this.x = x;
      this.y = y;
    }

    public SATVector(Pose2d poseToVector) {
      this.x = poseToVector.getX();
      this.y = poseToVector.getY();
    }

    /**
     * Returns a new vector which is orthogonal to the current vector
     */
    SATVector orthogonal() {
      return new SATVector(y, -x);
    }
  }

  /**
   * Takes a {@link Pose2d} object and returns a list of {@link SATVector SATVectors} that represents the robots current pose as a shape,
   * @param robotPose
   * @return
   */
  public static SATVector[] makePolyFromRobotPose(Pose2d robotPose) {
    SATVector[] vectorizedPoly = {
      new SATVector(robotPose.transformBy(FRONT_LEFT_CORNER_TRANSFORM)),
      new SATVector(robotPose.transformBy(FRONT_RIGHT_CORNER_TRANSFORM)),
      new SATVector(robotPose.transformBy(BACK_RIGHT_CORNER_TRANSFORM)),
      new SATVector(robotPose.transformBy(BACK_LEFT_CORNER_TRANSFORM)),
    };
    return vectorizedPoly;
  }

  /**
   * Returns whether or not a shape will collide with another shape. Giving a max distance is HIGHLY reccommended.
   * @param poly1 The first shape, which is supposed to be the moving shape
   * @param poly2 The second shape, which is supposed to be the shape your checking if your hitting
   * @param maxDist The maximum distance you can be away from the shape to not hit it
   * @return
   */
  public static Boolean hasCollided(
    SATVector[] poly1,
    SATVector[] poly2,
    Double maxDist
  ) {
    // Do an optimization check using the maxDist
    if (maxDist != null) {
      if (
        Math.pow(poly1[1].x - poly2[0].x, 2) +
          Math.pow(poly1[1].y - poly2[0].y, 2) <=
        Math.pow(maxDist, 2)
      ) {
        // Collision is possible so run SAT on the polys
        return runSAT(poly1, poly2);
      } else {
        return false;
      }
    } else {
      // No maxDist so run SAT on the polys
      return runSAT(poly1, poly2);
    }
  }

  private static Boolean runSAT(SATVector[] poly1, SATVector[] poly2) {
    // Implements the actual SAT algorithm
    ArrayList<SATVector> edges = polyToEdges(poly1);
    edges.addAll(polyToEdges(poly2));
    SATVector[] axes = new SATVector[edges.size()];
    for (int i = 0; i < edges.size(); i++) {
      axes[i] = edges.get(i).orthogonal();
    }

    for (SATVector axis : axes) {
      if (!overlap(project(poly1, axis), project(poly2, axis))) {
        // The polys don't overlap on this axis so they can't be touching
        return false;
      }
    }

    // The polys overlap on all axes so they must be touching
    return true;
  }

  /**
   * Returns a vector going from point1 to point2
   */
  private static SATVector edgeVector(SATVector point1, SATVector point2) {
    return new SATVector(point2.x - point1.x, point2.y - point1.y);
  }

  /**
   * Returns an array of the edges of the poly as vectors
   */
  private static ArrayList<SATVector> polyToEdges(SATVector[] poly) {
    ArrayList<SATVector> vectors = new ArrayList<>(poly.length);
    for (int i = 0; i < poly.length; i++) {
      vectors.add(edgeVector(poly[i], poly[(i + 1) % poly.length]));
    }
    return vectors;
  }

  /**
   * Returns the dot (or scalar) product of the two vectors
   */
  private static double dotProduct(SATVector vector1, SATVector vector2) {
    return vector1.x * vector2.x + vector1.y * vector2.y;
  }

  /**
   * Returns a vector showing how much of the poly lies along the axis
   */
  private static SATVector project(SATVector[] poly, SATVector axis) {
    List<Double> dots = new ArrayList<>();
    for (SATVector vector : poly) {
      dots.add(dotProduct(vector, axis));
    }
    return new SATVector(Collections.min(dots), Collections.max(dots));
  }

  /**
   * Returns a boolean indicating if the two projections overlap
   */
  private static boolean overlap(SATVector projection1, SATVector projection2) {
    return projection1.x <= projection2.y && projection2.x <= projection1.y;
  }
}
