/*
 This is public domain software, however it is preferred
 that the following disclaimers be attached.

 Software Copywrite/Warranty Disclaimer

 This software was developed at the National Institute of Standards and
 Technology by employees of the Federal Government in the course of their
 official duties. Pursuant to title 17 Section 105 of the United States
 Code this software is not subject to copyright protection and is in the
 public domain. This software is experimental. NIST assumes no responsibility
 whatsoever for its use by other parties, and makes no guarantees, expressed
 or implied, about its quality, reliability, or any other characteristic.

 We would appreciate acknowledgement if the software is used. This software can
 be redistributed and/or modified freely provided that any derivative works
 bear some notice that they are derived from it, and any modified
 versions bear some notice that they have been modified.

 */
package splinetest;

import java.awt.geom.Path2D;
import java.util.LinkedList;
import java.util.List;

/**
 * Class representing a path planned around obstacles and boundaries from a
 * given startPoint to a given goalPoint.
 *
 * @author Will Shackleford<shackle@nist.gov>
 */
public class PlannedPath {

    private CarrierState startPoint;
    private CarrierState goalPoint;
    private List<PlannerPoint> controlPoints;
    private List<Point2Dd> curvePoints;
    private List<Line2Dd> outlines;
    private Path2D.Double controlPath;
    private Path2D.Double curvePath;
    private Path2D.Double curveRightPath;
    private Path2D.Double curveLeftPath;
    private SplinePanel splinePanel;
    private PlannerInput plannerInput;

    public PlannerInput getPlannerInput() {
        return plannerInput;
    }

    public void setPlannerInput(PlannerInput pi) {
        this.plannerInput = pi;
    }

    public PlannedPath(SplinePanel _splinePanel) {
        this.splinePanel = _splinePanel;
    }
    private double PrePostPathDist = 10.0;

    /**
     * Get the value of PrePostPathDist
     *
     * @return the value of PrePostPathDist
     */
    public double getPrePostPathDist() {
        return PrePostPathDist;
    }

    /**
     * Set the value of PrePostPathDist
     *
     * @param PrePostPathDist new value of PrePostPathDist
     */
    public void setPrePostPathDist(double PrePostPathDist) {
        this.PrePostPathDist = PrePostPathDist;
    }

    private LinkedList<Point2Dd> extendedControlPoints;
    
    public LinkedList<Point2Dd> getExtendedControlPoints() {
        return extendedControlPoints;
    }
    
    public void updateCurvesPathsOutlines() {
        if (null != this.controlPoints) {
            extendedControlPoints = new LinkedList<>();
            extendedControlPoints.addAll(this.controlPoints);
            if (!this.splinePanel.isCrab()) {
                extendedControlPoints.addFirst(
                        Point2Dd.addPtDistAngle(startPoint, -PrePostPathDist, startPoint.getAngle()));
                extendedControlPoints.add(
                        Point2Dd.addPtDistAngle(goalPoint, +PrePostPathDist, goalPoint.getAngle()));
            }
            List<Point2Dd> newCurvePoints = BSplineCreator.createBSpline(this.splinePanel.getMinCurveIterations(),
                    this.splinePanel.getMaxCurveIterations(),
                    this.splinePanel.getCurveIterationDist(),
                    extendedControlPoints);
            this.setCurvePoints(newCurvePoints);
            Path2D.Double newControlPath = new Path2D.Double();
            boolean first = true;
            for (Point2Dd pt2d : this.controlPoints) {
                if (first) {
                    newControlPath.moveTo(pt2d.x, pt2d.y);
                    first = false;
                } else {
                    newControlPath.lineTo(pt2d.x, pt2d.y);
                }
            }
            this.setControlPath(newControlPath);
        }
    }

    /**
     * @return the startPoint
     */
    public CarrierState getStartPoint() {
        return startPoint;
    }

    /**
     * @param startPoint the startPoint to set
     */
    public void setStartPoint(CarrierState startPoint) {
        
        this.startPoint = startPoint;
        this.startVector = new Point2Dd(startPoint.getAngle().cos(), startPoint.getAngle().sin());
        this.startPoint.setPath(this);
    }

    /**
     * @return the goalPoint
     */
    public CarrierState getGoalPoint() {
        return goalPoint;
    }

    /**
     * @param goalPoint the goalPoint to set
     */
    public void setGoalPoint(CarrierState goalPoint) {
        this.goalPoint = goalPoint;
    }

    /**
     * @return the controlPoints
     */
    public List<PlannerPoint> getControlPoints() {
        return controlPoints;
    }

    /**
     * @param controlPoints the controlPoints to set
     */
    public void setControlPoints(List<PlannerPoint> controlPoints) {
        this.controlPoints = controlPoints;
        this.updateCurvesPathsOutlines();
    }

    /**
     * @return the curvePoints
     */
    public List<Point2Dd> getCurvePoints() {
        return curvePoints;
    }
    private Point2Dd startVector;
    List<Boundary> planOutline = null;

    public List<Boundary> getPlanOutLine() {
        if (null == planOutline) {
            planOutline = this.createPlanOutLine(null,
                    this.splinePanel.getOutlineDistIncrement(),
                    Math.PI / 12.0);
        }
        return planOutline;
    }

    public void setPlanOutline(List<Boundary> _planOutline) {
        this.planOutline = _planOutline;
    }

    public List<Boundary> createPlanOutLine(Point2Dd st_pt,
            double dist_increment,
            double angle_increment) {
        int closest_pt_index = -1;
        double min_dist = Double.POSITIVE_INFINITY;
        if (null == this.curvePoints) {
            return null;
        }
        if (st_pt != null) {
            for (int i = 0; i < this.curvePoints.size(); i++) {
                Point2Dd ptd = this.curvePoints.get(i);
                double dist = ptd.distance(st_pt);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_pt_index = i;
                }
            }
            if (closest_pt_index < 0 || closest_pt_index > this.curvePoints.size() - 3) {
                return null;
            }
        } else {
            closest_pt_index = 0;
        }
        List<Boundary> planOutline = new LinkedList<>();
        Point2Dd last_pt = this.curvePoints.get(closest_pt_index);
        Point2Dd last_outline_pt = null;
        double last_outline_angle = Double.POSITIVE_INFINITY;
        Point2Dd diff = new Point2Dd();
        Point2Dd diffu = new Point2Dd();
        boolean is_crab = this.splinePanel.isCrab();
        double vehicleWidth = this.splinePanel.getVehicleWidth();
        Point2Dd last_front_leftpt = null;
        Point2Dd last_front_rightpt = null;
        Point2Dd last_back_leftpt = null;
        Point2Dd last_back_rightpt = null;
        double front = this.splinePanel.getVehicleFront();
        double back = this.splinePanel.getVehicleBack();

        for (int i = closest_pt_index + 1; i < this.curvePoints.size(); i++) {
            Point2Dd pt2d = this.curvePoints.get(i);
            diff.x = pt2d.x - last_pt.x;
            diff.y = pt2d.y - last_pt.y;
            double angle = Math.atan2(diffu.y, diffu.x);
            double dist = Double.POSITIVE_INFINITY;
            if (null != last_outline_pt) {
                dist = pt2d.distance(last_outline_pt);
            }
            diffu = diff.unit();
            if (null != last_outline_pt
                    && i != this.curvePoints.size() - 1
                    && (dist < dist_increment
                    || Math.abs(angle - last_outline_angle) < angle_increment)) {
                last_pt = pt2d;
                continue;
            }
            if (is_crab) {
                diffu = startVector;
            }
            Point2Dd leftpt = new Point2Dd();
            Point2Dd rightpt = new Point2Dd();
            AngleD leftA = new AngleD(+Math.PI / 2);
            AngleD rightA = new AngleD(-Math.PI / 2);
            leftpt.x = (float) (pt2d.x - diffu.y * vehicleWidth / 2f);
            leftpt.y = (float) (pt2d.y + diffu.x * vehicleWidth / 2f);
            rightpt.x = (float) (pt2d.x + diffu.y * vehicleWidth / 2f);
            rightpt.y = (float) (pt2d.y - diffu.x * vehicleWidth / 2f);
            Point2Dd front_leftpt = new Point2Dd();
            Point2Dd front_rightpt = new Point2Dd();
            front_leftpt.x = (float) (leftpt.x + diffu.x * front);
            front_leftpt.y = (float) (leftpt.y + diffu.y * front);
            front_rightpt.x = (float) (rightpt.x + diffu.x * front);
            front_rightpt.y = (float) (rightpt.y + diffu.y * front);
            Point2Dd back_leftpt = new Point2Dd();
            Point2Dd back_rightpt = new Point2Dd();
            back_leftpt.x = (float) (leftpt.x - diffu.x * back);
            back_leftpt.y = (float) (leftpt.y - diffu.y * back);
            back_rightpt.x = (float) (rightpt.x - diffu.x * back);
            back_rightpt.y = (float) (rightpt.y - diffu.y * back);
            if (last_outline_pt == null || i == this.curvePoints.size() - 1) {
                planOutline.add(new Boundary(back_leftpt, front_leftpt));
                planOutline.add(new Boundary(back_rightpt, front_rightpt));
                planOutline.add(new Boundary(back_leftpt, back_rightpt));
                planOutline.add(new Boundary(front_leftpt, front_rightpt));
            } 
            if(null != last_outline_pt) {
                planOutline.add(new Boundary(last_front_leftpt, front_leftpt));
                planOutline.add(new Boundary(last_front_rightpt, front_rightpt));
                if (is_crab) {
                    planOutline.add(new Boundary(last_back_leftpt, back_leftpt));
                    planOutline.add(new Boundary(last_back_rightpt, back_rightpt));
                }
            }
            last_outline_angle = angle;
            last_outline_pt = pt2d;
            last_front_leftpt = front_leftpt;
            last_front_rightpt = front_rightpt;
            last_back_leftpt = back_leftpt;
            last_back_rightpt = back_rightpt;
            last_pt = pt2d;
            last_outline_pt = pt2d;
        }
//        System.out.println("planOutline.size() = " + planOutline.size());
        return planOutline;
    }

    /**
     * @param curvePoints the curvePoints to set
     */
    public void setCurvePoints(List<Point2Dd> curvePoints) {
        this.curvePoints = curvePoints;
        this.curvePath = null;
        this.curveLeftPath = null;
        this.curveRightPath = null;
        this.outlines = null;
        if (this.curvePoints != null) {
            this.curvePath = new Path2D.Double();
            Point2Dd last_pt = null;
            Point2Dd last_outline_pt = null;
            double last_outline_angle = Double.POSITIVE_INFINITY;
            LinkedList<Line2Dd> newOutlines = new LinkedList<>();
            Path2D.Double newCurveLeftPath = new Path2D.Double();
            Path2D.Double newCurveRightPath = new Path2D.Double();
            Point2Dd diff = new Point2Dd();
            Point2Dd diffu = new Point2Dd();
            boolean first = true;
            boolean firstlr = true;
            Point2Dd orig_diffu = null;
            boolean is_crab = this.splinePanel.isCrab();
            double vehicleWidth = this.splinePanel.getVehicleWidth();
            double front = this.splinePanel.getVehicleFront();
            double back = this.splinePanel.getVehicleBack();

            for (Point2Dd pt2d : this.curvePoints) {
                if (first) {
                    last_pt = pt2d;
                    curvePath.moveTo(pt2d.x, pt2d.y);
                    first = false;
                } else {
                    curvePath.lineTo(pt2d.x, pt2d.y);
                    diff.x = pt2d.x - last_pt.x;
                    diff.y = pt2d.y - last_pt.y;
                    double diff_mag = (double) diff.distance(0, 0);
                    if (diff_mag < 0.001) {
                        continue;
                    }

                    diffu = diff.unit();
                    if (is_crab) {
                        diffu = startVector;
                    }
                    Point2Dd leftpt = new Point2Dd();
                    Point2Dd rightpt = new Point2Dd();
                    AngleD leftA = new AngleD(+Math.PI / 2);
                    AngleD rightA = new AngleD(-Math.PI / 2);
                    leftpt.x = (float) (pt2d.x - diffu.y * vehicleWidth / 2f);
                    leftpt.y = (float) (pt2d.y + diffu.x * vehicleWidth / 2f);
                    rightpt.x = (float) (pt2d.x + diffu.y * vehicleWidth / 2f);
                    rightpt.y = (float) (pt2d.y - diffu.x * vehicleWidth / 2f);
                    Point2Dd front_leftpt = new Point2Dd();
                    Point2Dd front_rightpt = new Point2Dd();
                    front_leftpt.x = (float) (leftpt.x + diffu.x * front);
                    front_leftpt.y = (float) (leftpt.y + diffu.y * front);
                    front_rightpt.x = (float) (rightpt.x + diffu.x * front);
                    front_rightpt.y = (float) (rightpt.y + diffu.y * front);
                    Point2Dd back_leftpt = new Point2Dd();
                    Point2Dd back_rightpt = new Point2Dd();
                    back_leftpt.x = (float) (leftpt.x - diffu.x * back);
                    back_leftpt.y = (float) (leftpt.y - diffu.y * back);
                    back_rightpt.x = (float) (rightpt.x - diffu.x * back);
                    back_rightpt.y = (float) (rightpt.y - diffu.y * back);
                    double angle = Math.atan2(diffu.y, diffu.x);
                    if (null == last_outline_pt
                            || pt2d.distance(last_outline_pt) > this.splinePanel.getOutlineDistIncrement()
                            || Math.abs(angle - last_outline_angle) > Math.PI / 36.0) {
                        newOutlines.add(new Line2Dd(back_leftpt, front_leftpt));
                        newOutlines.add(new Line2Dd(back_rightpt, front_rightpt));
                        newOutlines.add(new Line2Dd(back_leftpt, back_rightpt));
                        newOutlines.add(new Line2Dd(front_leftpt, front_rightpt));
                        last_outline_angle = angle;
                        last_outline_pt = pt2d;
                    }
                    last_pt = pt2d;
                    if (firstlr) {
                        newCurveLeftPath.moveTo(leftpt.x, leftpt.y);
                        newCurveRightPath.moveTo(rightpt.x, rightpt.y);
                        firstlr = false;
                    } else {
                        newCurveLeftPath.lineTo(leftpt.x, leftpt.y);
                        newCurveRightPath.lineTo(rightpt.x, rightpt.y);
                    }
                }
            }
            this.setCurveLeftPath(newCurveLeftPath);
            this.setCurveRightPath(newCurveRightPath);
            this.setOutlines(newOutlines);
        }
    }

    public boolean checkOutlines(List<Boundary> _boundaries,
            List<Obstacle> _obstacles) {
        if (null != this.outlines) {
            for (Line2Dd l : this.outlines) {
                if (null != _boundaries) {
                    for (Boundary b : _boundaries) {
                        if (Double.isNaN(b.x1) || Double.isInfinite(b.x1)
                                || Double.isNaN(b.x2) || Double.isInfinite(b.x2)
                                || Double.isNaN(b.y1) || Double.isInfinite(b.y1)
                                || Double.isNaN(b.y2) || Double.isInfinite(b.y2)) {
                            continue;
                        }
                        if (b.intersectsLine(l)) {
                            return false;
                        }
                    }
                }
                if (null != _obstacles) {
                    for (Obstacle o : _obstacles) {
                        if (l.ptSegDist(o) < o.radius) {
                            return false;
                        }
                    }
                }
            }
        }
        return true;
    }

    /**
     * @return the outlines
     */
    public List<Line2Dd> getOutlines() {
        return outlines;
    }

    /**
     * @param outlines the outlines to set
     */
    public void setOutlines(List<Line2Dd> outlines) {
        this.outlines = outlines;
    }

    /**
     * @return the curvePath
     */
    public Path2D.Double getCurvePath() {
        return curvePath;
    }

    /**
     * @param curvePath the curvePath to set
     */
    public void setCurvePath(Path2D.Double curvePath) {
        this.curvePath = curvePath;
    }

    /**
     * @return the curveRightPath
     */
    public Path2D.Double getCurveRightPath() {
        return curveRightPath;
    }

    /**
     * @param curveRightPath the curveRightPath to set
     */
    public void setCurveRightPath(Path2D.Double curveRightPath) {
        this.curveRightPath = curveRightPath;
    }

    /**
     * @return the curveLeftPath
     */
    public Path2D.Double getCurveLeftPath() {
        return curveLeftPath;
    }

    /**
     * @param curveLeftPath the curveLeftPath to set
     */
    public void setCurveLeftPath(Path2D.Double curveLeftPath) {
        this.curveLeftPath = curveLeftPath;
    }

    /**
     * @return the controlPath
     */
    public Path2D.Double getControlPath() {
        return controlPath;
    }

    /**
     * @param controlPath the controlPath to set
     */
    public void setControlPath(Path2D.Double controlPath) {
        this.controlPath = controlPath;
    }
}
