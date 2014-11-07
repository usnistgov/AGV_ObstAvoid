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

import diagapplet.plotter.PlotData;
import diagapplet.plotter.plotterJFrame;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.InputEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.BufferedReader;
import java.io.StringReader;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.ConcurrentModificationException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.SynchronousQueue;
import javax.swing.JMenu;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import static splinetest.SplineDrawMode.BOUNDARY;
import static splinetest.SplineDrawMode.GOAL;
import static splinetest.SplineDrawMode.HORZ_BOUNDARY;
import static splinetest.SplineDrawMode.OBSTACLE;
import static splinetest.SplineDrawMode.SELECT;
import static splinetest.SplineDrawMode.START;
import static splinetest.SplineDrawMode.VERT_BOUNDARY;
import static splinetest.SplineDrawMode.WAYPOINT;

/**
 * Main JPanel for displaying and drawing or selecting objects in the 2D world
 * include vehicles, obstacles, boundaries and planned splines.
 *
 * @author Will Shackleford<shackle@nist.gov>
 */
public class SplinePanel extends JPanel implements MouseListener, MouseMotionListener {

    public SplinePanel() {
        this.addMouseListener(this);
        this.addMouseMotionListener(this);
    }
    private SplineDrawMode drawMode = SplineDrawMode.START;
    private List<CarrierState> startPoints;
    private List<CarrierState> goalPoints;
    private List<PlannedPath> plannedPaths;
    private boolean DebugVehicleComm;
    private int maxCntrlPts = -1;
    private List<CarrierState> waypoints;
    private boolean ignoreBoundaries = false;

    /**
     * Get the value of ignoreBoundaries
     *
     * @return the value of ignoreBoundaries
     */
    public boolean isIgnoreBoundaries() {
        return ignoreBoundaries;
    }

    /**
     * Set the value of ignoreBoundaries
     *
     * @param ignoreBoundaries new value of ignoreBoundaries
     */
    public void setIgnoreBoundaries(boolean ignoreBoundaries) {
        this.ignoreBoundaries = ignoreBoundaries;
    }
    private boolean ignoreObstacles = false;

    /**
     * Get the value of ignoreObstacles
     *
     * @return the value of ignoreObstacles
     */
    public boolean isIgnoreObstacles() {
        return ignoreObstacles;
    }

    /**
     * Set the value of ignoreObstacles
     *
     * @param ignoreObstacles new value of ignoreObstacles
     */
    public void setIgnoreObstacles(boolean ignoreObstacles) {
        this.ignoreObstacles = ignoreObstacles;
    }

    /**
     * Get the value of waypoints
     *
     * @return the value of waypoints
     */
    public List<CarrierState> getWaypoints() {
        return waypoints;
    }

    /**
     * Set the value of waypoints
     *
     * @param waypoints new value of waypoints
     */
    public void setWaypoints(List<CarrierState> waypoints) {
        this.waypoints = waypoints;
    }

    /**
     * Get the value of maxCntrlPts
     *
     * @return the value of maxCntrlPts
     */
    public int getMaxCntrlPts() {
        return maxCntrlPts;
    }

    /**
     * Set the value of maxCntrlPts
     *
     * @param maxCntrlPts new value of maxCntrlPts
     */
    public void setMaxCntrlPts(int maxCntrlPts) {
        this.maxCntrlPts = maxCntrlPts;
    }

    /**
     * Get the value of DebugVehicleComm
     *
     * @return the value of DebugVehicleComm
     */
    public boolean isDebugVehicleComm() {
        return DebugVehicleComm;
    }

    /**
     * Set the value of DebugVehicleComm
     *
     * @param DebugVehicleComm new value of DebugVehicleComm
     */
    public void setDebugVehicleComm(boolean DebugVehicleComm) {
        this.DebugVehicleComm = DebugVehicleComm;
    }

    /**
     * Get the value of drawMode
     *
     * @return the value of drawMode
     */
    public SplineDrawMode getDrawMode() {
        return drawMode;
    }

    public void clearTmpGoal() {
        if (null != this.tmpGoal) {
            if (null != this.startPoints && null == this.plan_thread) {
                for (CarrierState sp : this.startPoints) {
                    if (sp.getGoal() == tmpGoal) {
                        sp.setGoal(null);
                        sp.setPlannerList(null);
                        sp.updateShape();
                    }
                }
            }
            this.tmpGoal = null;
            this.repaint();
        }
    }

    /**
     * Set the value of drawMode
     *
     * @param drawMode new value of drawMode
     */
    public void setDrawMode(SplineDrawMode drawMode) {
        this.drawMode = drawMode;
        this.clearTmpGoal();
        this.ClearSelection();
    }
    private int MinCurveIterations = 5;

    /**
     * Get the value of MinCurveIterations
     *
     * @return the value of MinCurveIterations
     */
    public int getMinCurveIterations() {
        return MinCurveIterations;
    }

    /**
     * Set the value of MinCurveIterations
     *
     * @param MinCurveIterations new value of MinCurveIterations
     */
    public void setMinCurveIterations(int MinCurveIterations) {
        this.MinCurveIterations = MinCurveIterations;
    }
    private int MaxCurveIterations = 500;

    /**
     * Get the value of MaxCurveIterations
     *
     * @return the value of MaxCurveIterations
     */
    public int getMaxCurveIterations() {
        return MaxCurveIterations;
    }

    /**
     * Set the value of MaxCurveIterations
     *
     * @param MaxCurveIterations new value of MaxCurveIterations
     */
    public void setMaxCurveIterations(int MaxCurveIterations) {
        this.MaxCurveIterations = MaxCurveIterations;
    }
    private boolean showPlanOutline;

    /**
     * Get the value of showPlanOutline
     *
     * @return the value of showPlanOutline
     */
    public boolean isShowPlanOutline() {
        return showPlanOutline;
    }

    /**
     * Set the value of showPlanOutline
     *
     * @param showPlanOutline new value of showPlanOutline
     */
    public void setShowPlanOutline(boolean showPlanOutline) {
        this.showPlanOutline = showPlanOutline;
        this.repaint();
    }
    private double CurveIterationDist = 100.0;

    /**
     * Get the value of CurveIterationDist
     *
     * @return the value of CurveIterationDist
     */
    public double getCurveIterationDist() {
        return CurveIterationDist;
    }

    /**
     * Set the value of CurveIterationDist
     *
     * @param CurveIterationDist new value of CurveIterationDist
     */
    public void setCurveIterationDist(double CurveIterationDist) {
        this.CurveIterationDist = CurveIterationDist;
    }
    private Point2Dd[] CurveLeftPoints;

    /**
     * Get the value of CurveLeftPoints
     *
     * @return the value of CurveLeftPoints
     */
    public Point2Dd[] getCurveLeftPoints() {
        return CurveLeftPoints;
    }
    private Path2D.Double curveLeftPath;

    /**
     * Set the value of CurveLeftPoints
     *
     * @param CurveLeftPoints new value of CurveLeftPoints
     */
    public void setCurveLeftPoints(Point2Dd[] CurveLeftPoints) {
        this.CurveLeftPoints = CurveLeftPoints;
        boolean first = true;
        this.curveLeftPath = null;
        if (null != this.CurveLeftPoints) {
            this.curveLeftPath = new Path2D.Double();
            for (Point2Dd pt2d : this.CurveLeftPoints) {
                if (first) {
                    curveLeftPath.moveTo(pt2d.x, pt2d.y);
                    first = false;
                } else {
                    curveLeftPath.lineTo(pt2d.x, pt2d.y);
                }
            }
        }
    }
    private boolean showPlanning;

    /**
     * Get the value of showPlanning
     *
     * @return the value of showPlanning
     */
    public boolean isShowPlanning() {
        return showPlanning;
    }

    /**
     * Set the value of showPlanning
     *
     * @param showPlanning new value of showPlanning
     */
    public void setShowPlanning(boolean showPlanning) {
        this.showPlanning = showPlanning;
        this.repaint();
    }
    public static final double DEFAULT_VEHICLE_WIDTH = 100.0;

    /**
     * Get the value of CurveLeftPoints at specified index
     *
     * @param index
     * @return the value of CurveLeftPoints at specified index
     */
    public Point2Dd getCurveLeftPoints(int index) {
        return this.CurveLeftPoints[index];
    }
    private double vehicleWidth = DEFAULT_VEHICLE_WIDTH;

    /**
     * Get the value of vehicleWidth
     *
     * @return the value of vehicleWidth
     */
    public double getVehicleWidth() {
        return vehicleWidth;
    }

    /**
     * Set the value of vehicleWidth
     *
     * @param vehicleWidth new value of vehicleWidth
     */
    public void setVehicleWidth(double vehicleWidth) {
        this.vehicleWidth = vehicleWidth;
        if (null != this.goalPoints) {
            for (CarrierState cs : this.goalPoints) {
                cs.setWidth(this.vehicleWidth);
            }
        }
        if (null != this.startPoints) {
            for (CarrierState cs : this.startPoints) {
                cs.setWidth(this.vehicleWidth);
            }
        }
        if (null != this.waypoints) {
            for (CarrierState cs : this.waypoints) {
                cs.setWidth(this.vehicleWidth);
            }
        }
        if (null != this.carriers) {
            for (CarrierState cs : this.carriers) {
                cs.setWidth(this.vehicleWidth);
            }
        }
        this.repaint();
    }
    private double defaultObsRadius = 75.0;
    private Point2Dd[] CurveRightPoints;

    /**
     * Get the value of CurveRightPoints
     *
     * @return the value of CurveRightPoints
     */
    public Point2Dd[] getCurveRightPoints() {
        return CurveRightPoints;
    }
    private Path2D.Double curveRightPath;

    /**
     * the value of CurveRightPoints
     *
     * @param CurveRightPoints new value of CurveRightPoints
     */
    public void setCurveRightPoints(Point2Dd[] CurveRightPoints) {
        this.CurveRightPoints = CurveRightPoints;
        boolean first = true;
        this.curveRightPath = null;
        if (null != this.CurveRightPoints) {
            this.curveRightPath = new Path2D.Double();
            for (Point2Dd pt2d : this.CurveRightPoints) {
                if (first) {
                    curveRightPath.moveTo(pt2d.x, pt2d.y);
                    first = false;
                } else {
                    curveRightPath.lineTo(pt2d.x, pt2d.y);
                }
            }
        }
    }

    /**
     * Get the value of CurveRightPoints at specified index
     *
     * @param index
     * @return the value of CurveRightPoints at specified index
     */
    public Point2Dd getCurveRightPoints(int index) {
        return this.CurveRightPoints[index];
    }

    /**
     * Set the value of CurveRightPoints at specified index.
     *
     * @param index
     * @param newCurveRightPoints new value of CurveRightPoints at specified
     * index
     */
    public void setCurveRightPoints(int index, Point2Dd newCurveRightPoints) {
        this.CurveRightPoints[index] = newCurveRightPoints;
    }

    /**
     * Set the value of CurveLeftPoints at specified index.
     *
     * @param index
     * @param newCurveLeftPoints new value of CurveLeftPoints at specified index
     */
    public void setCurveLeftPoints(int index, Point2Dd newCurveLeftPoints) {
        this.CurveLeftPoints[index] = newCurveLeftPoints;
    }
    private List<Point2Dd> CurvePoints;
    private boolean ShowOutline = true;

    /**
     * Get the value of ShowOutline
     *
     * @return the value of ShowOutline
     */
    public boolean isShowOutline() {
        return ShowOutline;
    }

    /**
     * Set the value of ShowOutline
     *
     * @param ShowOutline new value of ShowOutline
     */
    public void setShowOutline(boolean ShowOutline) {
        this.ShowOutline = ShowOutline;
        this.repaint();
    }
    //private LinkedList<Line2Dd> outlines;
    public static final double DEFAULT_VEHICLE_FRONT = 90.0;
    public static final double DEFAULT_VEHICLE_BACK = 200.0;
    private double vehicleFront = DEFAULT_VEHICLE_FRONT;
    private double vehicleBack = DEFAULT_VEHICLE_BACK;

    /**
     * Set the value of CurvePoints
     *
     * @param CurvePoints new value of CurvePoints
     */
    public void setCurvePoints(List<Point2Dd> CurvePoints) {
        List<Point2Dd> oldCurvePoints = this.CurvePoints;
        this.curvePath = null;
        this.curveLeftPath = null;
        this.curveRightPath = null;
        this.CurvePoints = CurvePoints;
        this.setCurveLeftPoints(null);
        this.setCurveRightPoints(null);
        if (this.CurvePoints != null) {
            this.curvePath = new Path2D.Double();
            Point2Dd last_pt = null;
            LinkedList<Point2Dd> NewCurveLeftPoints
                    = new LinkedList<>();
            LinkedList<Point2Dd> NewCurveRightPoints
                    = new LinkedList<>();
//            outlines = new LinkedList<>();
            Point2Dd diff = new Point2Dd();
            Point2Dd diffu = new Point2Dd();
            int lrindex = 0;
            boolean first = true;
            for (Point2Dd pt2d : this.CurvePoints) {
                if (first) {
                    last_pt = pt2d;
                    curvePath.moveTo(pt2d.x, pt2d.y);
                    first = false;
                } else {
                    curvePath.lineTo(pt2d.x, pt2d.y);
                    diff.x = pt2d.x - last_pt.x;
                    diff.y = pt2d.y - last_pt.y;
                    float diff_mag = diff.mag();
                    if (diff_mag < 0.001) {
                        continue;
                    }
                    diffu.x = diff.x / diff_mag;
                    diffu.y = diff.y / diff_mag;
                    Point2Dd leftpt = new Point2Dd();
                    Point2Dd rightpt = new Point2Dd();
                    leftpt.x = pt2d.x - diffu.y * vehicleWidth / 2f;
                    leftpt.y = pt2d.y + diffu.x * vehicleWidth / 2f;
                    rightpt.x = pt2d.x + diffu.y * vehicleWidth / 2f;
                    rightpt.y = pt2d.y - diffu.x * vehicleWidth / 2f;
                    Point2Dd front_leftpt = new Point2Dd();
                    Point2Dd front_rightpt = new Point2Dd();
                    front_leftpt.x = leftpt.x + diffu.x * getVehicleFront();
                    front_leftpt.y = leftpt.y + diffu.y * getVehicleFront();
                    front_rightpt.x = rightpt.x + diffu.x * getVehicleFront();
                    front_rightpt.y = rightpt.y + diffu.y * getVehicleFront();
                    Point2Dd back_leftpt = new Point2Dd();
                    Point2Dd back_rightpt = new Point2Dd();
                    back_leftpt.x = leftpt.x - diffu.x * getVehicleBack();
                    back_leftpt.y = leftpt.y - diffu.y * getVehicleBack();
                    back_rightpt.x = rightpt.x - diffu.x * getVehicleBack();
                    back_rightpt.y = rightpt.y - diffu.y * getVehicleBack();
//                    outlines.add(new Line2Dd(back_leftpt, front_leftpt));
//                    outlines.add(new Line2Dd(back_rightpt, front_rightpt));
//                    outlines.add(new Line2Dd(back_leftpt, back_rightpt));
//                    outlines.add(new Line2Dd(front_leftpt, front_rightpt));
                    NewCurveLeftPoints.add(leftpt);
                    NewCurveRightPoints.add(rightpt);
                    lrindex++;
                    last_pt = pt2d;
                }
            }
            this.setCurveLeftPoints(NewCurveLeftPoints.toArray(new Point2Dd[NewCurveLeftPoints.size()]));
            this.setCurveRightPoints(NewCurveRightPoints.toArray(new Point2Dd[NewCurveRightPoints.size()]));
        }
    }
    private boolean ExclusivePaths;

    /**
     * Get the value of ExclusivePaths
     *
     * @return the value of ExclusivePaths
     */
    public boolean isExclusivePaths() {
        return ExclusivePaths;
    }

    /**
     * Set the value of ExclusivePaths
     *
     * @param ExclusivePaths new value of ExclusivePaths
     */
    public void setExclusivePaths(boolean ExclusivePaths) {
        this.ExclusivePaths = ExclusivePaths;
        this.replan();
    }
    private double right_offset;

    /**
     * Get the value of right_offset
     *
     * @return the value of right_offset
     */
    public double getRight_offset() {
        return right_offset;
    }

    /**
     * Set the value of right_offset
     *
     * @param right_offset new value of right_offset
     */
    public void setRight_offset(double right_offset) {
        this.right_offset = right_offset;
    }

    public void AddObstacle(Obstacle pt) {
        if (null == this.obstacles) {
            this.obstacles = new LinkedList<>();
        }
        this.obstacles.add(pt);
        this.setObstacles(obstacles);
    }

    public double computeMinDistToObs() {
        double min_dist_to_obs = java.lang.Double.POSITIVE_INFINITY;
        if (null == this.obstacles) {
            return java.lang.Double.POSITIVE_INFINITY;
        }
        if (null != this.CurveLeftPoints) {
            for (Point2Dd curve_pt : this.CurveLeftPoints) {
                for (Obstacle obs_pt : this.obstacles) {
                    double dist = curve_pt.distance(obs_pt);
                    if (dist < min_dist_to_obs) {
                        min_dist_to_obs = dist;
                    }
                }
            }
        }
        if (null != this.CurveRightPoints) {
            for (Point2Dd curve_pt : this.CurveRightPoints) {
                for (Obstacle obs_pt : this.obstacles) {
                    double dist = curve_pt.distance(obs_pt);
                    if (dist < min_dist_to_obs) {
                        min_dist_to_obs = dist;
                    }
                }
            }
        }
        return min_dist_to_obs;
    }
    private double max_pt2pt_dist = 100f;
    private double outlineDistIncrement = 50;

    /**
     * Get the value of outlineDistIncrement
     *
     * @return the value of outlineDistIncrement
     */
    public double getOutlineDistIncrement() {
        return outlineDistIncrement;
    }

    /**
     * Set the value of outlineDistIncrement
     *
     * @param outlineDistIncrement new value of outlineDistIncrement
     */
    public void setOutlineDistIncrement(double outlineDistIncrement) {
        this.outlineDistIncrement = outlineDistIncrement;
    }
    private List<Boundary> boundaries = new LinkedList<Boundary>();

    private Rectangle2Dd planningAreaRect = null;

    /**
     * Get the value of planningAreaRect
     *
     * @return the value of planningAreaRect
     */
    public Rectangle2Dd getPlanningAreaRect() {
        return planningAreaRect;
    }

    /**
     * Set the value of planningAreaRect
     *
     * @param planningAreaRect new value of planningAreaRect
     */
    public void setPlanningAreaRect(Rectangle2Dd planningAreaRect) {
        this.planningAreaRect = planningAreaRect;
    }

    public PlannerInput getPlannerInput() {
        PlannerInput pi = new PlannerInput();
        pi.rectB = this.getPlanningAreaRect();
        if (null == pi.rectB) {
            pi.rectB = this.getBoundingRect();
        }
        pi.use_static_planner_list = this.use_static_planner_list;
        pi.path_uncertainty = this.pathUncertainty;
        pi.obstacles = this.obstacles;
        pi.veh_width = this.vehicleWidth;
        pi.front = this.vehicleFront;
        pi.back = this.vehicleBack;
        pi.crab = this.crab;
        pi.reverse = this.reverse;
        pi.min_turn_radius = this.getMinTurnRadius();
        pi.max_turn_angle_degrees = this.getMaxTurnAngleDegrees();
        pi.max_pt2pt_dist = this.getMax_pt2pt_dist();
        pi.plannerResolution = this.getPlannerResolution();
        pi.max_cntrl_pts = this.getMaxCntrlPts();
        pi.planningHorizon = this.getPlanningHorizon();
        pi.segStartLength = this.getSegStartLength();
        if (this.ignoreBoundaries) {
            pi.boundaries = null;
        } else {
//                if (this.ExclusivePaths && null != this.boundaries) {
//                    fullBoundaries.addAll(this.boundaries);
//                    pi.boundaries = fullBoundaries;
//                } else {
            pi.boundaries = this.boundaries;
//                }
        }
        return pi;
    }

    /**
     * Get the value of boundaries
     *
     * @return the value of boundaries
     */
    public List<Boundary> getBoundaries() {
        return boundaries;
    }

    /**
     * Set the value of boundaries
     *
     * @param boundaries new value of boundaries
     */
    public void setBoundaries(List<Boundary> boundaries) {
        this.boundaries = boundaries;
        this.replan();
        this.repaint();
    }
    private double pathUncertainty = 1.0;

    /**
     * Get the value of pathUncertainty
     *
     * @return the value of pathUncertainty
     */
    public double getPathUncertainty() {
        return pathUncertainty;
    }

    /**
     * Set the value of pathUncertainty
     *
     * @param pathUncertainty new value of pathUncertainty
     */
    public void setPathUncertainty(double pathUncertainty) {
        this.pathUncertainty = pathUncertainty;
    }

    public void replan() {
        if (!this.replanOnAllChanges) {
            if (this.delay_replan_count > 0 || this.connectedToVehicle) {
                return;
            }
        }
        this.checkCollisions();
//        if (this.pathUncertainty < this.vehicleWidth / 10.0) {
//            this.setPathUncertainty(this.vehicleWidth / 10.0);
//        }
        while (!tryReplan());
        this.repaint();
    }
    private boolean crab;

    /**
     * Get the value of crab
     *
     * @return the value of crab
     */
    public boolean isCrab() {
        return crab;
    }

    /**
     * Set the value of crab
     *
     * @param crab new value of crab
     */
    public void setCrab(boolean crab) {
        this.crab = crab;
        this.replan();
        this.repaint();
    }
    private double plannerResolution = 20.0;

    /**
     * Get the value of plannerResolution
     *
     * @return the value of plannerResolution
     */
    public double getPlannerResolution() {
        return plannerResolution;
    }

    /**
     * Set the value of plannerResolution
     *
     * @param plannerResolution new value of plannerResolution
     */
    public void setPlannerResolution(double plannerResolution) {
        this.plannerResolution = plannerResolution;
    }
    private double speed = 25.0;

    /**
     * Get the value of speed
     *
     * @return the value of speed
     */
    public double getSpeed() {
        return speed;
    }

    /**
     * Set the value of speed
     *
     * @param speed new value of speed
     */
    public void setSpeed(double speed) {
        this.speed = speed;
    }
    private double maxTurnAngleDegrees = 75.0;

    /**
     * Get the value of maxTurnAngleDegrees
     *
     * @return the value of maxTurnAngleDegrees
     */
    public double getMaxTurnAngleDegrees() {
        return maxTurnAngleDegrees;
    }

    /**
     * Set the value of maxTurnAngleDegrees
     *
     * @param maxTurnAngleDegrees new value of maxTurnAngleDegrees
     */
    public void setMaxTurnAngleDegrees(double maxTurnAngleDegrees) {
        this.maxTurnAngleDegrees = maxTurnAngleDegrees;
    }
    Thread plan_thread = null;

    class PlanningArgs {

        final public List<PlannedPath> newPlannedPaths;
        final public boolean use_exclusive_paths;

        public PlanningArgs(final List<PlannedPath> _newPlannedPaths, final boolean _use_exclusive_paths) {
            this.newPlannedPaths = _newPlannedPaths;
            this.use_exclusive_paths = _use_exclusive_paths;
        }
    };
    BlockingQueue<PlanningArgs> planQueue = new SynchronousQueue<>();
    boolean showing_planner_msg = false;
    boolean planning = false;
    public boolean running_param_test = false;

    public void interruptPlanning() {
        try {
            if (null != plan_thread) {
                plan_thread.interrupt();;
                plan_thread.join();
                plan_thread = null;
                planning = false;
            }
        } catch (Exception exception) {
            exception.printStackTrace();
        }
    }

    private boolean tryReplan() {
        try {
            if (!this.replanOnAllChanges) {
                if (this.delay_replan_count > 0 || this.connectedToVehicle) {
                    return true;
                }
            }
            if (running_param_test) {
                return true;
            }
            if (planning) {
                return true;
            }
            if (!planQueue.isEmpty()) {
                return true;
            }
            if (showing_planner_msg) {
                return true;
            }
            final List<Boundary> fullBoundaries = new LinkedList<>();

            if ((null != this.startPoints
                    && null != this.goalPoints) || null != this.waypoints) {
                PlannerInput pi = this.getPlannerInput();
                if (this.ignoreBoundaries) {
                    pi.boundaries = null;
                } else {
                    if (this.ExclusivePaths && null != this.boundaries) {
                        fullBoundaries.addAll(this.boundaries);
                        pi.boundaries = fullBoundaries;
                    } else {
                        pi.boundaries = this.boundaries;
                    }
                }
                List<PlannerPoint> ll = Planner.createPlannerList(pi, this.startPoints, this.goalPoints, this.waypoints);
                final List<PlannedPath> newPlannedPaths = new LinkedList<>();
                if (null != this.startPoints) {
                    Collections.sort(this.startPoints, new Comparator<CarrierState>() {
                        @Override
                        public int compare(CarrierState o1, CarrierState o2) {
                            return Integer.compare(o1.getId(), o2.getId());
                        }
                    });
                    for (CarrierState sp : this.startPoints) {
                        if (sp.isColliding()) {
                            continue;
                        }
                        CarrierState gp = sp.getGoal();
                        if (null != gp
                                && (gp.getType() == CarrierStateTypeEnum.GOAL
                                || gp.getType() == CarrierStateTypeEnum.WAYPOINT)) {
                            if (gp.isColliding()) {
                                continue;
                            }
                            PlannedPath path = new PlannedPath(this);
                            path.setStartPoint(sp);
                            path.setGoalPoint(gp);
                            pi.start = sp;
                            pi.goal = gp;
                            List<PlannerPoint> llcopy = new LinkedList<>();
                            llcopy.addAll(ll);
                            sp.setPlannerList(llcopy);
                            gp.setPlannerList(llcopy);
                            path.setPlannerInput(pi.clone());
                            newPlannedPaths.add(path);
                        }
                    }
                }
                if (null != this.waypoints) {
                    for (CarrierState wp : this.waypoints) {
                        if (wp.isColliding()) {
                            continue;
                        }
                        CarrierState gp = wp.getGoal();
                        if (null != gp
                                && (gp.getType() == CarrierStateTypeEnum.GOAL
                                || gp.getType() == CarrierStateTypeEnum.WAYPOINT)) {
                            if (gp.isColliding()) {
                                continue;
                            }
                            PlannedPath path = new PlannedPath(this);
                            path.setStartPoint(wp);
                            path.setGoalPoint(gp);
                            pi.start = wp;
                            pi.goal = gp;
                            List<PlannerPoint> llcopy = new LinkedList<>();
                            llcopy.addAll(ll);
                            wp.setPlannerList(llcopy);
                            gp.setPlannerList(llcopy);
                            path.setPlannerInput(pi.clone());
                            newPlannedPaths.add(path);
                        }
                    }
                }
                final boolean use_exclusive_paths = this.isExclusivePaths();
                if (null == plan_thread) {
                    plan_thread = new Thread(
                            new Runnable() {
                                @Override
                                public void run() {
                                    try {
                                        while (!Thread.currentThread().isInterrupted()) {
                                            planning = false;
                                            PlanningArgs pa = planQueue.take();
                                            planning = true;
                                            final boolean use_exclusive_paths = pa.use_exclusive_paths;
                                            final List<PlannedPath> newPlannedPaths = pa.newPlannedPaths;
                                            for (int i = 0; i < newPlannedPaths.size(); i++) {
                                                if (Thread.currentThread().isInterrupted()) {
                                                    return;
                                                }
                                                PlannedPath path = newPlannedPaths.get(i);
                                                CarrierState sp = path.getStartPoint();
                                                CarrierState gp = path.getGoalPoint();
                                                PlannerInput pi = path.getPlannerInput();
                                                pi.start = sp;
                                                pi.goal = gp;
                                                if (use_exclusive_paths && !ignoreBoundaries) {
                                                    List<Boundary> path_outline
                                                    = path.getPlanOutLine();
                                                    if (path_outline != null) {
                                                        fullBoundaries.addAll(path_outline);
                                                        pi.boundaries = fullBoundaries;
                                                        List<PlannerPoint> newList = Planner.createPlannerList(pi, startPoints, goalPoints, waypoints);
                                                        sp.setPlannerList(newList);
                                                    }
                                                }
                                                List<PlannerPoint> newControlPoints = Planner.planWithPlannerList(pi, sp.getPlannerList());
                                                if (null != newControlPoints && newControlPoints.size() > 0) {
                                                    path.setPrePostPathDist(segStartLength);
                                                    path.setControlPoints(newControlPoints);
//                            if (!path.checkOutlines(pi.boundaries, pi.obstacles)) {
//                                if (this.pathUncertainty < this.maxPathUncertainty) {
//                                    this.setPathUncertainty(pathUncertainty + this.vehicleWidth * 0.05);
//                                    System.out.println("checkOutlines failed increasing pathUncertainty to" + pathUncertainty);
//                                    path.setControlPath(null);
//                                    return false;
//                                }
//                            }
                                                    if (use_exclusive_paths && null != path.getOutlines()) {
                                                        for (Line2Dd l : path.getOutlines()) {
                                                            fullBoundaries.add(new Boundary(l));
                                                        }
                                                    }
                                                }
                                                newPlannedPaths.set(i, path);
                                            }
                                            planning = false;
                                            try {
                                                java.awt.EventQueue.invokeAndWait(new Runnable() {
                                                    @Override
                                                    public void run() {
                                                        setPlannedPaths(newPlannedPaths);
                                                        repaint();
                                                    }
                                                });
                                            } catch (InterruptedException interruptedException) {
                                            } catch (InvocationTargetException invocationTargetException) {
                                            }

                                        }
                                    } catch (InterruptedException interruptedException) {
                                        //interruptedException.printStackTrace();
                                    }
                                    planning = false;
                                }
                            }, "planThread");
                    plan_thread.start();
                }
                try {
                    if (newPlannedPaths.size() > 0) {
                        planQueue.add(new PlanningArgs(newPlannedPaths, use_exclusive_paths));
                    }
                } catch (Exception e) {
                }
            }
        } catch (Exception exception) {
            if (!showing_planner_msg) {
                showing_planner_msg = true;
                exception.printStackTrace();
                JOptionPane.showMessageDialog(this, "Planner Exception: " + exception.getLocalizedMessage());
                showing_planner_msg = false;
            }
        }
        return true;
    }
    private double maxPathUncertainty = 5.0;

    /**
     * Get the value of maxPathUncertainty
     *
     * @return the value of maxPathUncertainty
     */
    public double getMaxPathUncertainty() {
        return maxPathUncertainty;
    }

    /**
     * Set the value of maxPathUncertainty
     *
     * @param maxPathUncertainty new value of maxPathUncertainty
     */
    public void setMaxPathUncertainty(double maxPathUncertainty) {
        this.maxPathUncertainty = maxPathUncertainty;
    }
    private List<Obstacle> obstacles;

    /**
     * Get the value of obstacles
     *
     * @return the value of obstacles
     */
    public List<Obstacle> getObstacles() {
        return obstacles;
    }
    private Ellipse2D.Double obstacleEllipses[];

    /**
     * Set the value of obstacles
     *
     * @param obstacles new value of obstacles
     */
    public void setObstacles(List<Obstacle> obstacles) {
        this.obstacles = obstacles;
        this.obstacleEllipses = null;
        try {
            if (null != obstacles) {
                obstacleEllipses = new Ellipse2D.Double[obstacles.size()];
                for (int i = 0; i < obstacles.size(); i++) {
                    Obstacle obs = obstacles.get(i);
                    obstacleEllipses[i] = new Ellipse2D.Double(
                            obs.x - obs.radius,
                            obs.y - obs.radius,
                            2 * obs.radius, 2 * obs.radius);
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
        if (this.replanOnAllChanges) {
            this.replan();
        }
        this.repaint();
    }
    private List<Obstacle> tempObstacles;
    private boolean reverse;
    public static final double DEFAULT_MIN_TURN_RADIUS = 120.0;
    private double minTurnRadius = DEFAULT_MIN_TURN_RADIUS;

    /**
     * Get the value of minTurnRadius
     *
     * @return the value of minTurnRadius
     */
    public double getMinTurnRadius() {
        return minTurnRadius;
    }

    /**
     * Set the value of minTurnRadius
     *
     * @param minTurnRadius new value of minTurnRadius
     */
    public void setMinTurnRadius(double minTurnRadius) {
        this.minTurnRadius = minTurnRadius;
    }

    /**
     * Get the value of reverse
     *
     * @return the value of reverse
     */
    public boolean isReverse() {
        return reverse;
    }

    /**
     * Set the value of reverse
     *
     * @param reverse new value of reverse
     */
    public void setReverse(boolean reverse) {
        this.incrementDelayReplanCount();
        this.reverse = reverse;
        if (null != this.carriers) {
            for (CarrierState cs : this.carriers) {
                cs.setReverse(reverse);
            }
        }
        if (null != this.startPoints) {
            for (CarrierState cs : this.startPoints) {
                cs.setReverse(reverse);
            }
        }
        if (null != this.goalPoints) {
            for (CarrierState cs : this.goalPoints) {
                cs.setReverse(reverse);
            }
        }
        if (null != this.waypoints) {
            for (CarrierState cs : this.waypoints) {
                cs.setReverse(reverse);
            }
        }
        this.decrementDelayReplanCount();
        if (this.replanOnAllChanges) {
            this.replan();
        }
        this.repaint();
    }

    /**
     * Get the value of obstacles
     *
     * @return the value of obstacles
     */
    public List<Obstacle> getTempObstacles() {
        return tempObstacles;
    }
    private Ellipse2D.Double tempObstacleEllipses[];

    /**
     * Set the value of obstacles
     *
     * @param tempObstacles new value of obstacles
     */
    public void setTempObstacles(List<Obstacle> tempObstacles) {
        this.tempObstacles = tempObstacles;
        this.tempObstacleEllipses = null;
        try {
            if (null != tempObstacles) {
                tempObstacleEllipses = new Ellipse2D.Double[tempObstacles.size()];
                for (int i = 0; i < tempObstacles.size(); i++) {
                    Obstacle obs = tempObstacles.get(i);
                    tempObstacleEllipses[i] = new Ellipse2D.Double(
                            obs.x - obs.radius,
                            obs.y - obs.radius,
                            2 * obs.radius, 2 * obs.radius);
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
        this.repaint();
    }
    private Path2D.Double controlPath;
    private Path2D.Double curvePath;
    double xmin = 0f;
    double xmax = 1000f;
    double ymin = 0f;
    double ymax = 1000f;
    double zoomScale = 1f;

    public double getZoomScale() {
        return zoomScale;
    }

    public void setZoomScale(double _scale) {
        if (Double.isInfinite(_scale) || Double.isNaN(_scale) || Math.abs(_scale) < 1e-3f) {
            throw new IllegalArgumentException("_scale = " + _scale);
        }
        this.zoomScale = _scale;
        this.repaint();
    }
    private boolean LabelControlPoints = false;
    private List<ShapeWTransform> backgroundShapes;

    /**
     * Get the value of LabelControlPoints
     *
     * @return the value of LabelControlPoints
     */
    public boolean isLabelControlPoints() {
        return LabelControlPoints;
    }

    /**
     * Set the value of LabelControlPoints
     *
     * @param LabelControlPoints new value of LabelControlPoints
     */
    public void setLabelControlPoints(boolean LabelControlPoints) {
        this.LabelControlPoints = LabelControlPoints;
        this.repaint();
    }

    private Rectangle2Dd checkPtRect(Point2D pt, Rectangle2Dd rect) {
        if (null == pt) {
            return rect;
        }
        double bound_buf_size = Math.max(vehicleWidth, this.defaultObsRadius);
        bound_buf_size = Math.max(bound_buf_size, getVehicleFront());
        bound_buf_size = Math.max(bound_buf_size, getVehicleBack());
////        bound_buf_size = 0f;
        if (rect.x > pt.getX() - 1 - bound_buf_size) {
            rect.x = (int) (pt.getX() - 1 - bound_buf_size);
        }
        if (rect.getY() > pt.getY() - 1 - bound_buf_size) {
            rect.y = (int) (pt.getY() - 1 - bound_buf_size);
        }
        if (rect.width < pt.getX() - rect.getX() + 1 + bound_buf_size) {
            rect.width = (int) (pt.getX() - rect.getX() + 1 + bound_buf_size);
        }
        if (rect.height < pt.getY() - rect.getY() + 1 + bound_buf_size) {
            rect.height = (int) (pt.getY() - rect.getY() + 1 + bound_buf_size);
        }
        return rect;
    }

    private Rectangle2Dd getBoundingRectPointList(Rectangle2Dd rect,
            List<? extends CarrierState> list) {
        if (null != list) {
            for (CarrierState sp : list) {
                rect = checkPtRect(sp, rect);
                if (null != sp.getShape()) {
                    Rectangle2D rectSp = sp.getShape().getBounds2D();
                    AffineTransform at = sp.getAffineTransform();
                    if (at != null) {
                        rect = checkPtRect(new Point2Dd(
                                rectSp.getMinX() + at.getTranslateX(),
                                rectSp.getMinY() + at.getTranslateY()),
                                rect);
                        rect = checkPtRect(new Point2Dd(
                                rectSp.getMaxX() + at.getTranslateX(),
                                rectSp.getMaxY() + at.getTranslateY()),
                                rect);
                    } else {
                        rect = checkPtRect(new Point2Dd(
                                rectSp.getMinX(),
                                rectSp.getMinY()),
                                rect);
                        rect = checkPtRect(new Point2Dd(
                                rectSp.getMaxX(),
                                rectSp.getMaxY()),
                                rect);
                    }
                }
            }
        }
        return rect;
    }

    public Rectangle2Dd getBoundingRect() {
        Rectangle2Dd rect = new Rectangle2Dd();
        rect.x = Double.POSITIVE_INFINITY;
        rect.y = Double.POSITIVE_INFINITY;
        rect.width = 0;
        rect.height = 0;
        if (null != this.backgroundShapes && this.showBackgroundShapes) {
            for (ShapeWTransform swt : this.backgroundShapes) {
                if (null == swt.getShape()) {
                    continue;
                }
                Rectangle2D rectSwt = swt.getShape().getBounds2D();
                AffineTransform at = swt.getAffineTransform();
                double hwmax = 0;
                if (at != null) {
                    rect = checkPtRect(new Point2Dd(
                            rectSwt.getMinX() + at.getTranslateX() + hwmax,
                            rectSwt.getMinY() + at.getTranslateY() + hwmax),
                            rect);
                    rect = checkPtRect(new Point2Dd(
                            rectSwt.getMaxX() + at.getTranslateX() - hwmax,
                            rectSwt.getMaxY() + at.getTranslateY() - hwmax),
                            rect);
                } else {
                    rect = checkPtRect(new Point2Dd(
                            rectSwt.getMinX() + hwmax,
                            rectSwt.getMinY() + hwmax),
                            rect);
                    rect = checkPtRect(new Point2Dd(
                            rectSwt.getMaxX() + hwmax,
                            rectSwt.getMaxY() + hwmax),
                            rect);
                }
            }
        }
        rect = getBoundingRectPointList(rect, this.startPoints);
        rect = getBoundingRectPointList(rect, this.waypoints);
        rect = getBoundingRectPointList(rect, this.goalPoints);
        if (null != this.plannedPaths) {
            for (PlannedPath path : this.plannedPaths) {
                List<PlannerPoint> ctlPts = path.getControlPoints();
                if (null != ctlPts) {
                    for (Point2Dd pt : ctlPts) {
                        rect = checkPtRect(pt, rect);
                    }
                }
            }
        }
        if (null != this.obstacles) {
            for (int i = 0; i < this.obstacles.size(); i++) {
                Point2Dd pt = this.obstacles.get(i);
                rect = checkPtRect(pt, rect);
            }
        }
        if (null != this.boundaries) {
            for (int i = 0; i < this.boundaries.size(); i++) {
                Boundary b = this.boundaries.get(i);
                rect = checkPtRect(b.getP1(), rect);
                rect = checkPtRect(b.getP2(), rect);

            }
        }
        return rect;
    }
    private double translateX = 0;

    /**
     * Get the value of translateX
     *
     * @return the value of translateX
     */
    public double getTranslateX() {
        return translateX;
    }

    /**
     * Set the value of translateX
     *
     * @param translateX new value of translateX
     */
    public void setTranslateX(double _translateX) {
        if (Double.isInfinite(_translateX) || Double.isNaN(_translateX)) {
            throw new IllegalArgumentException("_translateX = " + _translateX);
        }
        this.translateX = _translateX;
        this.repaint();
    }
    private double translateY = 0;

    /**
     * Get the value of translateY
     *
     * @return the value of translateY
     */
    public double getTranslateY() {
        return translateY;
    }

    /**
     * Set the value of translateY
     *
     * @param translateY new value of translateY
     */
    public void setTranslateY(double _translateY) {
        if (Double.isInfinite(_translateY) || Double.isNaN(_translateY)) {
            throw new IllegalArgumentException("_translateY = " + _translateY);
        }
        this.translateY = _translateY;
        this.repaint();
    }
    private double gridInc = 1000f;

    /**
     * Get the value of gridInc
     *
     * @return the value of gridInc
     */
    public double getGridInc() {
        return gridInc;
    }

    /**
     * Set the value of gridInc
     *
     * @param gridInc new value of gridInc
     */
    public void setGridInc(double gridInc) {
        this.gridInc = gridInc;
    }
    private String gridUnitName = "mm";

    /**
     * Get the value of gridUnitName
     *
     * @return the value of gridUnitName
     */
    public String getGridUnitName() {
        return gridUnitName;
    }

    /**
     * Set the value of gridUnitName
     *
     * @param gridUnitName new value of gridUnitName
     */
    public void setGridUnitName(String gridUnitName) {
        this.gridUnitName = gridUnitName;
    }
    private double gridUnitScale = 1f;

    /**
     * Get the value of gridUnitScale
     *
     * @return the value of gridUnitScale
     */
    public double getGridUnitScale() {
        return gridUnitScale;
    }

    /**
     * Set the value of gridUnitScale
     *
     * @param gridUnitScale new value of gridUnitScale
     */
    public void setGridUnitScale(double gridUnitScale) {
        this.gridUnitScale = gridUnitScale;
    }

    public void DeleteSelection() {
        if (null != obstacles) {
            LinkedList<Obstacle> obslist = new LinkedList<Obstacle>(this.obstacles);
            for (int i = 0; i < obslist.size(); i++) {
                if (obslist.get(i).selected) {
                    obslist.remove(i);
                    i--;
                }
            }
            this.setObstacles(obslist);
        }
        if (null != this.startPoints) {
            for (int i = 0; i < this.startPoints.size(); i++) {
                CarrierState sp = this.startPoints.get(i);
                if (sp.selected) {
                    this.startPoints.remove(i);
                    i--;
                }
            }
        }
        if (null != this.goalPoints) {
            for (int i = 0; i < this.goalPoints.size(); i++) {
                CarrierState gp = this.goalPoints.get(i);
                if (gp.selected) {
                    this.goalPoints.remove(i);
                    i--;
                }
            }
        }
        if (null != this.waypoints) {
            for (int i = 0; i < this.waypoints.size(); i++) {
                CarrierState wp = this.waypoints.get(i);
                if (wp.selected) {
                    this.waypoints.remove(i);
                    i--;
                }
            }
        }
        if (null != boundaries) {
            LinkedList<Boundary> blist = new LinkedList<Boundary>(this.boundaries);
            for (int i = 0; i < blist.size(); i++) {
                Boundary b = blist.get(i);
                if (b.P1Selected || b.P2Selected) {
                    blist.remove(i);
                    i--;
                }
            }
            this.setBoundaries(blist);
        }
    }

    public void DeleteObject(Object o) {
        if (null != obstacles) {
            LinkedList<Obstacle> obslist = new LinkedList<Obstacle>(this.obstacles);
            for (int i = 0; i < obslist.size(); i++) {
                if (obslist.get(i).equals(o)) {
                    obslist.remove(i);
                    i--;
                }
            }
            this.setObstacles(obslist);
        }
        if (null != this.startPoints) {
            for (int i = 0; i < this.startPoints.size(); i++) {
                CarrierState sp = this.startPoints.get(i);
                if (sp.equals(o)) {
                    CarrierState gp = sp.getGoal();
                    if (null != gp && gp.getStart() == o) {
                        gp.setStart(null);
                    }
                    this.startPoints.remove(i);
                    i--;
                }
            }
        }
        if (null != this.goalPoints) {
            for (int i = 0; i < this.goalPoints.size(); i++) {
                CarrierState gp = this.goalPoints.get(i);
                if (gp.equals(o)) {
                    CarrierState sp = gp.getStart();
                    if (null != sp && sp.getGoal() == o) {
                        sp.setGoal(null);
                    }
                    this.goalPoints.remove(i);
                    i--;
                }
            }
        }
        if (null != this.waypoints) {
            for (int i = 0; i < this.waypoints.size(); i++) {
                CarrierState wp = this.waypoints.get(i);
                if (wp.equals(o)) {
                    CarrierState gp = wp.getGoal();
                    if (null != gp && gp.getStart() == o) {
                        gp.setStart(null);
                    }
                    CarrierState sp = wp.getStart();
                    if (null != sp && sp.getGoal() == o) {
                        sp.setGoal(null);
                    }
                    this.waypoints.remove(i);
                    i--;
                }
            }
        }
        if (null != boundaries) {
            LinkedList<Boundary> blist = new LinkedList<Boundary>(this.boundaries);
            for (int i = 0; i < blist.size(); i++) {
                Boundary b = blist.get(i);
                if (b.equals(o)) {
                    blist.remove(i);
                    i--;
                }
            }
            this.setBoundaries(blist);
        }
    }
    private List<Shape> gridShapes = null;
    private Dimension lastGridShapeD = null;
    private double last_first_gridX = Double.NEGATIVE_INFINITY;
    private double last_first_gridY = Double.NEGATIVE_INFINITY;
    private List<GridLabel> gridLabels = null;
    private double last_zoomScale = Double.NEGATIVE_INFINITY;
    private double last_gridInc = Double.NEGATIVE_INFINITY;
    private String last_gridUnitName = "~";
    private double last_gridUnitScale = Double.NEGATIVE_INFINITY;

    private void makeGridShapes(double first_gridX, double first_gridY, Dimension d) {
        gridShapes = new LinkedList<>();
        for (double x = first_gridX; x < ((0f + d.width)); x += gridInc * zoomScale) {
            gridShapes.add(new Line2Dd(x, -2 * d.height, x, 2 * d.height));
        }
        for (double y = first_gridY; y < ((0f + d.height)); y += gridInc * zoomScale) {
            gridShapes.add(new Line2Dd(-2 * d.width, y, 2 * d.width, y));
        }
        gridLabels = new LinkedList<>();
        for (double x = first_gridX; x < (2 * d.width); x += gridInc * zoomScale) {
            for (double y = first_gridY; y < (2 * d.height); y += gridInc * zoomScale) {
                gridLabels.add(new GridLabel(String.format("(%.0f %s,%.0f %s)",
                        gridUnitScale * (x - translateX) / zoomScale,
                        gridUnitName,
                        gridUnitScale * (y - translateY) / zoomScale,
                        gridUnitName),
                        x, y));
            }
        }
        this.last_gridInc = gridInc;
        this.last_gridUnitName = this.gridUnitName;
        this.last_zoomScale = this.zoomScale;
        this.lastGridShapeD = d;
        this.last_first_gridX = first_gridX;
        this.last_first_gridY = first_gridY;
    }

    private void paintGrid(Graphics2D g2d) {
        Dimension d = this.getSize();
        g2d.setColor(Color.BLACK);
        double first_gridX = (zoomScale * gridInc - 2 * d.width + translateX);
        double first_gridY = (zoomScale * gridInc - 2 * d.height + translateY);
        if (gridInc * zoomScale < 300) {
            this.setGridInc(gridInc * 10.0f);
        }
        if (gridInc * zoomScale > 1000) {
            this.setGridInc(gridInc / 10.0);
        }
        if (null == this.gridShapes
                || null == this.gridLabels
                || this.last_gridInc != gridInc
                || this.last_zoomScale != zoomScale
                || !this.last_gridUnitName.equals(this.gridUnitName)
                || this.last_first_gridX != first_gridX
                || this.last_first_gridY != first_gridY
                || !this.lastGridShapeD.equals(d)) {
            this.makeGridShapes(first_gridX, first_gridY, d);
        }
        if (null != this.gridShapes) {
            for (Shape gs : this.gridShapes) {
                g2d.draw(gs);
            }
        }
        if (null != this.gridLabels && this.LabelGrid) {
            for (GridLabel gl : this.gridLabels) {
                drawString(g2d, gl.s, gl.x, gl.y);
            }
        }
//        System.out.println("d = " + d);
//        System.out.println("first_gridX = " + first_gridX);
//        System.out.println("first_gridY = " + first_gridY);
//        System.out.println("grid_labels = " + grid_labels);
//        System.out.println("grid_lines = " + grid_lines);
    }
    public static final double DEFAULT_PLANNER_POINT_DISPLAY_SIZE = 25.0;
    private double plannerPointDisplaySize = DEFAULT_PLANNER_POINT_DISPLAY_SIZE;

    /**
     * Get the value of plannerPointDisplaySize
     *
     * @return the value of plannerPointDisplaySize
     */
    public double getPlannerPointDisplaySize() {
        return plannerPointDisplaySize;
    }

    /**
     * Set the value of plannerPointDisplaySize
     *
     * @param plannerPointDisplaySize new value of plannerPointDisplaySize
     */
    public void setPlannerPointDisplaySize(double plannerPointDisplaySize) {
        this.plannerPointDisplaySize = plannerPointDisplaySize;
    }

    private void paintPlannerPointTestLines(Graphics2D g2d, PlannerPoint pp) {
        try {
            if (null != pp.testedInLinesMap) {
                for (PlannerPoint source_pp : pp.testedInLinesMap.keySet()) {
                    if (source_pp.selected) {
                        List<Line2Dd> testedInLines = pp.testedInLinesMap.get(source_pp);
                        if (null != testedInLines) {
                            for (Line2D l : testedInLines) {
                                g2d.setColor(Color.MAGENTA);
                                g2d.draw(l);
                            }
                        }
                    }
                }
            }
            if (null != pp.testedOutLinesMap) {

                for (PlannerPoint dest_pp : pp.testedOutLinesMap.keySet()) {
                    if (dest_pp.selected) {
                        List<Line2Dd> testedOutLines = pp.testedOutLinesMap.get(dest_pp);
                        if (null != testedOutLines) {
                            for (Line2D l : testedOutLines) {
                                g2d.setColor(Color.ORANGE);
                                g2d.draw(l);
                            }
                        }
                    }
                }

            }
        } catch (ConcurrentModificationException cme) {
        }
    }
    private double startingAngle = 90.0;

    /**
     * Get the value of startingAngle
     *
     * @return the value of startingAngle
     */
    public double getStartingAngle() {
        return startingAngle;
    }

    /**
     * Set the value of startingAngle
     *
     * @param startingAngle new value of startingAngle
     */
    public void setStartingAngle(double startingAngle) {
        this.startingAngle = startingAngle;
    }
    private double segStartLength = 50.0;

    /**
     * Get the value of segStartLength
     *
     * @return the value of segStartLength
     */
    public double getSegStartLength() {
        return segStartLength;
    }

    /**
     * Set the value of segStartLength
     *
     * @param segStartLength new value of segStartLength
     */
    public void setSegStartLength(double segStartLength) {
        this.segStartLength = segStartLength;
    }
    private double planningHorizon = 500.0;

    /**
     * Get the value of planningHorizon
     *
     * @return the value of planningHorizon
     */
    public double getPlanningHorizon() {
        return planningHorizon;
    }

    /**
     * Set the value of planningHorizon
     *
     * @param planningHorizon new value of planningHorizon
     */
    public void setPlanningHorizon(double planningHorizon) {
        this.planningHorizon = planningHorizon;
    }
    private boolean replanOnAllChanges = true;

    /**
     * Get the value of replanOnAllChanges
     *
     * @return the value of replanOnAllChanges
     */
    public boolean isReplanOnAllChanges() {
        return replanOnAllChanges;
    }

    /**
     * Set the value of replanOnAllChanges
     *
     * @param replanOnAllChanges new value of replanOnAllChanges
     */
    public void setReplanOnAllChanges(boolean replanOnAllChanges) {
        this.replanOnAllChanges = replanOnAllChanges;
    }

    private void paintBackgroundPlannerInfo(Graphics2D g2d, List<PlannerPoint> plannerList) {
        if (null == plannerList) {
            return;
        }
        try {
            for (int i = 0; null != plannerList && i < plannerList.size(); i++) {
                PlannerPoint pp = plannerList.get(i);
                g2d.setColor(new Color(200, 200, 200));
                if (pp.neighbors == null || pp.neighbors.size() == 0) {
                    if (pp.selected) {
                        this.paintPlannerPointTestLines(g2d, pp);
                        g2d.setColor(Color.GREEN);
                    }
                    double sz = this.plannerPointDisplaySize;
                    g2d.draw(new Ellipse2D.Double(pp.x - sz / 2.0, pp.y - sz / 2.0, sz, sz));
//                    for(PlannerPoint ppn : pp.neighbors) {
//                        paintBackgroundPlannerInfo(g2d,ppn.neighbors);
//                    }
                }
                g2d.setColor(new Color(200, 200, 200));
                try {
                    if (null != pp.failed_neighbors) {
                        for (int j = 0; j < pp.failed_neighbors.size(); j++) {
                            PlannerPoint failed_n = pp.failed_neighbors.get(j);
                            g2d.draw(new Line2Dd(pp, failed_n));
                        }
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
                g2d.setColor(Color.BLACK);
                try {
                    if (null != pp.neighbors) {
                        for (int j = 0; j < pp.neighbors.size(); j++) {
                            PlannerPoint n = pp.neighbors.get(j);
                            g2d.draw(new Line2Dd(pp, n));
                        }
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void paintFilledObstacles(Graphics2D g2d) {
        if (null != this.obstacleEllipses) {
            for (Ellipse2D.Double e : this.obstacleEllipses) {
                g2d.setColor(Color.ORANGE);
                g2d.fill(e);
            }
        }
    }

    private void paintFilledTempObstacles(Graphics2D g2d) {
        if (null != this.tempObstacleEllipses) {
            for (Ellipse2D.Double e : this.tempObstacleEllipses) {
                g2d.setColor(Color.GRAY);
                g2d.fill(e);
            }
        }
    }

    private void paintOutlines(Graphics2D g2d, List<? extends Shape> _outlines) {
        g2d.setColor(Color.yellow);
        for (Shape s : _outlines) {
            g2d.draw(s);
        }
    }

    private void paintControlPath(Graphics2D g2d, Path2D _controlPath) {
        if (this.showControlPath) {
            g2d.setColor(Color.BLACK);
            g2d.draw(_controlPath);
        }
    }
    private boolean showCenterCurve = true;

    /**
     * Get the value of showCenterCurve
     *
     * @return the value of showCenterCurve
     */
    public boolean isShowCenterCurve() {
        return showCenterCurve;
    }

    /**
     * Set the value of showCenterCurve
     *
     * @param showCenterCurve new value of showCenterCurve
     */
    public void setShowCenterCurve(boolean showCenterCurve) {
        this.showCenterCurve = showCenterCurve;
        this.repaint();
    }
    private boolean showSideCurves = false;

    /**
     * Get the value of showSideCurves
     *
     * @return the value of showSideCurves
     */
    public boolean isShowSideCurves() {
        return showSideCurves;
    }

    /**
     * Set the value of showSideCurves
     *
     * @param showSideCurves new value of showSideCurves
     */
    public void setShowSideCurves(boolean showSideCurves) {
        this.showSideCurves = showSideCurves;
        this.repaint();
    }

    private void paintCurvePaths(Graphics2D g2d,
            Path2D _curvePath,
            Path2D _curveLeftPath,
            Path2D _curveRightPath) {
        if (null != _curvePath && this.showCenterCurve) {
            g2d.setColor(Color.blue);
            g2d.draw(_curvePath);
        }
        if (this.showSideCurves) {
            if (null != _curveLeftPath) {
                g2d.setColor(Color.blue);
                g2d.draw(_curveLeftPath);
            }
            if (null != _curveRightPath) {
                g2d.setColor(Color.blue);
                g2d.draw(_curveRightPath);
            }
        }
    }

    public void drawString(Graphics2D g2d, String s, double x, double y) {
        AffineTransform orig_at = g2d.getTransform();
        //System.out.println("orig_at = " + orig_at);
        double zs = this.zoomScale;
        if (orig_at.getScaleY() < 0) {
            if (orig_at.getScaleX() == 1.0) {
                zs = 1.0;
            }
            g2d.scale(1.0 / zs, -1.0 / zs);
            g2d.drawString(s, (float) (x * zs), (float) (-y * zs));
            g2d.setTransform(orig_at);
        } else {
            g2d.drawString(s, (float) x, (float) y);
        }
    }
    private boolean connectedToVehicle;

    /**
     * Get the value of connectedToVehicle
     *
     * @return the value of connectedToVehicle
     */
    public boolean isConnectedToVehicle() {
        return connectedToVehicle;
    }

    /**
     * Set the value of connectedToVehicle
     *
     * @param connectedToVehicle new value of connectedToVehicle
     */
    public void setConnectedToVehicle(boolean connectedToVehicle) {
        this.connectedToVehicle = connectedToVehicle;
    }

    private void paintStart(Graphics2D g2d, CarrierState sp, Shape ss, Shape sl, Shape arrow) {
        AffineTransform orig_at = g2d.getTransform();
        if (null != ss) {
            if (!sp.isColliding()) {
                g2d.setColor(new Color(1f, 1f, 1f, 0.5f));
            } else {
                g2d.setColor(new Color(1f, 0f, 0f, 0.5f));
            }
            if (sp.getAffineTransform() != null) {
                AffineTransform full_at = new AffineTransform();
                full_at.concatenate(orig_at);
                full_at.concatenate(sp.getAffineTransform());
                g2d.setTransform(full_at);
            }
            g2d.fill(ss);
            g2d.setTransform(orig_at);
        }
        g2d.setColor(new Color(0f, 0f, 0f, 0.5f));
        drawString(g2d, "Start",
                sp.x,
                sp.y);
        if (null != sl) {
            g2d.draw(sl);
        }
        if (null != arrow) {
            g2d.fill(arrow);
        }
        if (sp.selected) {
            g2d.setColor(Color.GREEN);
            if (sp.getAffineTransform() != null) {
                AffineTransform full_at = new AffineTransform();
                full_at.concatenate(orig_at);
                full_at.concatenate(sp.getAffineTransform());
                g2d.setTransform(full_at);
            }
            g2d.draw(ss);
            g2d.setTransform(orig_at);
        }
    }

    public void removeImproperBoundaries() {
        if (null != boundaries) {
            for (int i = 0; i < boundaries.size(); i++) {
                Boundary b = boundaries.get(i);
                if (b.hasInfiniteOrNaN()) {
                    boundaries.remove(i);
                }
            }
        }
    }

    private void checkCollistionsPointList(List<? extends CarrierState> list) {
        if (null != list) {
            for (CarrierState cs : list) {
                cs.setColliding(false);
            }
            if (null != this.obstacles) {
                for (Obstacle obs : this.obstacles) {
                    for (CarrierState cs : list) {
                        cs.checkObstacle(obs);
                    }
                }
            }
            if (null != this.boundaries) {
                for (Boundary b : this.boundaries) {
                    for (CarrierState cs : list) {
                        cs.checkBoundary(b);
                    }
                }
            }
        }
    }

    public void checkCollisions() {
        checkCollistionsPointList(this.startPoints);
        checkCollistionsPointList(this.carriers);
        checkCollistionsPointList(this.goalPoints);
        checkCollistionsPointList(this.waypoints);
    }

    private void paintGoal(Graphics2D g2d, CarrierState gp, Shape gs, Shape gl, Shape arrow) {
        AffineTransform orig_at = g2d.getTransform();
        if (null != gs) {
            if (!gp.isColliding()) {
                g2d.setColor(new Color(0f, 0f, 0f, 0.5f));
            } else {
                g2d.setColor(new Color(1f, 0f, 0f, 0.5f));
            }
            if (gp.getAffineTransform() != null) {
                AffineTransform full_at = new AffineTransform();
                full_at.concatenate(orig_at);
                full_at.concatenate(gp.getAffineTransform());
                g2d.setTransform(full_at);
            }
            g2d.fill(gs);
            g2d.setTransform(orig_at);
        }
        g2d.setColor(new Color(1f, 1f, 1f, 0.5f));
        drawString(g2d, "Goal", gp.x, gp.y);
        if (null != gl) {
            g2d.draw(gl);
        }
        if (null != arrow) {
            g2d.fill(arrow);
        }
        if (gp.selected) {
            g2d.setColor(Color.GREEN);
            if (gp.getAffineTransform() != null) {
                AffineTransform full_at = new AffineTransform();
                full_at.concatenate(orig_at);
                full_at.concatenate(gp.getAffineTransform());
                g2d.setTransform(full_at);
            }
            g2d.draw(gs);
            g2d.setTransform(orig_at);
        }
    }

    private void paintWaypoint(Graphics2D g2d, CarrierState wp, Shape gs, Shape gl, Shape arrow) {
        AffineTransform orig_at = g2d.getTransform();
        if (null != gs) {
            if (!wp.isColliding()) {
                g2d.setColor(new Color(0.1f, 0.1f, 0.1f, 0.4f));
            } else {
                g2d.setColor(new Color(.9f, 0f, 0f, 0.4f));
            }
            if (wp.getAffineTransform() != null) {
                AffineTransform full_at = new AffineTransform();
                full_at.concatenate(orig_at);
                full_at.concatenate(wp.getAffineTransform());
                g2d.setTransform(full_at);
            }
            g2d.fill(gs);
            g2d.setTransform(orig_at);
        }
        g2d.setColor(new Color(1f, 1f, 1f, 0.5f));
        drawString(g2d, "Waypoint", wp.x, wp.y);
        if (null != gl) {
            g2d.draw(gl);
        }
        if (null != arrow) {
            g2d.fill(arrow);
        }
        if (wp.selected) {
            g2d.setColor(Color.GREEN);
            if (wp.getAffineTransform() != null) {
                AffineTransform full_at = new AffineTransform();
                full_at.concatenate(orig_at);
                full_at.concatenate(wp.getAffineTransform());
                g2d.setTransform(full_at);
            }
            g2d.draw(gs);
            g2d.setTransform(orig_at);
        }
    }

    private void paintObstacleOutlines(Graphics2D g2d) {
        if (null != obstacleEllipses) {
            g2d.setColor(Color.ORANGE);
            for (Ellipse2D.Double e : this.obstacleEllipses) {
                g2d.draw(e);
            }
        }
        if (null != obstacles && null != this.obstacleEllipses) {
            for (int i = 0; i < this.obstacles.size() && i < this.obstacleEllipses.length; i++) {
                Obstacle o = this.obstacles.get(i);
                Shape s = this.obstacleEllipses[i];
                if (o.selected) {
                    g2d.setColor(Color.GREEN);
                    g2d.draw(s);
                }
            }
        }
    }

    private void paintForegroundPlannerInfo(Graphics2D g2d, List<PlannerPoint> plannerList) {
        if (null == plannerList) {
            return;
        }
        g2d.setColor(Color.BLACK);
        double sz = this.plannerPointDisplaySize;
        for (PlannerPoint pp : plannerList) {
            if (pp.neighbors != null && pp.neighbors.size() > 0) {
                g2d.fill(new Ellipse2D.Double(pp.x - sz / 2.0, pp.y - sz / 2.0, sz, sz));
                //this.paintBackgroundPlannerInfo(g2d, pp.neighbors);
            }
            if (pp.selected) {
                this.paintPlannerPointTestLines(g2d, pp);
                g2d.setColor(Color.GREEN);
                g2d.draw(new Ellipse2D.Double(pp.x - sz / 2.0, pp.y - sz / 2.0, sz, sz));
                g2d.setColor(Color.BLACK);
            }
            if (null != pp.prev_pt) {
                g2d.draw(new Line2Dd(pp.x, pp.y, pp.prev_pt.x, pp.prev_pt.y));
            }
        }
    }

    private void paintControlPointLabels(Graphics2D g2d, List<PlannerPoint> _controlPoints) {
        g2d.setColor(Color.BLACK);
        for (int i = 0; i < _controlPoints.size(); i++) {
            Point2Dd pt = _controlPoints.get(i);
            g2d.fill(new Rectangle2Dd(pt.x - 1, pt.y - 1, 2, 2));
            drawString(g2d, Integer.toString(i), pt.x - 5f, pt.y - 5f);
        }
    }

    private void paintBoundaries(Graphics2D g2d) {
        g2d.setColor(Color.RED);
        for (Boundary b : this.boundaries) {
            if (Double.isNaN(b.x2) || Double.isNaN(b.y2)) {
                g2d.draw(new Rectangle2Dd(b.x1 - 1, b.y1 - 1, 3f, 3f));
            }
            if (b.P1Selected && b.P2Selected) {
                g2d.setColor(Color.green);
                AffineTransform at = g2d.getTransform();
                g2d.translate(-1, 0);
                g2d.draw(b);
                g2d.translate(+2, 0);
                g2d.draw(b);
                g2d.translate(-2, +1);
                g2d.draw(b);
                g2d.translate(+2, 0);
                g2d.draw(b);
                g2d.translate(-2, -2);
                g2d.draw(b);
                g2d.translate(+2, 0);
                g2d.draw(b);
                g2d.setTransform(at);
            } else if (b.P1Selected) {
                g2d.setColor(Color.green);
                g2d.draw(new Rectangle2Dd(b.x1 - 1, b.y1 - 1, 3, 3));
            } else if (b.P2Selected) {
                g2d.setColor(Color.green);
                g2d.draw(new Rectangle2Dd(b.x2 - 1, b.y2 - 1, 3, 3));
            }
            g2d.setColor(Color.RED);
            g2d.draw(b);
        }
    }
    private int delay_replan_count = 0;

    /**
     * Set the value of delay_replan
     *
     * @param delay_replan new value of delay_replan
     */
    public void incrementDelayReplanCount() {
//        try {
//            if (null != this.plan_thread) {
//                this.plan_thread.interrupt();
//                this.plan_thread.join();
//                this.plan_thread = null;
//            }
//        } catch (InterruptedException interruptedException) {
//        }
        if (null != planQueue && !planQueue.isEmpty()) {
            planQueue.remove(0);
        }
        this.delay_replan_count++;
    }

    /**
     * Set the value of delay_replan
     *
     * @param delay_replan new value of delay_replan
     */
    public void decrementDelayReplanCount() {
//        try {
//            if (null != this.plan_thread) {
//                this.plan_thread.interrupt();
//                this.plan_thread.join();
//                this.plan_thread = null;
//            }
//        } catch (InterruptedException interruptedException) {
//        }
        if (this.delay_replan_count > 0) {
            this.delay_replan_count--;
        }
    }

    public void forceReplan() {
        this.delay_replan_count = 0;
        boolean orig_connectedToVehicle = this.connectedToVehicle;
        this.connectedToVehicle = false;
        this.replan();
        this.connectedToVehicle = orig_connectedToVehicle;
        this.repaint();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.translate(0, this.getSize().height);
        g2d.scale(1.0, -1.0);
        Dimension d = this.getSize();
        if (this.ShowGrid) {
            this.paintGrid(g2d);
        }
        PlannerInput pi = this.getPlannerInput();
        
        g2d.translate(translateX, translateY);
        g2d.scale(zoomScale, zoomScale);
        if(null != pi && null != pi.rectB) {
            g.setColor(Color.lightGray);
            g2d.draw(pi.rectB);
        }
        if (null != this.backgroundShapes && this.showBackgroundShapes) {
            g2d.setColor(Color.DARK_GRAY);
            for (ShapeWTransform swt : this.backgroundShapes) {
                Shape s = swt.getShape();
                AffineTransform swt_at = swt.getAffineTransform();
                AffineTransform orig_g2d_at = g2d.getTransform();
                AffineTransform full_at = new AffineTransform();
                if (null != swt_at) {
                    full_at.concatenate(orig_g2d_at);
                    full_at.concatenate(swt_at);
                    g2d.setTransform(full_at);
                    g2d.draw(s);
                    g2d.setTransform(orig_g2d_at);
                } else {
                    g2d.draw(s);
                }
            }
        }
        if (!this.ExclusivePaths) {
//            if (null != Planner.ll && this.showPlanning) {
//                this.paintBackgroundPlannerInfo(g2d, Planner.ll);
//            }
            if (null != Planner.goalPPList && this.showPlanning) {
                this.paintBackgroundPlannerInfo(g2d, Planner.goalPPList);
            }
            if (null != Planner.startPPList && this.showPlanning) {
                this.paintBackgroundPlannerInfo(g2d, Planner.startPPList);
            }
        }
        if (null != tempObstacleEllipses) {
            this.paintFilledTempObstacles(g2d);
        }

        if (null != obstacleEllipses) {
            this.paintFilledObstacles(g2d);
        }
//        if (this.ShowOutline && null != outlines) {
//            this.paintOutlines(g2d, this.outlines);
//        }

        this.paintCurvePaths(g2d, this.curvePath, this.curveLeftPath, this.curveRightPath);
        if (null != this.controlPath && this.showControlPath) {
            this.paintControlPath(g2d, this.controlPath);
        }
        if (null != this.startPoints) {
            for (CarrierState sp : this.startPoints) {
                this.paintStart(g2d, sp, sp.getShape(), sp.getLine(), sp.getArrow());
                if (sp.selected) {
                    PlannedPath path = sp.getPath();
                    if (null != sp.getPlannerList() && this.showPlanning) {
                        this.paintBackgroundPlannerInfo(g2d, sp.getPlannerList());
                        if (null != sp.getPath()) {
                            this.paintBackgroundPlannerInfo(g2d, sp.getPath().getControlPoints());
                        }
                    }
                }
            }
        }
        if (null != this.goalPoints) {
            for (CarrierState gp : this.goalPoints) {
                if (null == gp) {
                    continue;
                }
                this.paintGoal(g2d, gp, gp.getShape(), gp.getLine(), gp.getArrow());
                if (gp.selected) {
                    PlannedPath path = gp.getPath();
                    if (null != gp.getPlannerList() && this.showPlanning) {
                        this.paintBackgroundPlannerInfo(g2d, gp.getPlannerList());
                        if (null != gp.getPath()) {
                            this.paintBackgroundPlannerInfo(g2d, gp.getPath().getControlPoints());
                        }
                    }
                }
            }
        }
        if (null != this.goalPoints) {
            for (CarrierState gp : this.goalPoints) {
                if (null == gp) {
                    continue;
                }
                this.paintGoal(g2d, gp, gp.getShape(), gp.getLine(), gp.getArrow());
                if (gp.selected) {
                    PlannedPath path = gp.getPath();
                    if (null != gp.getPlannerList() && this.showPlanning) {
                        this.paintBackgroundPlannerInfo(g2d, gp.getPlannerList());
                        if (null != gp.getPath()) {
                            this.paintBackgroundPlannerInfo(g2d, gp.getPath().getControlPoints());
                        }
                    }
                }
            }
        }
        if (null != this.waypoints) {
            for (CarrierState wp : this.waypoints) {
                if (null == wp) {
                    continue;
                }
                this.paintWaypoint(g2d, wp, wp.getShape(), wp.getLine(), wp.getArrow());
                if (wp.selected) {
                    PlannedPath path = wp.getPath();
                    if (null != wp.getPlannerList() && this.showPlanning) {
                        this.paintBackgroundPlannerInfo(g2d, wp.getPlannerList());
                        if (null != wp.getPath()) {
                            this.paintBackgroundPlannerInfo(g2d, wp.getPath().getControlPoints());
                        }
                    }
                }
            }
        }
        if (null != this.tmpGoal) {
            this.paintGoal(g2d, tmpGoal, tmpGoal.getShape(), tmpGoal.getLine(), tmpGoal.getArrow());
        }
        if (null != this.plannedPaths) {
            for (PlannedPath path : this.plannedPaths) {
                if (this.showPlanning && !this.ExclusivePaths) {
                    this.paintBackgroundPlannerInfo(g2d, path.getControlPoints());
                }
                this.paintCurvePaths(g2d, path.getCurvePath(), path.getCurveLeftPath(), path.getCurveRightPath());
                if (null != path.getControlPath()) {
                    this.paintControlPath(g2d, path.getControlPath());
                }
                if (null != path.getControlPoints() && this.LabelControlPoints) {
                    this.paintControlPointLabels(g2d, path.getControlPoints());
                }
                if (this.ShowOutline && null != path.getOutlines()) {
                    this.paintOutlines(g2d, path.getOutlines());
                }
                if (this.showPlanOutline && null != path.getPlanOutLine()) {
                    this.paintOutlines(g2d, path.getPlanOutLine());
                }
                if (this.showPlanning && !this.ExclusivePaths) {
                    this.paintForegroundPlannerInfo(g2d, path.getControlPoints());
                }
            }
        }
        if (null != this.carriers) {
            for (CarrierState cs : this.carriers) {
                PlannedPath path = cs.getPath();
                if (null != path) {
                    if (this.showPlanning && !this.ExclusivePaths) {
                        this.paintBackgroundPlannerInfo(g2d, path.getControlPoints());
                    }
                    this.paintCurvePaths(g2d, path.getCurvePath(), path.getCurveLeftPath(), path.getCurveRightPath());
                    if (null != path.getControlPath()) {
                        this.paintControlPath(g2d, path.getControlPath());
                    }
                    if (null != path.getControlPoints() && this.LabelControlPoints) {
                        this.paintControlPointLabels(g2d, path.getControlPoints());
                    }
                    if (this.ShowOutline && null != path.getOutlines()) {
                        this.paintOutlines(g2d, path.getOutlines());
                    }
                    if (this.showPlanOutline && null != path.getPlanOutLine()) {
                        this.paintOutlines(g2d, path.getPlanOutLine());
                    }
                    if (this.showPlanning && !this.ExclusivePaths) {
                        this.paintForegroundPlannerInfo(g2d, path.getControlPoints());
                    }
                }
            }
        }
        this.paintObstacleOutlines(g2d);
        if (!this.ExclusivePaths) {
//            if (null != Planner.ll && this.showPlanning) {
//                this.paintForegroundPlannerInfo(g2d, Planner.ll);
//            }
            if (null != Planner.startPPList && this.showPlanning) {
                this.paintForegroundPlannerInfo(g2d, Planner.startPPList);
            }
            if (null != Planner.goalPPList && this.showPlanning) {
                this.paintForegroundPlannerInfo(g2d, Planner.goalPPList);
            }
        }
        if (null != this.boundaries) {
            this.paintBoundaries(g2d);
        }
        if (null != this.startPoints) {
            for (CarrierState sp : this.startPoints) {
                this.paintStart(g2d, sp, sp.getShape(), sp.getLine(), sp.getArrow());
                if (sp.selected) {
                    PlannedPath path = sp.getPath();
                    if (null != sp.getPlannerList() && this.showPlanning) {
                        this.paintForegroundPlannerInfo(g2d, sp.getPlannerList());
                        if (null != sp.getPath()) {
                            this.paintForegroundPlannerInfo(g2d, sp.getPath().getControlPoints());
                        }
                    }
                }
            }
        }
        if (null != this.goalPoints) {
            for (CarrierState gp : this.goalPoints) {
                if (null == gp) {
                    continue;
                }
                if (gp.selected) {
                    PlannedPath path = gp.getPath();
                    if (null != gp.getPlannerList() && this.showPlanning) {
                        this.paintForegroundPlannerInfo(g2d, gp.getPlannerList());
                        if (null != gp.getPath()) {
                            this.paintForegroundPlannerInfo(g2d, gp.getPath().getControlPoints());
                        }
                    }
                }
            }
        }
        if (null != this.waypoints) {
            for (CarrierState wp : this.waypoints) {
                if (null == wp) {
                    continue;
                }
                if (wp.selected) {
                    PlannedPath path = wp.getPath();
                    if (null != wp.getPlannerList() && this.showPlanning) {
                        this.paintForegroundPlannerInfo(g2d, wp.getPlannerList());
                        if (null != wp.getPath()) {
                            this.paintForegroundPlannerInfo(g2d, wp.getPath().getControlPoints());
                        }
                    }
                }
            }
        }
        if (null != this.carriers) {
            for (CarrierState cs : this.carriers) {
                if (cs.getType() == CarrierStateTypeEnum.LIVE) {
                    if (cs.getPath() != null
                            && cs.getPath().getControlPath() != null) {
                        g2d.setColor(Color.GREEN);
                    } else {
                        g2d.setColor(Color.YELLOW);
                    }
                    g2d.fill(cs.getShape());
                    if (null != cs.getPlannerList() && this.showPlanning) {
                        this.paintBackgroundPlannerInfo(g2d, cs.getPlannerList());
                        if (null != cs.getPath()) {
                            this.paintBackgroundPlannerInfo(g2d, cs.getPath().getControlPoints());
                        }
                    }
                } else if (cs.getType() == CarrierStateTypeEnum.ESTOPPED) {
                    g2d.setColor(Color.RED);
                    g2d.fill(cs.getShape());
                }
                if (cs.selected || cs.getType() == CarrierStateTypeEnum.LIVE) {
                    PlannedPath path = cs.getPath();
                    if (null != path) {
                        this.paintControlPath(g2d, path.getControlPath());
                        this.paintCurvePaths(g2d, path.getCurvePath(), path.getCurveLeftPath(), path.getCurveRightPath());
                        if (null != path.getControlPoints() && this.LabelControlPoints) {
                            this.paintControlPointLabels(g2d, path.getControlPoints());
                        }
                    }
                    if (null != cs.getPlannerList() && this.showPlanning) {
                        this.paintForegroundPlannerInfo(g2d, cs.getPlannerList());
                        if (null != cs.getPath()) {
                            this.paintForegroundPlannerInfo(g2d, cs.getPath().getControlPoints());
                        }
                    }
                }
            }
        }
        if (null != this.select_rect) {
            g2d.setColor(Color.BLACK);
//            System.out.println("select_rect = " + select_rect);
            g2d.draw(this.select_rect);
        }
        if (null != this.planningAreaRect) {
            g2d.setColor(Color.MAGENTA);
//            System.out.println("select_rect = " + select_rect);
            g2d.draw(this.planningAreaRect);
        }
    }

    public void Clear() {
        this.StopSimulation();
        this.setObstacles(null);
        this.setStartPoints(null);
        this.setGoalPoints(null);
//        this.outlines = null;
        this.setBoundaries(null);
        this.setPlannedPaths(null);
        this.setCarriers(null);
        this.setStartPoints(null);
        this.setGoalPoints(null);
        this.setTempObstacles(null);
        this.setPlannedPaths(null);
        this.repaint();
    }

    public void ClearSelection() {
//        if (null != Planner.ll) {
//            for (PlannerPoint pp : Planner.ll) {
//                pp.selected = false;
//            }
//        }
        if (null != obstacles) {
            for (Obstacle o : obstacles) {
                o.selected = false;
            }
        }
        if (null != boundaries) {
            for (Boundary b : boundaries) {
                b.P1Selected = false;
                b.P2Selected = false;
            }
        }
        if (null != this.startPoints) {
            for (CarrierState sp : this.startPoints) {
                sp.selected = false;
            }
        }
        if (null != this.goalPoints) {
            for (CarrierState gp : this.goalPoints) {
                gp.selected = false;
            }
        }
        if (null != this.waypoints) {
            for (CarrierState wp : this.waypoints) {
                wp.selected = false;
            }
        }
        if (null != this.selectedObjects) {
            for (Object o : this.selectedObjects) {
                try {
                    Class clss = o.getClass();
                    Field f = clss.getField("selected");
                    if (null != f) {
                        f.setBoolean(o, false);
                    }
                } catch (Exception e) {
//                    System.out.println();
//                    System.err.println();
//                    e.printStackTrace();
//                    System.out.println();
//                    System.err.println();
//                    System.out.flush();
//                    System.out.flush();
                }
            }
            this.selectedObjects = null;
        }
    }
    private double BoundarySelectDist = 5f;

    /**
     * Get the value of BoundarySelectDist
     *
     * @return the value of BoundarySelectDist
     */
    public double getBoundarySelectDist() {
        return BoundarySelectDist;
    }

    /**
     * Set the value of BoundarySelectDist
     *
     * @param BoundarySelectDist new value of BoundarySelectDist
     */
    public void setBoundarySelectDist(double BoundarySelectDist) {
        this.BoundarySelectDist = BoundarySelectDist;
    }
    private Collection selectedObjects = null;

    private void addSelectedObject(Object o) {
        if (null == selectedObjects) {
            selectedObjects = new HashSet();
        }
        selectedObjects.add(o);
    }

    public void SelectByClick(Point2Dd pt) {
        boolean obstacle_selected = false;
        double fuzz = 3.0 / zoomScale;
        if (null != this.startPoints) {
            for (CarrierState sp : this.startPoints) {
                if (sp.contains(pt)) {
                    sp.selected = true;
                    addSelectedObject(sp);
                }
                List<PlannerPoint> ppl = sp.getPlannerList();
                if (null != ppl) {
                    for (PlannerPoint pp : ppl) {
                        if (pp.distance(pt) < this.plannerPointDisplaySize / 2 + fuzz) {
                            pp.selected = true;
                            addSelectedObject(pp);
                        }
                    }
                }
                PlannedPath plannedPath = sp.getPath();
                if (null != plannedPath) {
                    ppl = plannedPath.getControlPoints();
                    if (null != ppl) {
                        for (PlannerPoint pp : ppl) {
                            if (pp.distance(pt) < this.plannerPointDisplaySize / 2 + fuzz) {
                                pp.selected = true;
                                addSelectedObject(pp);
                            }
                        }
                    }
                }
            }

        }
        if (null != this.goalPoints) {
            for (CarrierState gp : this.goalPoints) {
                if (gp.contains(pt)) {
                    gp.selected = true;
                    addSelectedObject(gp);
                }
                List<PlannerPoint> ppl = gp.getPlannerList();
                if (null != ppl) {
                    for (PlannerPoint pp : ppl) {
                        if (pp.distance(pt) < this.plannerPointDisplaySize / 2 + fuzz) {
                            pp.selected = true;
                            addSelectedObject(pp);
                        }
                    }
                }
                PlannedPath plannedPath = gp.getPath();
                if (null != plannedPath) {
                    ppl = plannedPath.getControlPoints();
                    if (null != ppl) {
                        for (PlannerPoint pp : ppl) {
                            if (pp.distance(pt) < this.plannerPointDisplaySize / 2 + fuzz) {
                                pp.selected = true;
                                addSelectedObject(pp);
                            }
                        }
                    }
                }
            }
        }
        if (null != this.waypoints) {
            for (CarrierState wp : this.waypoints) {
                if (wp.contains(pt)) {
                    wp.selected = true;
                    addSelectedObject(wp);
                }
                List<PlannerPoint> ppl = wp.getPlannerList();
                if (null != ppl) {
                    for (PlannerPoint pp : ppl) {
                        if (pp.distance(pt) < this.plannerPointDisplaySize / 2 + fuzz) {
                            pp.selected = true;
                            addSelectedObject(pp);
                        }
                    }
                }
                PlannedPath plannedPath = wp.getPath();
                if (null != plannedPath) {
                    ppl = plannedPath.getControlPoints();
                    if (null != ppl) {
                        for (PlannerPoint pp : ppl) {
                            if (pp.distance(pt) < this.plannerPointDisplaySize / 2 + fuzz) {
                                pp.selected = true;
                                addSelectedObject(pp);
                            }
                        }
                    }
                }
            }
        }
        if (null != obstacles) {
            for (int i = 0; i < obstacles.size(); i++) {
                Obstacle o = obstacles.get(i);
                double dist = o.distance(pt);
                if (dist < o.radius + fuzz) {
                    o.selected = true;
                    obstacle_selected = true;
                    addSelectedObject(o);
                }
            }
        }
        if (!obstacle_selected) {
            if (null != boundaries) {
                for (int i = 0; i < boundaries.size(); i++) {
                    Boundary b = boundaries.get(i);
                    double dist = b.ptSegDist(pt);
                    if (dist < this.BoundarySelectDist + fuzz) {
                        double p1_dist = b.getP1().distance(pt);
                        double p2_dist = b.getP2().distance(pt);
                        if (p1_dist < p2_dist / 2f) {
                            b.P1Selected = true;
                            b.P2Selected = false;
                        } else if (p2_dist < p1_dist / 2f) {
                            b.P1Selected = false;
                            b.P2Selected = true;
                        } else {
                            b.P1Selected = true;
                            b.P2Selected = true;
                        }
                        addSelectedObject(b);
                    }
                }
            }
        }
//        if (null != Planner.ll) {
//            for (PlannerPoint pp : Planner.ll) {
//                if (pp.distance(pt) < this.plannerPointDisplaySize / 2 + fuzz) {
//                    pp.selected = true;
//                    addSelectedObject(pp);
//                }
//            }
//        }
        this.repaint();
    }

    public void AddBoundaryPoint(Point2Dd pt, boolean horz, boolean vert) {
        Boundary last_boundary = null;
        if (null == boundaries) {
            boundaries = new LinkedList<>();
        }
        if (boundaries.size() > 0) {
            last_boundary = boundaries.get(boundaries.size() - 1);
        }
        if (last_boundary == null) {
            last_boundary = new Boundary(pt.x, pt.y, Double.NaN, Double.NaN, horz, vert);
            boundaries.add(last_boundary);
        } else if (Double.isNaN(last_boundary.x2) || Double.isNaN(last_boundary.y2)) {
            last_boundary.x2 = pt.x;
            last_boundary.y2 = pt.y;
            if (last_boundary.force_horizontal) {
                last_boundary.y2 = last_boundary.y1;
            } else if (last_boundary.force_vertical) {
                last_boundary.x2 = last_boundary.x1;
            }
            if (last_boundary.x1 == last_boundary.x2 && last_boundary.y1 == last_boundary.y2) {
                last_boundary.x2 = Double.NaN;
                last_boundary.y2 = Double.NaN;
            } else {
                this.replan();
            }
        } else {
            Boundary b = new Boundary(pt.x, pt.y, Double.NaN, Double.NaN, horz, vert);
            boundaries.add(b);
        }
        this.repaint();
    }

    List findObjectsAtPt(Point2Dd pt) {
        List l = new LinkedList();
        double fuzz = 3 / this.zoomScale;
        if (null != obstacles) {
            for (Obstacle o : this.obstacles) {
                if (pt.distance(o) < o.radius + fuzz) {
                    l.add(o);
                }
            }
        }
        if (null != boundaries) {
            for (Boundary b : this.boundaries) {
                if (b.ptSegDist(pt) < this.BoundarySelectDist + fuzz) {
                    l.add(b);
                }
            }
        }
        if (null != this.startPoints) {
            for (CarrierState sp : this.startPoints) {
                if (sp.contains(pt)) {
                    l.add(sp);
                }
            }
        }
        if (null != this.goalPoints) {
            for (CarrierState gp : this.goalPoints) {
                if (gp.contains(pt)) {
                    l.add(gp);
                }
            }
        }
        if (null != this.waypoints) {
            for (CarrierState wp : this.waypoints) {
                if (wp.contains(pt)) {
                    l.add(wp);
                }
            }
        }
        return l;
    }
    private Point2Dd startDragPt;
    private boolean drag_moves_selection = false;
    private Rectangle2Dd select_rect = null;

    public void updatePlannerInfo(CarrierState startPoint,
            CarrierState goalPoint,
            List<PlannerPoint> _plannerPoints,
            List<PlannerPoint> _cntrlPoints) {
        this.incrementDelayReplanCount();
        if (null != startPoint) {
            startPoint.setGoal(goalPoint);
            if (null != this.startPoints && this.startPoints.size() == 1
                    && startPoint == this.startPoints.get(0)) {
                // skip it this startPoint already  is the list
            } else if (null == this.startPoints || this.startPoints.size() < 2) {
                this.setStartPoints(new LinkedList<CarrierState>(Arrays.asList(startPoint)));
            } else {
                if (!startPoints.contains(startPoint)) {
                    CarrierState closestPoint = null;
                    double min_dist = Double.POSITIVE_INFINITY;
                    for (CarrierState sp : this.startPoints) {
                        double dist = sp.distance(startPoint) + Planner.chord(startPoint.unit(), sp.unit(),
                                minTurnRadius + plannerResolution);
                        if (dist < min_dist) {
                            min_dist = dist;
                            closestPoint = sp;
                        }
                    }
                    if (null != closestPoint) {
                        this.startPoints.remove(closestPoint);
                    }
                    this.startPoints.add(startPoint);
                }
            }
        }
        if (null != goalPoint) {
            goalPoint.setStart(startPoint);
            if (null != this.goalPoints && this.goalPoints.size() == 1
                    && goalPoint == this.goalPoints.get(0)) {
                // skip it
            } else if (this.goalPoints == null || this.goalPoints.size() < 2) {
                this.setGoalPoints(new LinkedList<CarrierState>(Arrays.asList(goalPoint)));
            } else {
                if (!goalPoints.contains(goalPoint)) {
                    CarrierState closestPoint = null;
                    double min_dist = Double.POSITIVE_INFINITY;
                    for (CarrierState gp : this.goalPoints) {
                        double dist = gp.distance(goalPoint) + Planner.chord(goalPoint.unit(), gp.unit(),
                                minTurnRadius + plannerResolution);
                        if (dist < min_dist) {
                            min_dist = dist;
                            closestPoint = gp;
                        }
                    }
                    if (null != closestPoint) {
                        this.goalPoints.remove(closestPoint);
                    }
                    this.goalPoints.add(goalPoint);
                }
            }
        }
        if (null != _cntrlPoints) {
            if (null == startPoint
                    || null == startPoint.getPath()
                    || null == startPoint.getPath().getControlPoints()
                    || _cntrlPoints != startPoint.getPath().getControlPoints()) {
                PlannedPath path = new PlannedPath(this);
                path.setStartPoint(startPoint);
                path.setGoalPoint(goalPoint);
                path.setPrePostPathDist(segStartLength);
                path.setControlPoints(_cntrlPoints);
                this.setPlannedPaths(Arrays.asList(path));
            }
        }
        this.decrementDelayReplanCount();
        this.repaint();
    }

    private static boolean checkClearSelectionModifiers(int _modifiers) {
        return ((_modifiers
                & (InputEvent.SHIFT_DOWN_MASK | InputEvent.CTRL_DOWN_MASK | InputEvent.SHIFT_MASK | InputEvent.CTRL_MASK)) == 0);
    }

    @Override
    public void mouseClicked(MouseEvent e) {
        //System.out.println("e = " + e);
        drag_moves_selection = false;
        if (e.isPopupTrigger()) {
            showPopup(e);
            return;
        }
    }

    private Point2Dd mouseEventToPoint2Dd(MouseEvent e) {
        return new Point2Dd(
                (e.getX() - translateX) / zoomScale,
                (this.getSize().height - translateY - e.getY()) / zoomScale);
    }

    public static String ObjectToString(Object o) {
        StringBuffer sb = new StringBuffer();
        Class clss = o.getClass();
        Method ma[] = clss.getDeclaredMethods();
        List<Method> lm = Arrays.asList(ma);
        Collections.sort(lm, new Comparator<Method>() {
            @Override
            public int compare(Method t, Method t1) {
                return t.getName().compareTo(t1.getName());
            }
        });
        ma = lm.toArray(new Method[lm.size()]);
        Field fa[] = clss.getFields();
        Set<String> feildNames = new HashSet<>();
        for (int i = 0; i < fa.length; i++) {
            Field fld = fa[i];
            if (null == fld) {
                continue;
            }
            String fn = fld.getName();
            sb.append(fn);
            sb.append("=");
            try {
                sb.append(fld.get(o));
            } catch (Exception e) {
                e.printStackTrace();
            }
            sb.append("\n");
            feildNames.add(fn);
        }
        for (int i = 0; i < ma.length; i++) {
            Method method = ma[i];
            if (method.getName().startsWith("get")) {
                Class paramtypes[] = method.getParameterTypes();
                if (null != paramtypes && paramtypes.length != 0) {
                    continue;
                }
                String fn = method.getName().substring(3);
                if (feildNames.contains(fn)) {
                    continue;
                }
                sb.append(fn);
                sb.append("=");
                try {
                    sb.append(method.invoke(o, (Object[]) null));
                } catch (Exception e) {
                    e.printStackTrace();
                }
                sb.append("\n");
            } else if (method.getName().startsWith("is")) {
                Class paramtypes[] = method.getParameterTypes();
                if (null != paramtypes && paramtypes.length != 0) {
                    continue;
                }
                sb.append(method.getName().substring(2));
                sb.append("=");
                try {
                    sb.append(method.invoke(o, (Object[]) null));
                } catch (Exception e) {
                    e.printStackTrace();
                }
                sb.append("\n");
            }
        }
        return sb.toString();
    }

    public void StringToObject(Object objectToModify, String s) {
        String var = "";
        String val = "";
        String line = "";
        try {
            BufferedReader br = new BufferedReader(new StringReader(s));
            Class clss = objectToModify.getClass();
            Method ma[] = clss.getDeclaredMethods();
            Map<String, Method> methodMap = new HashMap<>();
            for (Method m : ma) {
                methodMap.put(m.getName(), m);
            }
            Field fa[] = clss.getDeclaredFields();
            Map<String, Field> fieldMap = new HashMap<>();
            for (Field f : fa) {
                fieldMap.put(f.getName(), f);
            }
            while ((line = br.readLine()) != null) {
                int eq_index = line.indexOf('=');
                if (eq_index < 0) {
                    continue;
                }
                var = line.substring(0, eq_index).trim();
                val = line.substring(eq_index + 1).trim();
                Field fld = fieldMap.get(var);
                Object o = null;
                if (null != fld) {
                    Class fldType = fld.getType();
                    Method value_of = null;
                    try {
                        value_of = fldType.getDeclaredMethod("valueOf", String.class);
                        o = value_of.invoke((Class[]) null, val);
                    } catch (Exception e) {
                    }
                    if (fldType == double.class) {
                        o = Double.valueOf(val);
                    } else if (fldType == float.class) {
                        o = Float.valueOf(val);
                    } else if (fldType == int.class) {
                        o = Integer.valueOf(val);
                    } else if (fldType == boolean.class) {
                        o = Boolean.valueOf(val);
                    } else if (fldType.isAssignableFrom(LinkedList.class)) {
                        LinkedList ll = SplineTestJFrame.parseList(val);
                        o = ll;
                    }
                    if (null != o) {
                        fld.set(objectToModify, o);
                    } else {

                        fld.set(objectToModify, null);
                    }
                }
                String setMethodName = "set" + Character.toUpperCase(var.charAt(0)) + var.substring(1);
                Method set_method = methodMap.get(setMethodName);
                if (null != set_method) {
                    Class paramtypes[] = set_method.getParameterTypes();
                    if (null == paramtypes) {
                        continue;
                    }
                    if (paramtypes.length != 1) {
                        continue;
                    }
                    Method value_of = null;
                    try {
                        value_of = paramtypes[0].getDeclaredMethod("valueOf", String.class);
                        o = value_of.invoke((Class[]) null, val);
                    } catch (Exception e) {
                    }
                    if (paramtypes[0] == double.class) {
                        o = Double.valueOf(val);
                    } else if (paramtypes[0] == float.class) {
                        o = Float.valueOf(val);
                    } else if (paramtypes[0] == int.class) {
                        o = Integer.valueOf(val);
                    } else if (paramtypes[0] == boolean.class) {
                        o = Boolean.valueOf(val);
                    } else if (paramtypes[0].isAssignableFrom(LinkedList.class)) {
                        LinkedList ll = SplineTestJFrame.parseList(val);
                        o = ll;
                    }
                    if (null != o) {
                        set_method.invoke(objectToModify, o);
                    } else {
                        try {
                            if (val.length() < 1 || val.compareTo("null") == 0) {
                                set_method.invoke(objectToModify, new Object[]{null});
                            } else if (paramtypes[0] == String.class) {
                                set_method.invoke(objectToModify, val);
                            }
                        } catch (Exception e) {
                            System.err.println("line=" + line);
                            e.printStackTrace();
                        }
                    }
                } else {
                    System.err.println("No method in " + clss + " named " + setMethodName);
                }
            }
        } catch (Exception exception) {
            System.err.println("line=" + line);
            exception.printStackTrace();
        }
    }

    public void EditProperties(Object o) {
        String orig_props = ObjectToString(o);
        if (orig_props == null) {
            return;
        }
        String new_props = PropertiesJPanel.showDialog(null,
                orig_props);
        if (null == new_props || new_props.compareTo(orig_props) == 0) {
            return;
        }
        StringToObject(o, new_props);
        if (o.getClass().equals(Obstacle.class)) {
            this.setObstacles(obstacles);
        }
        if (this.replanOnAllChanges) {
            this.replan();
        }
        this.repaint();
    }

    private void showPopup(MouseEvent e) {
        Point2Dd pt = mouseEventToPoint2Dd(e);
        List obs_at_pt = this.findObjectsAtPt(pt);
        if (obs_at_pt == null || obs_at_pt.size() < 1) {
            return;
        }
        final JPopupMenu popup = new JPopupMenu();
        for (final Object o : obs_at_pt) {
            JMenu submenu = new JMenu(o.toString());
            if (CarrierState.class.isAssignableFrom(o.getClass())) {
                final CarrierState cs = (CarrierState) o;
                if (cs.getType() != CarrierStateTypeEnum.GOAL) {
                    List<CarrierState> goalPoints = this.getGoalPoints();
                    if (null != goalPoints || null != waypoints) {
                        JMenu setGoalMenu = new JMenu("Set Goal");
                        if (null != goalPoints) {
                            for (final CarrierState gp : goalPoints) {
                                JMenuItem mi = new JMenuItem(gp.toString());
                                mi.addActionListener(new ActionListener() {
                                    @Override
                                    public void actionPerformed(ActionEvent e) {
                                        cs.setGoal(gp);
                                        popup.setVisible(false);
                                        replan();
                                        repaint();
                                    }
                                });
                                setGoalMenu.add(mi);
                            }
                        }
                        if (null != waypoints) {
                            for (final CarrierState wp : waypoints) {
                                if (wp == cs) {
                                    continue;
                                }
                                JMenuItem mi = new JMenuItem(wp.toString());
                                mi.addActionListener(new ActionListener() {
                                    @Override
                                    public void actionPerformed(ActionEvent e) {
                                        cs.setGoal(wp);
                                        popup.setVisible(false);
                                        replan();
                                        repaint();
                                    }
                                });
                                setGoalMenu.add(mi);
                            }
                        }
                        submenu.add(setGoalMenu);
                    }
                    if (null != cs.getGoal()) {
                        JMenuItem mi = new JMenuItem("Clear Goal");
                        mi.addActionListener(new ActionListener() {
                            @Override
                            public void actionPerformed(ActionEvent e) {
                                cs.setGoal(null);
                                popup.setVisible(false);
                                replan();
                                repaint();
                            }
                        });
                    }
                    JMenuItem frontMenuItem = new JMenuItem("Bring to front of exclusive planning list.");
                    frontMenuItem.addActionListener(new ActionListener() {
                        @Override
                        public void actionPerformed(ActionEvent e) {
                            startPoints.remove(cs);
                            for (CarrierState csi : startPoints) {
                                csi.setId(csi.getId() + cs.getId() + 1);
                            }
                            startPoints.add(0, cs);
                            popup.setVisible(false);
                            replan();
                            repaint();
                        }
                    });
                    submenu.add(frontMenuItem);
                }

            }
            JMenuItem propsMi = new JMenuItem("Properties ...");
            propsMi.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    EditProperties(o);
                    if (CarrierState.class.isAssignableFrom(o.getClass())) {
                        final CarrierState cs = (CarrierState) o;
                        cs.updateShape();
                    }
                    popup.setVisible(false);
                    replan();
                    repaint();
                }
            });
            submenu.add(propsMi);
            JMenuItem deleteMi = new JMenuItem("Delete");
            deleteMi.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    DeleteObject(o);
                    popup.setVisible(false);
                    replan();
                    repaint();
                }
            });
            submenu.add(deleteMi);
            popup.add(submenu);
        }
        popup.show(e.getComponent(),
                e.getX(), e.getY());
    }
    private boolean debugObsDetComm;

    /**
     * Get the value of debugObsDetComm
     *
     * @return the value of debugObsDetComm
     */
    public boolean isDebugObsDetComm() {
        return debugObsDetComm;
    }

    /**
     * Set the value of debugObsDetComm
     *
     * @param debugObsDetComm new value of debugObsDetComm
     */
    public void setDebugObsDetComm(boolean debugObsDetComm) {
        this.debugObsDetComm = debugObsDetComm;
    }

    @Override
    public void mousePressed(MouseEvent e) {
        try {
            this.clearTmpGoal();
//            System.out.println("e = " + e);
            if (e.isPopupTrigger()) {
                showPopup(e);
                return;
            }
            Point2Dd pt = mouseEventToPoint2Dd(e);

            //System.out.println("pt = " + pt);
            this.startDragPt = pt;
            switch (drawMode) {

                case START:
//                StartOrGoalPoint oldStartPoint = this.startPoint;
//                if (oldStartPoint != null && oldStartPoint.distance(pt) < 10f) {
//                    double angle =  Math.atan2(pt.y - oldStartPoint.y, pt.x - oldStartPoint.x);
//                    System.out.println("goalPoint.angle = " + oldStartPoint.angle);
//                    this.setStartPoint(new StartOrGoalPoint(oldStartPoint.x, oldStartPoint.y, angle));
//                } else {
//                    this.setStartPoint(new StartOrGoalPoint(pt));
//                }

                    if (null == this.startPoints) {
                        this.startPoints = new LinkedList<>();
                    }
                    boolean sp_found = false;
                    for (CarrierState sp : this.startPoints) {
                        if (sp.contains(pt)) {
                            sp_found = true;
//                            sp.setAngle(new AngleD(Math.atan2(pt.y - sp.y, pt.x - sp.x)));
//                            sp.setLine(new Line2Dd(sp,
//                                    Point2Dd.addPtDistAngle(sp, this.plannerPointDisplaySize / 2.0, sp.getAngle())));
                        }
                    }
                    if (!sp_found) {
                        double sz = this.plannerPointDisplaySize;
                        double angle = Math.toRadians(this.startingAngle);
//                        String angleS = JOptionPane.showInputDialog(this, "Angle (in Degrees)",
//                                String.format("%.0f", this.startingAngle));
//                        if (angleS != null && angleS.length() > 0) {
//                            angle = Math.toRadians(Double.valueOf(angleS));
//                            this.startingAngle = Double.valueOf(angleS);
//                        }
                        CarrierState sp = new CarrierState(pt,
                                new AngleD(angle),
                                CarrierStateTypeEnum.START,
                                this.plannerPointDisplaySize,
                                this.vehicleWidth,
                                this.vehicleFront,
                                this.vehicleBack);
                        if (null != this.goalPoints) {
                            for (CarrierState gp : this.goalPoints) {
                                if (gp.getType() == CarrierStateTypeEnum.GOAL
                                        && gp.getStart() == null) {
                                    sp.setGoal(gp);
                                    break;
                                }
                            }
                        }
                        this.startPoints.add(sp);
                    }
                    this.replan();
                    this.repaint();
                    break;

                case GOAL:
//                StartOrGoalPoint oldGoalPoint = this.goalPoint;
//                if (oldGoalPoint != null && oldGoalPoint.distance(pt) < 10f) {
//                    double angle =  Math.atan2(pt.y - oldGoalPoint.y, pt.x - oldGoalPoint.x);
//                    System.out.println("goalPoint.angle = " + oldGoalPoint.angle);
//                    this.setGoalPoint(new StartOrGoalPoint(oldGoalPoint.x, oldGoalPoint.y, angle));
//                } else {
//                    this.setGoalPoint(new StartOrGoalPoint(pt));
//                }
                    if (null == this.goalPoints) {
                        this.goalPoints = new LinkedList<>();
                    }
                    boolean gp_found = false;
                    for (CarrierState gp : this.goalPoints) {
                        if (gp.contains(pt)) {
                            gp_found = true;
//                            gp.setAngle(new AngleD(Math.atan2(pt.y - gp.y,
//                                    pt.x - gp.x)));
//                            gp.setLine(new Line2Dd(gp,
//                                    Point2Dd.addPtDistAngle(gp,
//                                    this.plannerPointDisplaySize / 2.0, gp.getAngle())));
                        }
                    }
                    if (!gp_found) {
                        double sz = this.plannerPointDisplaySize;
                        double angle = Math.toRadians(this.startingAngle);
//                        String angleS = JOptionPane.showInputDialog(this, "Angle (in Degrees)",
//                                String.format("%.0f", this.startingAngle));
//                        if (angleS != null && angleS.length() > 0) {
//                            angle = Math.toRadians(Double.valueOf(angleS));
//                            this.startingAngle = Double.valueOf(angleS);
//                        }
                        CarrierState gp = new CarrierState(pt,
                                new AngleD(angle),
                                CarrierStateTypeEnum.GOAL,
                                this.plannerPointDisplaySize,
                                this.vehicleWidth,
                                this.vehicleFront,
                                this.vehicleBack);
                        if (null != this.startPoints) {
                            for (CarrierState sp : this.startPoints) {
                                if (sp.getType() == CarrierStateTypeEnum.START
                                        && sp.getGoal() == null) {
                                    gp.setStart(sp);
                                    break;
                                }
                            }
                        }
                        if (null == this.goalPoints) {
                            this.goalPoints = new LinkedList<CarrierState>();
                        }
                        try {
                            this.goalPoints.add(gp);
                        } catch (UnsupportedOperationException uoe) {
                            this.goalPoints = new LinkedList<>();
                            this.goalPoints.add(gp);
                        }
                    }
                    this.replan();
                    this.repaint();
                    break;

                case WAYPOINT:
//                StartOrGoalPoint oldGoalPoint = this.goalPoint;
//                if (oldGoalPoint != null && oldGoalPoint.distance(pt) < 10f) {
//                    double angle =  Math.atan2(pt.y - oldGoalPoint.y, pt.x - oldGoalPoint.x);
//                    System.out.println("goalPoint.angle = " + oldGoalPoint.angle);
//                    this.setGoalPoint(new StartOrGoalPoint(oldGoalPoint.x, oldGoalPoint.y, angle));
//                } else {
//                    this.setGoalPoint(new StartOrGoalPoint(pt));
//                }
                    if (null == this.waypoints) {
                        this.waypoints = new LinkedList<>();
                    }
                    boolean wp_found = false;
                    for (CarrierState wp : this.waypoints) {
                        if (wp.contains(pt)) {
                            wp_found = true;
//                            wp.setAngle(new AngleD(Math.atan2(pt.y - wp.y,
//                                    pt.x - wp.x)));
//                            wp.setLine(new Line2Dd(wp,
//                                    Point2Dd.addPtDistAngle(wp,
//                                    this.plannerPointDisplaySize / 2.0, wp.getAngle())));
                        }
                    }
                    if (!wp_found) {
                        double sz = this.plannerPointDisplaySize;
                        double angle = Math.toRadians(this.startingAngle);
//                        String angleS = JOptionPane.showInputDialog(this, "Angle (in Degrees)",
//                                String.format("%.0f", this.startingAngle));
//                        if (angleS != null && angleS.length() > 0) {
//                            angle = Math.toRadians(Double.valueOf(angleS));
//                            this.startingAngle = Double.valueOf(angleS);
//                        }
                        CarrierState wp = new CarrierState(pt,
                                new AngleD(angle),
                                CarrierStateTypeEnum.WAYPOINT,
                                this.plannerPointDisplaySize,
                                this.vehicleWidth,
                                this.vehicleFront,
                                this.vehicleBack);
                        if (null != this.startPoints) {
                            for (CarrierState sp : this.startPoints) {
                                if (sp.getType() == CarrierStateTypeEnum.START
                                        && sp.getGoal() == null) {
                                    wp.setStart(sp);
                                    break;
                                }
                            }
                        }
                        if (wp.getStart() == null) {
                            for (CarrierState wp2 : this.waypoints) {
                                if (wp == wp2) {
                                    continue;
                                }
                                if (wp2.getType() == CarrierStateTypeEnum.WAYPOINT
                                        && wp2.getGoal() == null) {
                                    wp.setStart(wp2);
                                    break;
                                }
                            }
                        }
                        if (wp.getStart() == null) {
                            for (CarrierState wp2 : this.waypoints) {
                                if (wp == wp2) {
                                    continue;
                                }
                                if (wp2.getType() == CarrierStateTypeEnum.WAYPOINT
                                        && wp2.getStart() == null) {
                                    wp.setGoal(wp2);
                                    break;
                                }
                            }
                        }
                        if (wp.getStart() == null && wp.getGoal() == null) {
                            if (null != this.goalPoints) {
                                for (CarrierState gp : this.goalPoints) {
                                    if (gp.getType() == CarrierStateTypeEnum.GOAL
                                            && gp.getStart() == null) {
                                        wp.setGoal(gp);
                                        break;
                                    }
                                }
                            }
                        }
                        try {
                            this.waypoints.add(wp);
                        } catch (UnsupportedOperationException uoe) {
                            this.waypoints = new LinkedList<>();
                            this.waypoints.add(wp);
                        }
                    }
                    this.replan();
                    this.repaint();
                    break;

                case OBSTACLE:
                    this.AddObstacle(new Obstacle(pt, this.defaultObsRadius));
                    break;

                case SELECT:
                    if (checkClearSelectionModifiers(e.getModifiers())) {
                        this.ClearSelection();
                    }
                    this.SelectByClick(pt);
                    this.startDragPt = pt;
                    List obs_at_pt = this.findObjectsAtPt(pt);
                    this.drag_moves_selection = (null != obs_at_pt && obs_at_pt.size() > 0);
                    break;
                case BOUNDARY:
                    this.AddBoundaryPoint(pt, false, false);
                    break;

                case HORZ_BOUNDARY:
                    this.AddBoundaryPoint(pt, true, false);
                    break;

                case VERT_BOUNDARY:
                    this.AddBoundaryPoint(pt, false, true);
                    break;
            }
        } catch (Exception except) {
            except.printStackTrace();
        }
    }
    private boolean plotAllCtrlPtsSent = false;

    /**
     * Get the value of plotAllCtrlPtsSent
     *
     * @return the value of plotAllCtrlPtsSent
     */
    public boolean isPlotAllCtrlPtsSent() {
        return plotAllCtrlPtsSent;
    }

    /**
     * Set the value of plotAllCtrlPtsSent
     *
     * @param plotAllCtrlPtsSent new value of plotAllCtrlPtsSent
     */
    public void setPlotAllCtrlPtsSent(boolean plotAllCtrlPtsSent) {
        this.plotAllCtrlPtsSent = plotAllCtrlPtsSent;
    }
    private boolean showBackgroundShapes;

    /**
     * Get the value of showBackgroundShapes
     *
     * @return the value of showBackgroundShapes
     */
    public boolean isShowBackgroundShapes() {
        return showBackgroundShapes;
    }

    /**
     * Set the value of showBackgroundShapes
     *
     * @param showBackgroundShapes new value of showBackgroundShapes
     */
    public void setShowBackgroundShapes(boolean showBackgroundShapes) {
        this.showBackgroundShapes = showBackgroundShapes;
    }

    private void checkPlannerPointListSelections(Rectangle2Dd rect,
            List<PlannerPoint> ll) {
        int num_selected = 0;
        for (PlannerPoint pp : ll) {
            if (pp.x >= rect.x
                    && pp.x <= rect.x + rect.width
                    && pp.y >= rect.y
                    && pp.y <= rect.y + rect.height) {
                pp.selected = true;
                num_selected++;
            }
        }
    }

    public void SelectByRectangle(Rectangle2Dd rect) {
        boolean obstacle_selected = false;
//        if (null != Planner.ll) {
//            checkPlannerPointListSelections(rect, Planner.ll);
//        }
        if (null != Planner.goalPPList) {
            checkPlannerPointListSelections(rect, Planner.goalPPList);
        }
        if (null != Planner.startPPList) {
            checkPlannerPointListSelections(rect, Planner.startPPList);
        }
        if (null != this.startPoints) {
            for (CarrierState cs : this.startPoints) {
                if (cs.x >= rect.x
                        && cs.x <= rect.x + rect.width
                        && cs.y >= rect.y
                        && cs.y <= rect.y + rect.height) {
                    cs.selected = true;
                }
                List<PlannerPoint> ppl = cs.getPlannerList();
                if (null != ppl) {
                    checkPlannerPointListSelections(rect, ppl);
                }
            }
        }
        if (null != this.goalPoints) {
            for (CarrierState cs : this.goalPoints) {
                if (cs.x >= rect.x
                        && cs.x <= rect.x + rect.width
                        && cs.y >= rect.y
                        && cs.y <= rect.y + rect.height) {
                    cs.selected = true;
                }
                List<PlannerPoint> ppl = cs.getPlannerList();
                if (null != ppl) {
                    checkPlannerPointListSelections(rect, ppl);
                }
            }
        }
        if (null != this.plannedPaths) {
            for (PlannedPath path : this.plannedPaths) {
                if (path.getControlPoints() != null) {
                    checkPlannerPointListSelections(rect, path.getControlPoints());
                }
            }
        }
        if (null != obstacles) {
            for (int i = 0; i < obstacles.size(); i++) {
                Obstacle o = obstacles.get(i);
                if (o.x >= rect.x
                        && o.x <= rect.x + rect.width
                        && o.y >= rect.y
                        && o.y <= rect.y + rect.height) {
                    o.selected = true;
                }
            }
        }
        if (!obstacle_selected) {
            if (null != boundaries) {
                for (int i = 0; i < boundaries.size(); i++) {
                    Boundary b = boundaries.get(i);
                    Point2Dd p1 = b.getP1();
                    Point2Dd p2 = b.getP2();
                    if (p1.x >= rect.x
                            && p1.x <= rect.x + rect.width
                            && p1.y >= rect.y
                            && p1.y <= rect.y + rect.height) {
                        p1.selected = true;
                        b.P1Selected = true;
                    }
                    if (p2.x >= rect.x
                            && p2.x <= rect.x + rect.width
                            && p2.y >= rect.y
                            && p2.y <= rect.y + rect.height) {
                        p2.selected = true;
                        b.P2Selected = true;
                    }
                }
            }
        }
        this.repaint();
    }
    private boolean replan_on_mouse_release = false;

    @Override
    public void mouseReleased(MouseEvent e) {
//        System.out.println("e = " + e);
        this.lastDraggedE = null;
        this.clearTmpGoal();
        if (e.isPopupTrigger()) {
            showPopup(e);
            if (replan_on_mouse_release) {
                this.replan();
                replan_on_mouse_release = false;
            }
            return;
        }
        if (!this.drag_moves_selection
                && this.drawMode == SplineDrawMode.SELECT
                && null != select_rect) {
            if (checkClearSelectionModifiers(e.getModifiers())) {
                this.ClearSelection();
            }
            this.SelectByRectangle(select_rect);
        }
        this.select_rect = null;
        if (replan_on_mouse_release) {
            this.replan();
            replan_on_mouse_release = false;
        }
        this.repaint();
        // do nothing
    }

    @Override
    public void mouseEntered(MouseEvent e) {
        // do nothing
    }

    @Override
    public void mouseExited(MouseEvent e) {
        this.clearTmpGoal();
        if (null != this.planToTmpGoalTimer) {
            this.planToTmpGoalTimer.stop();
            this.planToTmpGoalTimer = null;
        }
    }
    private boolean ShowGrid;

    /**
     * Get the value of ShowGrid
     *
     * @return the value of ShowGrid
     */
    public boolean isShowGrid() {
        return ShowGrid;
    }

    /**
     * Set the value of ShowGrid
     *
     * @param ShowGrid new value of ShowGrid
     */
    public void setShowGrid(boolean ShowGrid) {
        this.ShowGrid = ShowGrid;
        this.repaint();
    }
    private boolean LabelGrid;

    /**
     * Get the value of LabelGrid
     *
     * @return the value of LabelGrid
     */
    public boolean isLabelGrid() {
        return LabelGrid;
    }

    /**
     * Set the value of LabelGrid
     *
     * @param LabelGrid new value of LabelGrid
     */
    public void setLabelGrid(boolean LabelGrid) {
        this.LabelGrid = LabelGrid;
        this.repaint();
    }
    private Point2Dd last_obs_pt;

    public void swapGoalStart() {
        this.incrementDelayReplanCount();
        List<CarrierState> temp = this.startPoints;
        this.startPoints = this.goalPoints;
        this.goalPoints = temp;
        if (null != this.startPoints) {
            for (CarrierState sp : this.startPoints) {
                CarrierState gp = sp.getStart();
                gp.setStart(sp);
                sp.setGoal(gp);
                sp.setStart(null);
                gp.setType(CarrierStateTypeEnum.GOAL);
                sp.setType(CarrierStateTypeEnum.START);
            }
        }
        if (null != this.goalPoints) {
            for (CarrierState gp : this.goalPoints) {
                CarrierState sp = gp.getGoal();
                gp.setStart(sp);
                sp.setGoal(gp);
                gp.setGoal(null);
                gp.setType(CarrierStateTypeEnum.GOAL);
                sp.setType(CarrierStateTypeEnum.START);
            }
        }
        this.decrementDelayReplanCount();
        if (this.replanOnAllChanges) {
            this.replan();
        }
        this.repaint();
    }

    public void moveSelection(Point2Dd move) throws InterruptedException {
        if (null != this.selectedObjects && this.selectedObjects.size() > 0) {
            this.incrementDelayReplanCount();
            boolean update_obstacles = false;
            for (Object o : this.selectedObjects) {
                Class clss = o.getClass();
                if (Point2Dd.class.isAssignableFrom(clss)) {
                    Point2Dd pt = (Point2Dd) o;
                    pt.setLocation(pt.x + move.x, pt.y + move.y);
                    if (Obstacle.class.isAssignableFrom(clss)) {
                        update_obstacles = true;
                    }
                    continue;
                } else if (Boundary.class.isAssignableFrom(clss)) {
                    Boundary b = (Boundary) o;
                    if (b.P1Selected && b.P2Selected) {
                        b.setLine(b.x1 + move.x, b.y1 + move.y,
                                b.x2 + move.x, b.y2 + move.y);
                    } else if (b.P1Selected) {
                        b.setLine(b.x1 + move.x, b.y1 + move.y,
                                b.x2, b.y2);
                    } else if (b.P2Selected) {
                        b.setLine(b.x1, b.y1,
                                b.x2 + move.x, b.y2 + move.y);
                    }
                    continue;
                }
            }
            if (update_obstacles) {
                this.setObstacles(obstacles);
            }
            this.decrementDelayReplanCount();
            if (this.replanOnAllChanges) {
                this.replan();
            }
            repaint();
        }
//        
//        if (null != obstacles) {
//            LinkedList<Obstacle> obslist = new LinkedList<Obstacle>(this.obstacles);
//            for (Obstacle o : obslist) {
//                if (o.selected) {
//                    o.setLocation(o.x + move.x, o.y + move.y);
//                }
//            }
//            this.setObstacles(obslist);
//        }
//        if (null != boundaries) {
//            LinkedList<Boundary> blist = new LinkedList<>(this.boundaries);
//            for (int i = 0; i < blist.size(); i++) {
//                Boundary b = blist.get(i);
//                if (i > 0 && b.P1Selected) {
//                    Boundary bp = blist.get(i - 1);
//                    if (Math.abs(bp.x2 - b.x1) < this.BoundarySelectDist / 2f
//                            && Math.abs(bp.y2 - b.y1) < this.BoundarySelectDist / 2f) {
//                        bp.setLine(bp.x1, bp.y1, bp.x2 + move.x, bp.y2 + move.y);
//                    }
//                }
//                if (b.P1Selected && b.P2Selected) {
//                    b.setLine(b.x1 + move.x, b.y1 + move.y, b.x2 + move.x, b.y2 + move.y);
//                } else if (b.P1Selected) {
//                    b.setLine(b.x1 + move.x, b.y1 + move.y, b.x2, b.y2);
//                } else if (b.P2Selected) {
//                    b.setLine(b.x1, b.y1, b.x2 + move.x, b.y2 + move.y);
//                }
//                if (i < blist.size() - 1 && b.P2Selected) {
//                    Boundary bn = blist.get(i + 1);
//                    if (Math.abs(bn.x1 - b.x2) < this.BoundarySelectDist / 2f
//                            && Math.abs(bn.y1 - b.y2) < this.BoundarySelectDist / 2f) {
//                        bn.setLine(bn.x1 + move.x, bn.y1 + move.y, bn.x2, bn.y2);
//                    }
//                }
//            }
//            this.setBoundaries(blist);
//        }
//        if (null != this.startPoints) {
//            for (CarrierState sp : this.startPoints) {
//                if (sp.selected) {
//                    sp.setLocation(sp.x + move.x, sp.y + move.y);
//                }
//            }
//        }
//        if (null != this.goalPoints) {
//            for (CarrierState gp : this.goalPoints) {
//                if (gp.selected) {
//                    gp.setLocation(gp.x + move.x, gp.y + move.y);
//                }
//            }
//        }

    }
    private MouseEvent lastDraggedE = null;

    @Override
    public void mouseDragged(MouseEvent e) {
        try {
            this.clearTmpGoal();
            Point2Dd pt2 = mouseEventToPoint2Dd(e);;
            if ((e.getModifiersEx() & MouseEvent.BUTTON1_DOWN_MASK) != 0
                    && (e.getModifiersEx() & MouseEvent.BUTTON2_DOWN_MASK) == 0
                    && (e.getModifiersEx() & MouseEvent.BUTTON3_DOWN_MASK) == 0
                    && (e.getModifiersEx() & MouseEvent.ALT_DOWN_MASK) == 0) {
                switch (drawMode) {
                    case OBSTACLE:
                        Obstacle o = new Obstacle(
                                mouseEventToPoint2Dd(e),
                                this.defaultObsRadius);
                        if (last_obs_pt != null && last_obs_pt.distance(o) < this.defaultObsRadius) {
                            return;
                        }
                        this.AddObstacle(o);
                        last_obs_pt = o;
                        break;

                    case SELECT:

                        if (this.drag_moves_selection) {
                            this.moveSelection(pt2.diff(this.startDragPt));
                            this.startDragPt = pt2;
                        } else {
                            this.select_rect = new Rectangle2Dd(Math.min(pt2.x, this.startDragPt.x), Math.min(pt2.y, this.startDragPt.y),
                                    Math.abs(pt2.x - this.startDragPt.x), Math.abs(pt2.y - this.startDragPt.y));
                        }
                        this.repaint();
                        break;

                    case DRAW_PLANNING_RECT:
                        if (this.drag_moves_selection) {
                            this.moveSelection(pt2.diff(this.startDragPt));
                            this.startDragPt = pt2;
                        } else {
                            this.planningAreaRect = new Rectangle2Dd(Math.min(pt2.x, this.startDragPt.x), Math.min(pt2.y, this.startDragPt.y),
                                    Math.abs(pt2.x - this.startDragPt.x), Math.abs(pt2.y - this.startDragPt.y));
                        }
                        this.repaint();
                        break;

                    case PAN:
                        this.select_rect = null;
                        if (null != this.lastDraggedE) {
                            this.setTranslateX(this.translateX + e.getX() - lastDraggedE.getX());
                            this.setTranslateY(this.translateY - e.getY() + lastDraggedE.getY());
                        }
                        this.lastDraggedE = e;
                        this.repaint();

                    case START:
                        for (CarrierState sp : this.startPoints) {
                            if (sp.contains(pt2)) {
                                sp.setLocation(pt2);
                                this.repaint();
                                if (this.replanOnAllChanges) {
                                    this.replan_on_mouse_release = true;
                                }
                                break;
                            }
                        }
                        break;

                    case GOAL:
                        for (CarrierState gp : this.goalPoints) {
                            if (gp.contains(pt2)) {
                                gp.setLocation(pt2);
                                this.repaint();
                                if (this.replanOnAllChanges) {
                                    this.replan_on_mouse_release = true;
                                }
                                break;
                            }
                        }
                        break;

                    case WAYPOINT:
                        for (CarrierState wp : this.waypoints) {
                            if (wp.contains(pt2)) {
                                wp.setLocation(pt2);
                                this.repaint();
                                if (this.replanOnAllChanges) {
                                    this.replan_on_mouse_release = true;
                                }
                                break;
                            }
                        }
                        break;
                }
            } else if (
                    (e.getModifiersEx() & MouseEvent.BUTTON2_DOWN_MASK) != 0
                    || (e.getModifiersEx() & MouseEvent.ALT_DOWN_MASK) != 0) {
                switch (drawMode) {
                    case START:
                        this.checkStartPointAngles(e, pt2);
                        break;

                    case GOAL:
                        this.checkGoalPointAngles(e, pt2);
                        break;

                    case WAYPOINT:
                        this.checkWaypointAngles(e, pt2);
                        break;
                        
                    default:
                        this.checkStartPointAngles(e, pt2);
                        this.checkGoalPointAngles(e, pt2);
                        this.checkWaypointAngles(e, pt2);
                        break;
                }
            }
        } catch (Exception exception) {
            exception.printStackTrace();
        }
    }

    private void checkWaypointAngles(MouseEvent evt, Point2Dd pt2) {
        if(null != this.waypoints) {
            return;
        }
        for (CarrierState wp : this.waypoints) {
            if (wp.contains(pt2)) {
                wp.setAngle(new AngleD(Math.atan2(pt2.y - wp.y, pt2.x - wp.x)));
                wp.setLine(new Line2Dd(wp,
                        Point2Dd.addPtDistAngle(wp, this.plannerPointDisplaySize / 2.0, wp.getAngle())));
                if (this.crab) {
                    CarrierState sp = wp.getStart();
                    if (null != sp) {
                        sp.setAngle(wp.getAngle());
                        sp.setLine(new Line2Dd(sp,
                                Point2Dd.addPtDistAngle(sp, this.plannerPointDisplaySize / 2.0, sp.getAngle())));
                    }
                    CarrierState gp = wp.getGoal();
                    if (null != gp) {
                        gp.setAngle(wp.getAngle());
                        gp.setLine(new Line2Dd(sp,
                                Point2Dd.addPtDistAngle(sp, this.plannerPointDisplaySize / 2.0, gp.getAngle())));
                    }
                }
                this.repaint();
                if (this.replanOnAllChanges) {
                    this.replan_on_mouse_release = true;
                }
                break;
            }
        }
    }

    private void checkStartPointAngles(MouseEvent evt, Point2Dd pt2) {
        if(null != this.startPoints) {
            return;
        }
        for (CarrierState sp : this.startPoints) {
            if (sp.contains(pt2)) {
                sp.setAngle(new AngleD(Math.atan2(pt2.y - sp.y, pt2.x - sp.x)));
                sp.setLine(new Line2Dd(sp,
                        Point2Dd.addPtDistAngle(sp, this.plannerPointDisplaySize / 2.0, sp.getAngle())));
                if (this.crab) {
                    CarrierState gp = sp.getGoal();
                    if (null != gp) {
                        gp.setAngle(sp.getAngle());
                        gp.setLine(new Line2Dd(sp,
                                Point2Dd.addPtDistAngle(sp, this.plannerPointDisplaySize / 2.0, gp.getAngle())));
                    }
                }
                this.repaint();
                if (this.replanOnAllChanges) {
                    this.replan_on_mouse_release = true;
                }
                break;
            }
        }
    }

    private void checkGoalPointAngles(MouseEvent evt, Point2Dd pt2) {
        if(null != this.goalPoints) {
            return;
        }
        for (CarrierState gp : this.goalPoints) {
            if (gp.contains(pt2)) {
                gp.setAngle(new AngleD(Math.atan2(pt2.y - gp.y, pt2.x - gp.x)));
                gp.setLine(new Line2Dd(gp,
                        Point2Dd.addPtDistAngle(gp, this.plannerPointDisplaySize / 2.0, gp.getAngle())));
                if (this.crab) {
                    CarrierState sp = gp.getStart();
                    if (null != sp) {
                        sp.setAngle(gp.getAngle());
                        sp.setLine(new Line2Dd(sp,
                                Point2Dd.addPtDistAngle(sp, this.plannerPointDisplaySize / 2.0, sp.getAngle())));
                    }
                }
                this.repaint();
                if (this.replanOnAllChanges) {
                    this.replan_on_mouse_release = true;
                }
                break;
            }
        }
    }

    CarrierState tmpGoal = null;
    public boolean planToTmpGoalUpdated = true;
    javax.swing.Timer planToTmpGoalTimer = null;

    public void updatePlanToTmpGoal() {
        if (null != tmpGoal
                && !tmpGoal.isColliding() && this.plan_thread == null && !planToTmpGoalUpdated) {
            if (this.goalPoints == null || this.goalPoints.size() < 1) {
                if (null != this.startPoints && this.startPoints.size() > 0) {
                    for (CarrierState sp : this.startPoints) {
                        sp.setGoal(tmpGoal);
                    }
                    this.replan();
                }
            }
        }
        planToTmpGoalUpdated = true;
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        this.clearTmpGoal();
        if (this.drawMode == SplineDrawMode.GOAL) {
            Point2Dd pt2 = mouseEventToPoint2Dd(e);
            if (null != tmpGoal) {
                if (Math.abs(pt2.x - tmpGoal.x) <= Double.MIN_NORMAL
                        && Math.abs(pt2.y - tmpGoal.y) <= Double.MIN_NORMAL) {
                    return;
                }
            }
            double angle = Math.toRadians(this.startingAngle);
            tmpGoal = new CarrierState(pt2,
                    new AngleD(angle),
                    CarrierStateTypeEnum.GOAL,
                    this.plannerPointDisplaySize,
                    this.vehicleWidth,
                    this.vehicleFront,
                    this.vehicleBack);
            if (null != boundaries) {
                tmpGoal.checkBoundaries(boundaries);
            }
            if (null != obstacles) {
                tmpGoal.checkObstacles(obstacles);
            }
            this.planToTmpGoalUpdated = false;
            if (null == this.planToTmpGoalTimer) {
                this.planToTmpGoalTimer = new javax.swing.Timer(500, new ActionListener() {
                    @Override
                    public void actionPerformed(ActionEvent ae) {
                        updatePlanToTmpGoal();
                    }
                });
                this.planToTmpGoalTimer.start();
            }
            this.repaint();
        }
    }

    /**
     * @return the front
     */
    public double getVehicleFront() {
        if (reverse) {
            return vehicleBack;
        }
        return vehicleFront;
    }

    /**
     * @param front the front to set
     */
    public void setVehicleFront(double front) {
//        if (reverse) {
//            double back = front;
//            this.vehicleBack = back;
//            if (null != this.goalPoints) {
//                for (CarrierState cs : this.goalPoints) {
//                    cs.setBack(this.vehicleBack);
//                }
//            }
//            if (null != this.startPoints) {
//                for (CarrierState cs : this.startPoints) {
//                    cs.setBack(this.vehicleBack);
//                }
//            }
//            return;
//        }
        this.vehicleFront = front;
        if (null != this.goalPoints) {
            for (CarrierState cs : this.goalPoints) {
                cs.setFront(this.vehicleFront);
            }
        }
        if (null != this.startPoints) {
            for (CarrierState cs : this.startPoints) {
                cs.setFront(this.vehicleFront);
            }
        }
        if (null != this.waypoints) {
            for (CarrierState cs : this.waypoints) {
                cs.setFront(this.vehicleFront);
            }
        }
        if (null != this.carriers) {
            for (CarrierState cs : this.carriers) {
                cs.setFront(this.vehicleFront);
            }
        }
    }

    /**
     * @return the back
     */
    public double getVehicleBack() {
        if (reverse) {
            return vehicleFront;
        }
        return vehicleBack;
    }

    /**
     * @param back the back to set
     */
    public void setVehicleBack(double back) {
//        if (reverse) {
//            this.vehicleFront = back;
//            if (null != this.goalPoints) {
//                for (CarrierState cs : this.goalPoints) {
//                    cs.setFront(this.vehicleFront);
//                }
//            }
//            if (null != this.startPoints) {
//                for (CarrierState cs : this.startPoints) {
//                    cs.setFront(this.vehicleFront);
//                }
//            }
//            return;
//        }
        this.vehicleBack = back;
        if (null != this.goalPoints) {
            for (CarrierState cs : this.goalPoints) {
                cs.setBack(this.vehicleBack);
            }
        }
        if (null != this.startPoints) {
            for (CarrierState cs : this.startPoints) {
                cs.setBack(this.vehicleBack);
            }
        }
        if (null != this.waypoints) {
            for (CarrierState cs : this.waypoints) {
                cs.setBack(this.vehicleBack);
            }
        }
        if (null != this.carriers) {
            for (CarrierState cs : this.carriers) {
                cs.setBack(this.vehicleBack);
            }
        }
        this.repaint();
    }

    /**
     * @return the startPoints
     */
    public List<CarrierState> getStartPoints() {
        return startPoints;
    }

    /**
     * @param startPoints the startPoints to set
     */
    public void setStartPoints(List<CarrierState> startPoints) {
        this.startPoints = startPoints;
        if (null != this.startPoints) {
            for (CarrierState cs : this.startPoints) {
                cs.setType(CarrierStateTypeEnum.START);
                cs.setWidth(this.getVehicleWidth());
                cs.setFront(this.getVehicleFront());
                cs.setBack(this.getVehicleBack());
            }
        }
        this.replan();
        this.repaint();
    }

    /**
     * @return the goalPoints
     */
    public List<CarrierState> getGoalPoints() {
        return goalPoints;
    }

    /**
     * @param goalPoints the goalPoints to set
     */
    public void setGoalPoints(List<CarrierState> goalPoints) {
        this.goalPoints = goalPoints;
        if (null != this.goalPoints) {
            for (CarrierState cs : this.goalPoints) {
                if (null == cs) {
                    continue;
                }
                cs.setType(CarrierStateTypeEnum.GOAL);
                cs.setWidth(this.getVehicleWidth());
                cs.setFront(this.getVehicleFront());
                cs.setBack(this.getVehicleBack());
            }
        }
        this.replan();
        this.repaint();
    }

    /**
     * @return the plannedPaths
     */
    public List<PlannedPath> getPlannedPaths() {
        return plannedPaths;
    }

    /**
     * @param plannedPaths the plannedPaths to set
     */
    public void setPlannedPaths(List<PlannedPath> plannedPaths) {
        this.plannedPaths = plannedPaths;
    }
    private int simDelayMs = 10;

    /**
     * Get the value of simDelayMs
     *
     * @return the value of simDelayMs
     */
    public int getSimDelayMs() {
        return simDelayMs;
    }

    /**
     * Set the value of simDelayMs
     *
     * @param simDelayMs new value of simDelayMs
     */
    public void setSimDelayMs(int simDelayMs) {
        this.simDelayMs = simDelayMs;
    }
    javax.swing.Timer sim_timer;
    private List<CarrierState> carriers;

    public List<CarrierState> getSelectedCarriers() {
        List<CarrierState> l = new LinkedList<>();
        if(null != carriers) {
            for(CarrierState cs : carriers) {
            if(cs.selected && !l.contains(cs))
                l.add(cs);
            }
        }
        if(null != waypoints) {
            for(CarrierState cs : waypoints) {
            if(cs.selected && !l.contains(cs))
                l.add(cs);
            }
        }
        if(null != this.goalPoints) {
            for(CarrierState cs : goalPoints) {
            if(cs.selected && !l.contains(cs))
                l.add(cs);
            }
        }
        if(null != this.startPoints) {
            for(CarrierState cs : startPoints) {
            if(cs.selected && !l.contains(cs))
                l.add(cs);
            }
        }
        return l;
    }
    /**
     * Get the value of carriers
     *
     * @return the value of carriers
     */
    public List<CarrierState> getCarriers() {
        return carriers;
    }

    /**
     * Set the value of carriers
     *
     * @param carriers new value of carriers
     */
    public void setCarriers(List<CarrierState> carriers) {
        this.carriers = carriers;
    }
    private int next_goal_index = 0;
    private List<CarrierState> simGoals = this.goalPoints;

    private CarrierState nextSimGoal(CarrierStateTypeEnum _type) {
        simGoals = this.goalPoints;
        if (_type == CarrierStateTypeEnum.GOAL) {
            simGoals = this.startPoints;
        }
        if (next_goal_index >= simGoals.size() - 1) {
            next_goal_index = 0;
        } else {
            next_goal_index++;
        }
//        System.out.println("next_goal_index = " + next_goal_index);
        return simGoals.get(next_goal_index);
    }
    private double simMoveDist = 0.5f;

    /**
     * Get the value of simMoveDist
     *
     * @return the value of simMoveDist
     */
    public double getSimMoveDist() {
        return simMoveDist;
    }

    /**
     * Set the value of simMoveDist
     *
     * @param simMoveDist new value of simMoveDist
     */
    public void setSimMoveDist(double simMoveDist) {
        this.simMoveDist = simMoveDist;
    }
    private double simAngleChange = (Math.PI / 180.0);

    /**
     * Get the value of simAngleChange
     *
     * @return the value of simAngleChange
     */
    public double getSimAngleChange() {
        return simAngleChange;
    }

    /**
     * Set the value of simAngleChange
     *
     * @param simAngleChange new value of simAngleChange
     */
    public void setSimAngleChange(double simAngleChange) {
        this.simAngleChange = simAngleChange;
    }
    diagapplet.plotter.plotterJFrame plotFrame = null;
    diagapplet.plotter.PlotData transvel_data = null;
    diagapplet.plotter.PlotData rotvel_data = null;
    long sim_time = 0;
    private boolean showPlotter = false;

    /**
     * Get the value of showPlotter
     *
     * @return the value of showPlotter
     */
    public boolean isShowPlotter() {
        return showPlotter;
    }

    /**
     * Set the value of showPlotter
     *
     * @param showPlotter new value of showPlotter
     */
    public void setShowPlotter(boolean showPlotter) {
        this.showPlotter = showPlotter;
    }
    private boolean showControlPath = false;

    /**
     * Get the value of showControlPoints
     *
     * @return the value of showControlPoints
     */
    public boolean isShowControlPath() {
        return showControlPath;
    }

    /**
     * Set the value of showControlPoints
     *
     * @param showControlPath new value of showControlPoints
     */
    public void setShowControlPath(boolean showControlPath) {
        this.showControlPath = showControlPath;
        this.repaint();
    }
//    private void addCarrierStateBoundaries(List<Boundary> boundaries_list,
//            CarrierState cs) {
//        AngleD csj_angle = cs.getAngle();
//        double c = csj_angle.cos();
//        double s = csj_angle.sin();
//        Point2Dd left_rear_corner = new Point2Dd(
//                cs.x - vehicleBack * c + vehicleWidth / 2f * s,
//                cs.y - vehicleBack * s - vehicleWidth / 2f * c);
//        Point2Dd right_rear_corner = new Point2Dd(
//                cs.x - vehicleBack * c - vehicleWidth / 2f * s,
//                cs.y - vehicleBack * s + vehicleWidth / 2f * c);
//        Point2Dd right_front_corner = new Point2Dd(
//                cs.x + vehicleFront * c - vehicleWidth / 2f * s,
//                cs.y + vehicleFront * s + vehicleWidth / 2f * c);
//        Point2Dd left_front_corner = new Point2Dd(
//                cs.x + vehicleFront * c + vehicleWidth / 2f * s,
//                cs.y + vehicleFront * s - vehicleWidth / 2f * c);
//        boundaries_list.add(new Boundary(left_rear_corner, right_rear_corner));
//        boundaries_list.add(new Boundary(left_rear_corner, left_front_corner));
//        boundaries_list.add(new Boundary(right_front_corner, right_rear_corner));
//        boundaries_list.add(new Boundary(right_rear_corner, left_front_corner));
//        if (cs.getTransVel() != 0.0 || cs.getRotVel() != 0.0) {
//            AngleD new_csj_angle = AngleD.add(csj_angle, new AngleD(cs.getRotVel()));
//            c = new_csj_angle.cos();
//            s = new_csj_angle.sin();
//            Point2Dd proj_pt = Point2Dd.addPtDistAngle(cs, cs.getTransVel(), new_csj_angle);
//            Point2Dd proj_left_rear_corner = new Point2Dd(
//                    proj_pt.x - vehicleBack * c + vehicleWidth / 2f * s,
//                    proj_pt.y - vehicleBack * s - vehicleWidth / 2f * c);
//            Point2Dd proj_right_rear_corner = new Point2Dd(
//                    proj_pt.x - vehicleBack * c - vehicleWidth / 2f * s,
//                    proj_pt.y - vehicleBack * s + vehicleWidth / 2f * c);
//            Point2Dd proj_right_front_corner = new Point2Dd(
//                    proj_pt.x + vehicleFront * c - vehicleWidth / 2f * s,
//                    proj_pt.y + vehicleFront * s + vehicleWidth / 2f * c);
//            Point2Dd proj_left_front_corner = new Point2Dd(
//                    proj_pt.x + vehicleFront * c + vehicleWidth / 2f * s,
//                    proj_pt.y + vehicleFront * s - vehicleWidth / 2f * c);
//            boundaries_list.add(new Boundary(proj_left_rear_corner, proj_right_rear_corner));
//            boundaries_list.add(new Boundary(proj_left_rear_corner, proj_left_front_corner));
//            boundaries_list.add(new Boundary(proj_right_front_corner, proj_right_rear_corner));
//            boundaries_list.add(new Boundary(proj_right_rear_corner, proj_left_front_corner));
//        }
//    }
    int sim_cycles = 0;

    private void runSimulation() {
        if (null == carriers || carriers.size() < 1) {
            this.StopSimulation();
            return;
        }
        try {
            sim_cycles++;
            //System.out.println("sim_cycles = " + sim_cycles);
//            if (this.pathUncertainty < this.vehicleWidth / 4.0) {
//                this.setPathUncertainty(this.vehicleWidth / 4.0);
//            }
            sim_time += this.simDelayMs;
            if (plotFrame == null && showPlotter) {
                plotFrame = new plotterJFrame();
                transvel_data = new PlotData();
                rotvel_data = new PlotData();
                plotFrame.AddPlot(transvel_data, "transVel");
                plotFrame.AddPlot(rotvel_data, "rotVel");
                plotFrame.setVisible(true);
            }
            PlannerInput pi = this.getPlannerInput();
            pi.obstacles = this.obstacles;
            pi.veh_width = this.vehicleWidth;
            pi.front = this.vehicleFront;
            pi.back = this.vehicleBack;
            pi.crab = this.crab;
            pi.reverse = this.reverse;
            pi.min_turn_radius = this.getMinTurnRadius();
            pi.max_turn_angle_degrees = this.getMaxTurnAngleDegrees();
            pi.plannerResolution = this.getPlannerResolution();
            pi.max_pt2pt_dist = this.getMax_pt2pt_dist();
            pi.planningHorizon = this.getPlanningHorizon();
            double vb = this.getVehicleBack();
            double vf = this.getVehicleFront();
            double vw = this.getVehicleWidth();
            for (int i = 0; i < this.carriers.size(); i++) {
                CarrierState csi = this.carriers.get(i);
                if (csi.isColliding() || csi.getType() == CarrierStateTypeEnum.ESTOPPED) {
                    continue;
                }
                int csi_failed_cycle = csi.getSimCyclePlanFailed();
                if (csi_failed_cycle > 0 && csi_failed_cycle < sim_cycles + 20) {
                    continue;
                }
                if (null != this.boundaries) {
                    if (csi.checkBoundaries(this.boundaries)) {
                        csi.setType(CarrierStateTypeEnum.ESTOPPED);
                        continue;
                    }
                }
                if (null != this.obstacles) {
                    if (csi.checkObstacles(this.obstacles)) {
                        csi.setType(CarrierStateTypeEnum.ESTOPPED);
                        continue;
                    }
                }
                List<Boundary> curBoundaries = new LinkedList<>();
                if (null != this.boundaries) {
                    curBoundaries.addAll(this.boundaries);
                }
                for (int j = 0; j < this.carriers.size(); j++) {
                    if (i == j) {
                        continue;
                    }
                    CarrierState csj = this.carriers.get(j);
//                    if (csj.distance(cs)
//                            > cs.getTransVel() * 4 + cs.getBack() + cs.getFront() + cs.getWidth()
//                            + csj.getTransVel() * 4 + csj.getBack() + csj.getFront() + csj.getWidth()) {
//                        continue;
//                    }
                    List<Boundary> csj_immediate_boundaries = csj.getImmediateBoundaries();
                    if (csi.checkBoundaries(csj_immediate_boundaries)) {
                        csi.setType(CarrierStateTypeEnum.ESTOPPED);
                    }
                    curBoundaries.addAll(csj_immediate_boundaries);
                    if (csj.getType() == CarrierStateTypeEnum.LIVE) {
                        PlannedPath path = csj.getPath();
                        if (this.isExclusivePaths() && null != path) {
                            curBoundaries.addAll(path.getPlanOutLine());
                        }
                    }
                    curBoundaries.addAll(csj.getProjectedBoundaries(0.5));
                    curBoundaries.addAll(csj.getProjectedBoundaries(1.0));
                    curBoundaries.addAll(csj.getProjectedBoundaries(1.5));
                    curBoundaries.addAll(csj.getProjectedBoundaries(2.0));
                    curBoundaries.addAll(csj.getProjectedBoundaries(2.5));
                    curBoundaries.addAll(csj.getProjectedBoundaries(3.0));
                }
                if (csi.isColliding() || csi.getType() == CarrierStateTypeEnum.ESTOPPED) {
                    continue;
                }
                csi.setBoundaries(curBoundaries);
                if (ignoreBoundaries) {
                    pi.boundaries = null;
                } else {
                    pi.boundaries = csi.getBoundaries();
                }
                CarrierState goal = csi.getGoal();
                int goals_checked = 0;
                boolean no_goal_found = false;
                boolean goal_too_close_to_carrier = false;
                if (null != goal) {
                    AngleD goal_angle_diff = AngleD.diff(goal.getAngle(), csi.getAngle());
                    if (goal.distance(csi) < this.simMoveDist * 3f
                            && goal_angle_diff.getValue() < this.simAngleChange * 3f) {
//                        System.out.println("goal completed : " + goal);
                        CarrierState old_goal = csi.getGoal();
                        CarrierState old_start = csi.getStart();
                        csi.setGoal(null);
                        csi.setStart(null);
                        csi.setGoal(old_start);
                        csi.setStart(old_goal);
                        csi.setPath(null);
                        goal = old_start;
                    }
                }
                while (goal == null
                        || goal_too_close_to_carrier) {
                    if (null != this.simGoals && goals_checked > this.simGoals.size()) {
                        no_goal_found = true;
                        break;
                    }
                    goal = nextSimGoal(csi.getLastGoalType());
                    goal_too_close_to_carrier = false;
                    for (int j = 0; j < this.carriers.size(); j++) {
                        CarrierState csj = this.carriers.get(j);
                        if (csj.distance(goal) < 50) {
                            goal_too_close_to_carrier = true;
                            break;
                        }
                    }
                    goals_checked++;
                }
                if (no_goal_found) {
                    continue;
                }
                csi.setGoal(goal);
                csi.setType(CarrierStateTypeEnum.LIVE);
                PlannedPath path = csi.getPath();
                boolean skip_replan = false;
                if (path == null) {
                    path = new PlannedPath(this);
                } else if (path.checkOutlines(csi.getBoundaries(), this.obstacles)) {
                    skip_replan = true;
                }
                //path.setPrePostPathDist(5.0);
                path.setStartPoint(csi);
                path.setGoalPoint(goal);
                pi.start = csi;
                pi.goal = goal;
                if (!skip_replan) {
                    csi.setPlannerList(Planner.createPlannerList(pi, this.startPoints, this.goalPoints, waypoints));
                    List<PlannerPoint> newControlPoints = Planner.planWithPlannerList(pi, csi.getPlannerList());
                    if (null == newControlPoints || newControlPoints.size() < 1) {
                        csi.setPath(null);
                        csi.setTransVel(0);
                        csi.setRotVel(0);
                        csi.setSimCyclePlanFailed(sim_cycles);
                        path = null;
                        continue;
                    }
                    path.setPrePostPathDist(segStartLength);
                    path.setControlPoints(newControlPoints);
                    csi.setPath(path);
                    csi.setCurrentCurvePointsIndex(0);
                    if (!path.checkOutlines(csi.getBoundaries(), this.obstacles)) {
                        System.err.println("path.checkOutlines() failed");
                        path = null;
                        csi.setPath(null);
                        csi.setTransVel(0);
                        csi.setRotVel(0);
                        continue;
                    }
                }
                List<Point2Dd> curve = path.getCurvePoints();
                double dist = 0;
                int k = csi.getCurrentCurvePointsIndex();
                Point2Dd last_pt = csi;
                Point2Dd pt = csi;
                Point2Dd diff;
                Point2Dd diffu;
                AngleD last_angle = new AngleD(csi.getAngle());
                AngleD angle_change = new AngleD(0);
                AngleD angle_diff = new AngleD(0);
                AngleD new_angle = new AngleD(csi.getAngle());
                double segment_dist = 0;
                LinkedList<AngleD> angle_diffs = new LinkedList<>();
                if (csi.distance(goal) > simMoveDist + 10f) {
                    while (dist < simMoveDist
                            && Math.abs(angle_change.getValue()) < this.simAngleChange
                            && k < curve.size()) {
                        Point2Dd next_pt = curve.get(k);
                        segment_dist = next_pt.diff(pt).mag();
                        dist += segment_dist;
                        if (segment_dist > 0.1f) {
                            last_pt = pt;
                            pt = next_pt;
                            diff = pt.diff(last_pt);
                            diffu = diff.unit();
                            if (!crab) {
                                last_angle.setValue(new_angle.getValue());
                                new_angle.setValue(Math.atan2(diff.y, diff.x));
                                angle_diff = AngleD.diff(new_angle, last_angle);
                                angle_diffs.add(angle_diff);
                                angle_change = AngleD.add(angle_change, angle_diff);
                            }
                        }
                        pt = next_pt;
                        k++;
                    }
                    if (k >= curve.size() && dist < simMoveDist
                            && angle_change.getValue() < this.simAngleChange) {
                        segment_dist = csi.getGoal().diff(pt).mag();
                        dist += segment_dist;
                        last_pt = pt;
                        pt = csi.getGoal();
                        diff = pt.diff(last_pt);
                        diffu = diff.unit();
                        if (!crab) {
                            last_angle.setValue(new_angle.getValue());
                            new_angle.setValue(csi.getGoal().getAngle().getValue());
                            angle_diff = AngleD.diff(new_angle, last_angle);
                            angle_change = AngleD.add(angle_change, angle_diff);
                        }
                    }
                } else {
                    last_pt = csi;
                    pt = goal;
                    diff = pt.diff(last_pt);
                    segment_dist = diff.mag();
                    dist = segment_dist;
                    diffu = diff.unit();
                    if (!crab) {
                        last_angle.setValue(csi.getAngle().getValue());
                        new_angle.setValue(goal.getAngle().getValue());
                        angle_diff = AngleD.diff(new_angle, last_angle);
                        angle_change = AngleD.add(angle_change, angle_diff);
                    }
                }
                double move_interpolation_scale = 1f;
                if (dist > simMoveDist) {
                    move_interpolation_scale = (simMoveDist - (dist - segment_dist)) / segment_dist;
                }
                double angle_interpolation_scale = 1f;
                if (!crab && Math.abs(angle_change.getValue()) > simAngleChange) {
                    angle_interpolation_scale = (simAngleChange - (Math.abs(angle_change.getValue()) - Math.abs(angle_diff.getValue()))) / Math.abs(angle_diff.getValue());
                }
                double interpolation_scale = Math.min(move_interpolation_scale, angle_interpolation_scale);
                AngleD a2 = AngleD.interpolate(last_angle, new_angle, interpolation_scale);
                Point2Dd new_pt = new Point2Dd(
                        pt.x * interpolation_scale + last_pt.x * (1 - interpolation_scale),
                        pt.y * interpolation_scale + last_pt.y * (1 - interpolation_scale));
                double transVel = (new_pt.distance(csi));
                if (transVel > 1.01f * simMoveDist) {
                    throw new Exception("new_pt.distance(cs) = " + new_pt.distance(csi));
                }
                csi.setLocation(new_pt);
                double rotVel = 0;
                if (!crab) {
                    rotVel = AngleD.diff(a2, csi.getAngle()).getValue();
                    if (Math.abs(rotVel) > 1.01f * this.simAngleChange) {
                        System.out.println("angle_diffs = " + angle_diffs);
                        throw new Exception("rotVel = " + rotVel);
                    }
                    csi.setRotVel(rotVel);
                    csi.setAngle(a2);
                }
                csi.setTransVel(transVel);
                if (showPlotter && null != plotFrame) {
                    plotFrame.AddPointToPlot(transvel_data, sim_time, new_pt.distance(csi), true);
                    if (!crab) {
                        plotFrame.AddPointToPlot(rotvel_data, sim_time, rotVel, true);
                    }
                    if (sim_time < 3000) {
                        plotFrame.FitToGraph();
                    } else {
                        plotFrame.ScrollRight();
                    }
                }
                csi.setCurrentCurvePointsIndex(k > 0 ? (k - 1) : k);
                Path2D.Double cs_shape = new Path2D.Double();
                double c = csi.getAngle().cos();
                double s = csi.getAngle().sin();
                cs_shape.moveTo(
                        csi.x - vb * c + vw / 2f * s,
                        csi.y - vb * s - vw / 2f * c);
                cs_shape.lineTo(
                        csi.x - vb * c - vw / 2f * s,
                        csi.y - vb * s + vw / 2f * c);
                cs_shape.lineTo(
                        csi.x + vf * c - vw / 2f * s,
                        csi.y + vf * s + vw / 2f * c);
                cs_shape.lineTo(
                        csi.x + vf * c + vw / 2f * s,
                        csi.y + vf * s - vw / 2f * c);
                cs_shape.lineTo(
                        csi.x - vb * c + vw / 2f * s,
                        csi.y - vb * s - vw / 2f * c);
                csi.setShape(cs_shape);
            }
            this.repaint();
        } catch (Exception exception) {
            exception.printStackTrace();
        }
    }
    private boolean running_simulation = false;

    public void StartSimulation() throws InterruptedException {
        this.StopSimulation();
        if (null == this.startPoints) {
            return;
        }
        this.running_simulation = true;
        this.ShowOutline = false;
        this.plannedPaths = null;
        List<CarrierState> newCarrierList = new LinkedList<>();
        for (CarrierState sp : this.startPoints) {
            CarrierState cs = new CarrierState(sp.x, sp.y, sp.getAngle(),
                    CarrierStateTypeEnum.LIVE,
                    this.getPlannerPointDisplaySize(),
                    this.getVehicleWidth(),
                    this.getVehicleFront(),
                    this.getVehicleBack());
            cs.setGoal(sp.getGoal());
            cs.setStart(sp);
            newCarrierList.add(cs);
        }
        this.setCarriers(newCarrierList);
        if (null != this.plan_thread) {
            this.plan_thread.interrupt();
            this.plan_thread.join();
            this.plan_thread = null;
        }
        this.sim_timer = new javax.swing.Timer(simDelayMs, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                runSimulation();
            }
        });
        this.sim_timer.start();
    }

    public void StopSimulation() {
        if (null != sim_timer) {
            sim_timer.stop();
            sim_timer = null;
        }
        if (this.carriers != null) {
            int ync = JOptionPane.showConfirmDialog(this,
                    "Set start positions to carrier positions?",
                    "", JOptionPane.YES_NO_OPTION);
            if (ync == JOptionPane.YES_OPTION) {
                this.setStartPoints(carriers);
                for (CarrierState cs : this.startPoints) {
                    cs.setType(CarrierStateTypeEnum.START);
                    cs.updateShape();
                }
            }
            ync = JOptionPane.showConfirmDialog(this, "Clear carrier positions?",
                    "", JOptionPane.YES_NO_OPTION);
            if (ync == JOptionPane.YES_OPTION) {
                this.carriers = null;
            }
        }
        this.running_simulation = false;
        this.repaint();
    }

    /**
     * @return the backgroundShapes
     */
    public List<ShapeWTransform> getBackgroundShapes() {
        return backgroundShapes;
    }

    /**
     * @param backgroundShapes the backgroundShapes to set
     */
    public void setBackgroundShapes(List<ShapeWTransform> backgroundShapes) {
        this.backgroundShapes = backgroundShapes;
        this.repaint();
    }

    /**
     * @return the defaultObsRadius
     */
    public double getDefaultObsRadius() {
        return defaultObsRadius;
    }

    /**
     * @param defaultObsRadius the defaultObsRadius to set
     */
    public void setDefaultObsRadius(double defaultObsRadius) {
        this.defaultObsRadius = defaultObsRadius;
    }

    /**
     * @return the max_pt2pt_dist
     */
    public double getMax_pt2pt_dist() {
        return max_pt2pt_dist;
    }

    /**
     * @param max_pt2pt_dist the max_pt2pt_dist to set
     */
    public void setMax_pt2pt_dist(double max_pt2pt_dist) {
        this.max_pt2pt_dist = max_pt2pt_dist;
    }
    private GoalSourceEnum goalSource = GoalSourceEnum.CONNECTION;

    /**
     * Get the value of goalSource
     *
     * @return the value of goalSource
     */
    public GoalSourceEnum getGoalSource() {
        return goalSource;
    }

    /**
     * Set the value of goalSource
     *
     * @param goalSource new value of goalSource
     */
    public void setGoalSource(GoalSourceEnum goalSource) {
        this.goalSource = goalSource;
    }

    private boolean use_static_planner_list = false;

    /**
     * Get the value of use_static_planner_list
     *
     * @return the value of use_static_planner_list
     */
    public boolean isUse_static_planner_list() {
        return use_static_planner_list;
    }

    /**
     * Set the value of use_static_planner_list
     *
     * @param use_static_planner_list new value of use_static_planner_list
     */
    public void setUse_static_planner_list(boolean use_static_planner_list) {
        this.use_static_planner_list = use_static_planner_list;
    }

}
