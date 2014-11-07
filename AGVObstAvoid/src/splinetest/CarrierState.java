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

import java.awt.Polygon;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * Class representing the state of a carrier or AGV. It is used for start and
 * goal states as well as live states during simulation.
 *
 * @author Will Shackleford<shackle@nist.gov>
 */
public class CarrierState extends Point2Dd {

    
        private boolean reverse;

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
        this.reverse = reverse;
    }

        private int simCyclePlanFailed = -1;

    /**
     * Get the value of simCyclePlanFailed
     *
     * @return the value of simCyclePlanFailed
     */
    public int getSimCyclePlanFailed() {
        return simCyclePlanFailed;
    }

    /**
     * Set the value of simCyclePlanFailed
     *
     * @param simCyclePlanFailed new value of simCyclePlanFailed
     */
    public void setSimCyclePlanFailed(int simCyclePlanFailed) {
        this.simCyclePlanFailed = simCyclePlanFailed;
    }

    
    public Point2Dd unit() {
        return getAngle().unit();
    }
    
    /**
     * @return the lastGoalType
     */
    public CarrierStateTypeEnum getLastGoalType() {
        return lastGoalType;
    }

    /**
     * @param lastGoalType the lastGoalType to set
     */
    public void setLastGoalType(CarrierStateTypeEnum lastGoalType) {
        this.lastGoalType = lastGoalType;
    }

    /**
     * @return the plannerList
     */
    public List<PlannerPoint> getPlannerList() {
        if(plannerList == null) {
            return null;
        }
        return Collections.unmodifiableList(plannerList);
    }

    /**
     * @param plannerList the plannerList to set
     */
    public void setPlannerList(List<PlannerPoint> plannerList) {
       
        this.plannerList = plannerList;
    }

    private AngleD reverseAngle;
    
    /**
     * @return the angle
     */
    public AngleD getAngle() {
        if(this.reverse) {
            if(null == reverseAngle) {
                reverseAngle = new AngleD(angle.getValue()+Math.PI);
            }
            return reverseAngle;
        }
        return angle;
    }

    /**
     * @param _angle the angle to set
     */
    public void setAngle(AngleD _angle) {
//        if (null != this.angle && null != _angle) {
//            if (Math.abs(AngleD.diff(_angle, this.angle).getValue()) > Math.PI / 36) {
//                System.out.println("_angle = " + _angle);
//                System.out.println("this.angle = " + this.angle);
//                System.out.println("AngleD.diff(_angle, this.angle) = " + AngleD.diff(_angle, this.angle));
//            }
//        }
        this.angle = _angle;
        this.reverseAngle = null;
        this.updateShape();
    }

    /**
     * @return the affineTransform
     */
    public AffineTransform getAffineTransform() {
        return affineTransform;
    }

    /**
     * @param affineTransform the affineTransform to set
     */
    public void setAffineTransform(AffineTransform affineTransform) {
        this.affineTransform = affineTransform;
    }

    /**
     * @return the width
     */
    public double getWidth() {
        return width;
    }

    /**
     * @param width the width to set
     */
    public void setWidth(double width) {
        this.width = width;
        this.updateShape();
    }

    /**
     * @return the back
     */
    public double getBack() {
        if(reverse) {
            return front;
        }
        return back;
    }

    /**
     * @param back the back to set
     */
    public void setBack(double back) {
        this.back = back;
        this.updateShape();
    }

    /**
     * @return the front
     */
    public double getFront() {
        if(reverse) {
            return back;
        }
        return front;
    }

    /**
     * @param front the front to set
     */
    public void setFront(double front) {
        this.front = front;
        this.updateShape();
    }
    private AngleD angle = new AngleD(0);
    private Shape shape;
    private Line2Dd line;
    private double plannerPointDisplaySize = 25.0;
    private AffineTransform affineTransform = null;
    private double width = 0.0;
    private double back = 0.0;
    private double front = 0.0;

        private Shape arrow;

    /**
     * Get the value of arrow
     *
     * @return the value of arrow
     */
    public Shape getArrow() {
        return arrow;
    }

    /**
     * Set the value of arrow
     *
     * @param arrow new value of arrow
     */
    public void setArrow(Shape arrow) {
        this.arrow = arrow;
    }

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

    public CarrierState(double _x, double _y,
            AngleD _angle,
            CarrierStateTypeEnum _type,
            double _plannerPointDisplaySize,
            double _width,
            double _front,
            double _back) {
        super(_x, _y);
        next_id++;
        this.id = next_id;
        this.angle = _angle;
        this.type = _type;
        this.plannerPointDisplaySize = _plannerPointDisplaySize;
        this.width = _width;
        this.front = _front;
        this.back = _back;
        updateShape();
    }

    public CarrierState(Point2D.Double pt,
            AngleD _angle,
            CarrierStateTypeEnum _type,
            double _plannerPointDisplaySize,
            double _width,
            double _front,
            double _back) {
        super(pt.x, pt.y);
        next_id++;
        this.id = next_id;
        this.angle = _angle;
        this.type = _type;
        this.plannerPointDisplaySize = _plannerPointDisplaySize;
        this.width = _width;
        this.front = _front;
        this.back = _back;
        updateShape();
    }

    public CarrierState(CarrierState cs) {
        super(cs.x, cs.y);
        next_id++;
        this.id = next_id;
        this.angle = cs.angle;
        this.shape = cs.shape;
        this.plannerPointDisplaySize = cs.plannerPointDisplaySize;
        this.type = cs.type;
        this.plannerPointDisplaySize = cs.plannerPointDisplaySize;
        this.width = cs.width;
        this.front = cs.front;
        this.back = cs.back;
        updateShape();
    }

    boolean contains(Point2Dd pt) {
        return !checkPtOutsidePoly(getPoly(), pt);
    }
    private List<Point2Dd> poly;

    public List<Point2Dd> getPoly() {
        if (null != poly) {
            return poly;
        }
        this.poly = getPoly(x, y, angle, width, front, back);
        return this.poly;

    }

    public static List<Point2Dd> getPoly(double x, double y, AngleD angle,
            double width, double front,
            double back) {
        double c = angle.cos();
        double s = angle.sin();
        List<Point2Dd> poly = Arrays.asList(
                new Point2Dd(x - back * c - width / 2 * s, y - back * s + width / 2 * c),
                new Point2Dd(x - back * c + width / 2 * s, y - back * s - width / 2 * c),
                new Point2Dd(x + front * c + width / 2 * s, y + front * s - width / 2 * c),
                new Point2Dd(x + front * c - width / 2 * s, y + front * s + width / 2 * c));
        return poly;
    }
    private List<Boundary> immediateBoundaries = null;

    public List<Boundary> getImmediateBoundaries() {
        if (poly != null && immediateBoundaries != null) {
            return immediateBoundaries;
        }
        poly = getPoly();
        this.immediateBoundaries = new LinkedList<>();
        Point2Dd p0 = poly.get(poly.size() - 1);
        for (int i = 0; i < poly.size(); i++) {
            Point2Dd p1 = poly.get(i);
            Boundary b = new Boundary(p0, p1);
            this.immediateBoundaries.add(b);
            p0 = p1;
        }
        return this.immediateBoundaries;
    }

    CarrierState getProjectedState(double time) {
        CarrierState cs_proj = new CarrierState(this);
        double angle_inc = time * rotVel;
        if (Math.abs(angle_inc) <= 2 * java.lang.Double.MIN_NORMAL) {
            cs_proj.x += transVel * angle.cos();
            cs_proj.y += transVel * angle.sin();
            return cs_proj;
        }
        double r = transVel / rotVel;
        if (angle_inc > Math.PI) {
            angle_inc = Math.PI;
        }
        if (angle_inc < -Math.PI) {
            angle_inc = -Math.PI;
        }
        AngleD new_angle = AngleD.add(angle, new AngleD(angle_inc));
        Point2Dd cp;
        Point2Dd new_p;
        if (angle_inc > 0) {
            cp = new Point2Dd(x - angle.sin() * r, y + angle.cos() * r);
            new_p = new Point2Dd(cp.x + new_angle.sin() * r, y - new_angle.cos() * r);
        } else {
            cp = new Point2Dd(x + angle.sin() * r, y - angle.cos() * r);
            new_p = new Point2Dd(cp.x - new_angle.sin() * r, y + new_angle.cos() * r);
        }
        cs_proj.x = new_p.x;
        cs_proj.y = new_p.y;
        cs_proj.angle = new_angle;
        return cs_proj;
    }

    public List<Boundary> getProjectedBoundaries(double time) {
        return getProjectedState(time).getImmediateBoundaries();
    }

    public boolean checkObstacleEx(Obstacle o, double expansion) {
        if (null == poly) {
            poly = getPoly();
        }
        double orig_radius = o.radius;
        o.radius += expansion;
        boolean result = !Planner.checkPoly(poly, o);
        o.radius = orig_radius;
        return result;
    }

    public boolean checkObstacle(Obstacle o) {
        if (null == poly) {
            poly = getPoly();
        }
        boolean result = !Planner.checkPoly(poly, o);
        if (result) {
            this.setColliding(true);
        }
        return result;
    }

    public boolean checkBoundary(Boundary b) {
        if (null == poly) {
            poly = getPoly();
        }
        boolean result = !Planner.checkPoly(poly, b);
        if (result) {
            this.setColliding(true);
        }
        return result;
    }

    public boolean checkBoundaries(List<Boundary> boundaries) {
        if (null == poly) {
            poly = getPoly();
        }
        if (null != boundaries) {
            for (Boundary b : boundaries) {
                boolean result = checkBoundary(b);
                if (result) {
                    this.setColliding(true);
                    return true;
                }
            }
        }
        return false;
    }

    public boolean checkObstacles(List<Obstacle> obstacles) {
        if (null == poly) {
            poly = getPoly();
        }
        if (null != obstacles) {
            for (int i = 0; i < obstacles.size(); i++) {
                Obstacle o = obstacles.get(i);
                if(o.radius < java.lang.Double.MIN_NORMAL) {
                    obstacles.remove(o);
                    continue;
                }
                boolean result = checkObstacle(o);
                if (result) {
                    this.setColliding(true);
                    return true;
                }
            }
        }
        return false;
    }
    private boolean colliding;

    /**
     * Get the value of colliding
     *
     * @return the value of colliding
     */
    public boolean isColliding() {
        return colliding;
    }

    /**
     * Set the value of colliding
     *
     * @param colliding new value of colliding
     */
    public void setColliding(boolean colliding) {
        this.colliding = colliding;
    }

    static private boolean checkPtOutsidePoly(List<Point2Dd> poly, Point2Dd o) {
        Point2Dd last_pt = null;
        boolean last_sign = false;
        boolean last_sign_set = false;
        int sign_change_count = 0;
//        System.out.println("o = " + o);
        for (int i = 0; i < poly.size(); i++) {
            if (last_pt == null) {
                last_pt = poly.get(poly.size() - 1);
            }
            Point2Dd pt = poly.get(i);
//            System.out.println("i = " + i);
//            System.out.println("pt = " + pt);
            Point2Dd diff = pt.diff(last_pt);
            //System.out.println("diff = " + diff);
            if (diff.mag() < 1e-4) {
                continue;
            }
            Point2Dd diffo = o.diff(last_pt);
            Point2Dd diffu = diff.unit();
            Point2Dd diffou = diffo.unit();
            boolean sign = (diffu.x * diffou.y - diffu.y * diffou.x) > 0;
            if (last_sign_set && sign != last_sign) {
                return true;
            }
            last_sign = sign;
            last_sign_set = true;
            last_pt = pt;
        }
        return false;
    }
    private CarrierStateTypeEnum type = CarrierStateTypeEnum.START;

    /**
     * Get the value of type
     *
     * @return the value of type
     */
    public CarrierStateTypeEnum getType() {
        return type;
    }

    /**
     * Set the value of type
     *
     * @param type new value of type
     */
    public void setType(CarrierStateTypeEnum type) {
        this.type = type;
    }

    @Override
    public void setLocation(double arg0, double arg1) {
        super.setLocation(arg0, arg1); //To change body of generated methods, choose Tools | Templates.
        this.updateShape();
    }

    @Override
    public void setLocation(Point2D pd) {
        super.setLocation(pd); //To change body of generated methods, choose Tools | Templates.
        this.updateShape();
    }

    public final void updateShape() {
        double sz = this.plannerPointDisplaySize;
        this.poly = null;
        this.immediateBoundaries = null;
        List<Point2Dd> new_poly = this.getPoly();
        Polygon p = new Polygon();
        for (Point2D p2d : new_poly) {
            p.addPoint((int) p2d.getX(), (int) p2d.getY());
        }
        this.setShape(p);
        this.setAffineTransform(null);
        if (this.type != CarrierStateTypeEnum.LIVE) {
            sz *= 2.0;
            this.line = new Line2Dd(x, y,
                    (x + angle.cos() * sz / 2.0), (y + angle.sin() * sz / 2.0));
            Polygon2Dd arrowPoly = new Polygon2Dd();
            arrowPoly.addPoint(new Point2Dd(x + angle.cos() * sz / 2.0, y + angle.sin() * sz / 2.0));
            arrowPoly.addPoint(new Point2Dd(x + angle.cos() * sz / 2.0 - angle.sin() * sz / 2.0, 
                    y + angle.sin() * sz / 2.0 + angle.cos() * sz / 2.0));
            arrowPoly.addPoint(new Point2Dd(x + angle.cos() * sz, y + angle.sin() * sz));
            arrowPoly.addPoint(new Point2Dd(x + angle.cos() * sz / 2.0 + angle.sin() * sz / 2.0, 
                    y + angle.sin() * sz / 2.0 - angle.cos() * sz / 2.0));
            arrowPoly.closePath();
            this.arrow = arrowPoly;
            //arrowPoly.addPoint((int)(x + angle.cos() * sz / 2.0), (int)(y + angle.sin() * sz / 2.0));
            
        } else {
            this.line = null;
            this.arrow = null;
            
        }
    }
    private static int next_id = 1;
    private int id = -1;

    @Override
    public String toString() {
        return this.getClass().getName()
                + String.format("[%.2f,%.2f,%s,%s,%d,%d,%d]",
                x, y, angle,
                type.toString(),
                id,
                (null != goal) ? goal.getId() : 0,
                (null != start) ? start.getId() : 0);
    }
    static private Map<Integer, CarrierState> idMap = new HashMap<>();

    public static CarrierState valueOf(String s) {
        s = s.trim();
        int left_sq_paren_index = s.indexOf('[');
        if (left_sq_paren_index < 0) {
            return null;
        }
        int right_sq_paren_index = s.indexOf(']', left_sq_paren_index);
        if (right_sq_paren_index < left_sq_paren_index) {
            return null;
        }
        String list_s = s.substring(left_sq_paren_index + 1, right_sq_paren_index).trim();
        String la[] = list_s.split(",");
        CarrierState cs = new CarrierState(
                java.lang.Double.valueOf(la[0]),
                java.lang.Double.valueOf(la[1]),
                AngleD.valueOf(la[2]),
                CarrierStateTypeEnum.START,
                SplinePanel.DEFAULT_PLANNER_POINT_DISPLAY_SIZE,
                SplinePanel.DEFAULT_VEHICLE_WIDTH,
                SplinePanel.DEFAULT_VEHICLE_FRONT,
                SplinePanel.DEFAULT_VEHICLE_BACK);
        if (la.length > 3) {
            cs.setType(CarrierStateTypeEnum.valueOf(la[3]));
        }
        if (cs.type == CarrierStateTypeEnum.START
                || cs.type == CarrierStateTypeEnum.WAYPOINT) {
            if (la.length > 4) {
                cs.setId(Integer.valueOf(la[4]));
                if (cs.id >= next_id) {
                    next_id = cs.id + 1;
                }
                idMap.put(cs.id, cs);
            }
            if (la.length > 5) {
                int goal_id = Integer.valueOf(la[5]);
                if (goal_id > 0) {
                    CarrierState goal = idMap.get(goal_id);
                    if (goal != null) {
                        cs.setGoal(goal);
                    }
                }
                if(next_id <= goal_id) {
                    next_id = goal_id+1;
                }
            }
        } 
        if (cs.type == CarrierStateTypeEnum.GOAL
                || cs.type == CarrierStateTypeEnum.WAYPOINT) {
            if (la.length > 4) {
                cs.setId(Integer.valueOf(la[4]));
                if (cs.id >= next_id) {
                    next_id = cs.id + 1;
                }
                idMap.put(cs.id, cs);
            }
            if (la.length > 6) {
                int start_id = Integer.valueOf(la[6]);
                if (start_id > 0) {
                    CarrierState start = idMap.get(start_id);
                    if (start != null) {
                        cs.setStart(start);
                    }
                }
                if(next_id <= start_id) {
                    next_id = start_id+1;
                }
            }
        }
        return cs;
    }

    /**
     * @return the shape
     */
    public Shape getShape() {
        return shape;
    }

    /**
     * @param shape the shape to set
     */
    public void setShape(Shape shape) {
        this.shape = shape;
    }

    /**
     * @return the line
     */
    public Line2Dd getLine() {
        return line;
    }

    /**
     * @param line the line to set
     */
    public void setLine(Line2Dd line) {
        this.line = line;
    }
    private CarrierState start;
    private CarrierState goal;

    /**
     * Get the value of goal
     *
     * @return the value of goal
     */
    public CarrierState getGoal() {
        return goal;
    }
    private CarrierStateTypeEnum lastGoalType = CarrierStateTypeEnum.START;

    /**
     * Set the value of goal
     *
     * @param goal new value of goal
     */
    public void setGoal(CarrierState _goal) {
        CarrierState oldGoal = this.goal;
        this.goal = null;
        if(null != oldGoal && this == oldGoal.getStart()) {
            oldGoal.setStart(null);
        }
        this.goal = _goal;
        if ((this.type == CarrierStateTypeEnum.START ||
                this.type == CarrierStateTypeEnum.WAYPOINT)
                && _goal != null
                && _goal.getStart() == null) {
            _goal.setStart(this);
        }
        if (null != goal) {
            this.setLastGoalType(goal.type);
        } else {
            this.setRotVel(0);
            this.setTransVel(0);
        }
    }
    private PlannedPath path;

    /**
     * Get the value of path
     *
     * @return the value of path
     */
    public PlannedPath getPath() {
        return path;
    }

    /**
     * Set the value of path
     *
     * @param path new value of path
     */
    public void setPath(PlannedPath path) {
        this.path = path;
    }
    private List<Boundary> boundaries = null;

    /**
     * @return the boundaries
     */
    public List<Boundary> getBoundaries() {
        return boundaries;
    }

    /**
     * @param boundaries the boundaries to set
     */
    public void setBoundaries(List<Boundary> boundaries) {
        this.boundaries = boundaries;
    }
    private List<PlannerPoint> plannerList;
    private int currentCurvePointsIndex = 0;

    /**
     * Get the value of currentCurvePointsIndex
     *
     * @return the value of currentCurvePointsIndex
     */
    public int getCurrentCurvePointsIndex() {
        return currentCurvePointsIndex;
    }

    /**
     * Set the value of currentCurvePointsIndex
     *
     * @param currentCurvePointsIndex new value of currentCurvePointsIndex
     */
    public void setCurrentCurvePointsIndex(int currentCurvePointsIndex) {
        this.currentCurvePointsIndex = currentCurvePointsIndex;
    }
    private double transVel;

    /**
     * Get the value of transVel
     *
     * @return the value of transVel
     */
    public double getTransVel() {
        return transVel;
    }

    /**
     * Set the value of transVel
     *
     * @param transVel new value of transVel
     */
    public void setTransVel(double transVel) {
        this.transVel = transVel;
    }
    private double rotVel;

    /**
     * Get the value of rotVel
     *
     * @return the value of rotVel
     */
    public double getRotVel() {
        return rotVel;
    }

    /**
     * Set the value of rotVel
     *
     * @param rotVel new value of rotVel
     */
    public void setRotVel(double rotVel) {
        this.rotVel = rotVel;
    }

    /**
     * @return the start
     */
    public CarrierState getStart() {
        return start;
    }

    /**
     * @param _start the start to set
     */
    public void setStart(CarrierState _start) {
        CarrierState oldStart = this.start;
        this.start = null;
        if(null != oldStart && this == oldStart.getGoal()) {
            oldStart.setGoal(null);
        }
        this.start = _start;
        if ((this.type == CarrierStateTypeEnum.GOAL ||
                this.type == CarrierStateTypeEnum.WAYPOINT)
                && _start != null
                && _start.getGoal() == null) {
            _start.setGoal(this);
        }
    }

    /**
     * @return the id
     */
    public int getId() {
        return id;
    }

    /**
     * @param id the id to set
     */
    public void setId(int id) {
        this.id = id;
    }
}
