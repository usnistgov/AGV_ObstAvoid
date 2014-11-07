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

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

/**
 * Class provides a way of creating planned paths though a series of control
 * points that reach a given goalPoint from a given startPoint while avoiding
 * obstacles and barriers with a vehicle of finite width front offset and back
 * offset.
 *
 * @author Will Shackleford<shackle@nist.gov>
 */
public class Planner {

    //public static PlannerPoint startPP;
    //public static List<PlannerPoint> ll;
    public static List<PlannerPoint> goalPPList;
    public static List<PlannerPoint> startPPList;

    public static void addPointsAtOffset(List<PlannerPoint> ll, double offset1, double offset2, Obstacle o) {
        ll.add(new PlannerPoint(o.x + offset1, o.y + offset2));
        ll.add(new PlannerPoint(o.x + offset1, o.y - offset2));
        ll.add(new PlannerPoint(o.x - offset1, o.y + offset2));
        ll.add(new PlannerPoint(o.x - offset1, o.y - offset2));
        ll.add(new PlannerPoint(o.x + offset2, o.y + offset1));
        ll.add(new PlannerPoint(o.x + offset2, o.y - offset1));
        ll.add(new PlannerPoint(o.x - offset2, o.y + offset1));
        ll.add(new PlannerPoint(o.x - offset2, o.y - offset1));
    }

    public static List<PlannerPoint> surroundPoint(PlannerInput pi, Point2Dd pt, List<PlannerPoint> ll) {
        if (null == pt) {
            return ll;
        }
        for (double angle = 0; angle < 2 * Math.PI; angle += Math.PI / 12.0) {
            double cos_angle = Math.cos(angle);
            double sin_angle = Math.sin(angle);
            for (double l = pi.min_turn_radius / 2.0; l < 7.0 * pi.min_turn_radius; l += pi.min_turn_radius / 2.0) {
                ll.add(new PlannerPoint((double) (pt.x + l * cos_angle),
                        (double) (pt.y + l * sin_angle)));
            }
        }
        return ll;
    }

    public static List<PlannerPoint> staticPlannerList = null;

    public static void resetPlannerList(List<PlannerPoint> ll) {
        for (int i = 0; i < ll.size(); i++) {
            PlannerPoint pp = ll.get(i);
            pp.checked = false;
            pp.distFromGoal = Double.POSITIVE_INFINITY;
            pp.failed_neighbors = null;
            pp.potential_neighbors = null;
            pp.opened = false;
        }

    }

    public static List<PlannerPoint> createStaticPlannerList(PlannerInput pi,
            List<? extends CarrierState> startList,
            List<? extends CarrierState> goalList,
            List<? extends CarrierState> waypointsList) {

        if (pi.use_static_planner_list && staticPlannerList != null) {
            resetPlannerList(staticPlannerList);
            return staticPlannerList;
        }
        pi.create_planner_list_start_cpu_ns = Planner.getCpuTimeNs();
        pi.create_planner_list_start_world_ms = System.currentTimeMillis();
        double veh_width = pi.veh_width + pi.path_uncertainty;

        List<PlannerPoint> ll = new LinkedList<PlannerPoint>();
        for (double x = pi.rectB.x; x < pi.rectB.x + pi.rectB.width; x += pi.plannerResolution) {
            for (double y = pi.rectB.y; y < pi.rectB.y + pi.rectB.height; y += pi.plannerResolution) {
                ll.add(new PlannerPoint(x, y));
            }
        }
        List<Boundary> boundaries = pi.boundaries;
        for (int i = 0; i < ll.size(); i++) {
            PlannerPoint pp = ll.get(i);
            boolean pp_removed = false;
            if (pp_removed) {
                continue;
            }
            if (null != boundaries) {
                for (int k = 0; k < boundaries.size(); k++) {
                    Boundary b = boundaries.get(k);
                    if (b.ptSegDist(pp) < veh_width / 2f + pi.plannerResolution) {
                        ll.remove(i);
                        i--;
                        break;
                    }
                }
            }
        }
        staticPlannerList = ll;
        pi.create_planner_list_end_cpu_ns = Planner.getCpuTimeNs();
        pi.create_planner_list_end_world_ms = System.currentTimeMillis();
        return staticPlannerList;
    }

    private static void adjustRectB(PlannerInput pi, Point2Dd pt) {
        if (pi.rectB != null) {
            if (pi.rectB.x > pt.x - pi.min_turn_radius * 2) {
                double dx = pi.rectB.x - pt.x - pi.min_turn_radius * 2;
                pi.rectB.width += dx;
                pi.rectB.x -= dx;
            }
            if (pi.rectB.x + pi.rectB.width < pt.x + pi.min_turn_radius * 2) {
                double dx = pt.x + pi.min_turn_radius * 2 - (pi.rectB.x + pi.rectB.width);
                pi.rectB.width += dx;
            }
            if (pi.rectB.y > pt.y - pi.min_turn_radius * 2) {
                double dx = pi.rectB.y - pt.y - pi.min_turn_radius * 2;
                pi.rectB.height += dx;
                pi.rectB.y -= dx;
            }
            if (pi.rectB.y + pi.rectB.height < pt.y + pi.min_turn_radius * 2) {
                double dx = pt.y + pi.min_turn_radius * 2 - (pi.rectB.y + pi.rectB.height);
                pi.rectB.height += dx;
            }
        }
    }

    public static List<PlannerPoint> createPlannerList(PlannerInput pi,
            List<? extends CarrierState> startList,
            List<? extends CarrierState> goalList,
            List<? extends CarrierState> waypointsList) {
        if (pi.use_static_planner_list) {
            return createStaticPlannerList(pi, startList, goalList, waypointsList);
        }
        pi.create_planner_list_start_cpu_ns = Planner.getCpuTimeNs();
        pi.create_planner_list_start_world_ms = System.currentTimeMillis();

        goalPPList = new LinkedList<>();
        startPPList = new LinkedList<>();
        List<Obstacle> obstacles = pi.obstacles;
        List<Boundary> boundaries = pi.boundaries;
        double front = pi.front + pi.path_uncertainty;
        double back = pi.back + pi.path_uncertainty;
        double veh_width = pi.veh_width + pi.path_uncertainty;

//        boolean reverse = pi.reverse;
        double max_pt2pt_dist = pi.max_pt2pt_dist;
        List<PlannerPoint> ll = new LinkedList<PlannerPoint>();
        double max_frontback = Math.max(front, back);

        if (null != obstacles) {
            try {
                for (int i = 0; i < obstacles.size(); i++) {
                    if (Thread.currentThread().isInterrupted()) {
                        return null;
                    }
                    Obstacle o = obstacles.get(i);
                    if (null != pi.rectB && !pi.rectB.contains(o)) {
                        continue;
                    }
                    double offset1 = (o.radius + veh_width / 2f + 0.0001f) * 1.0001f;
                    ll.add(new PlannerPoint(o.x + offset1, o.y));
                    ll.add(new PlannerPoint(o.x - offset1, o.y));
                    ll.add(new PlannerPoint(o.x, o.y + offset1));
                    ll.add(new PlannerPoint(o.x, o.y - offset1));
                    ll.add(new PlannerPoint(o.x - offset1, o.y + offset1));
                    ll.add(new PlannerPoint(o.x - offset1, o.y - offset1));
                    ll.add(new PlannerPoint(o.x + offset1, o.y + offset1));
                    ll.add(new PlannerPoint(o.x + offset1, o.y - offset1));
                    addPointsAtOffset(ll, offset1, offset1, o);
                    double offset2 = (o.radius + front + 0.0001f) * 1.0001f;
                    if (Math.abs(offset1 - offset2) / (offset1 + offset2) > 0.01) {
                        addPointsAtOffset(ll, offset1, offset2, o);
                    }
                    double offset3 = (o.radius + back + 0.0001f) * 1.0001f;
                    if (Math.abs(offset1 - offset3) / (offset1 + offset3) > 0.01
                            && Math.abs(offset2 - offset3) / (offset2 + offset3) > 0.01) {
                        addPointsAtOffset(ll, offset1, offset3, o);
                    }
                    double offset4 = (o.radius + Math.sqrt(veh_width * veh_width / 4.0 + max_frontback * max_frontback) + 0.0001f) * 1.0001f;
                    if (Math.abs(offset1 - offset4) / (offset1 + offset4) > 0.01
                            && Math.abs(offset2 - offset4) / (offset2 + offset4) > 0.01
                            && Math.abs(offset3 - offset4) / (offset3 + offset4) > 0.01) {
                        addPointsAtOffset(ll, offset1, offset4, o);
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        if (null != boundaries) {
            for (int i = 0; i < boundaries.size(); i++) {
                if (Thread.currentThread().isInterrupted()) {
                    return null;
                }
                Boundary b = boundaries.get(i);
                if (Double.isNaN(b.x2) || Double.isNaN(b.y2)) {
                    continue;
                }
                Point2Dd p1 = b.getP1();
                Point2Dd p2 = b.getP2();
                Point2Dd bdiff = p2.diff(p1);
                if (bdiff.mag() < Double.MIN_NORMAL) {
                    continue;
                }
                Point2Dd bdiffu = bdiff.unit();
                Point2Dd bright = new Point2Dd(-bdiffu.y, bdiffu.x);
                double extra = 0.05 * Math.max(veh_width / 2.0, Math.max(front, back));
//                Point2Df bleft = new Point2Df(bdiffu.y, -bdiffu.x);
                double advals[] = {-front - extra, -back - extra, -veh_width / 2.0 - extra, bdiff.mag() + veh_width / 2.0 + extra, bdiff.mag() + front + extra, bdiff.mag() + back + extra};
                double bdvals[] = {-front - extra, -back - extra, -veh_width / 2.0 - extra, veh_width / 2.0 + extra, +front + extra, back + extra};
                for (double ad : advals) {
                    for (double bd : bdvals) {
                        PlannerPoint bpp = new PlannerPoint(p1.x + ad * bdiffu.x + bright.x * bd,
                                p1.y + ad * bdiffu.y + bright.y * bd);
                        ll.add(bpp);
                    }
                }
//                for (int j = 0; j < boundaries.size(); j++) {
//                    if(i == j) {
//                        continue;
//                    }
//                    Boundary bj = boundaries.get(j);
//                    if (Double.isNaN(bj.x2) || Double.isNaN(bj.y2)) {
//                        continue;
//                    }
//                    Point2Df pj1 = bj.getP1();
//                    Point2Df pj2 = bj.getP2();
//                    ll.add(new PlannerPoint((p1.x+pj1.x+pj2.x)/3f,(p1.y+pj1.y+pj2.y)/3f));
//                    ll.add(new PlannerPoint((p2.x+pj1.x+pj2.x)/3f,(p2.y+pj1.y+pj2.y)/3f));
//                }
            }
        }

        if (pi.min_turn_radius > 0 && !pi.crab) {
            if (null != pi.start) {
                if (!pi.crab) {
                    pi.start.setReverse(pi.reverse);
                }
                Point2Dd pt = new Point2Dd(pi.start.x + pi.segStartLength * pi.start.getAngle().cos(),
                        pi.start.y + pi.segStartLength * pi.start.getAngle().sin());
                adjustRectB(pi,pt);
                ll = surroundPoint(pi, pt, ll);
//                for (double angle = 0; angle < 2 * Math.PI; angle += Math.PI / 12.0) {
//                    double cos_angle = Math.cos(angle);
//                    double sin_angle = Math.sin(angle);
//                    for (double l = pi.min_turn_radius / 2.0; l < 7.0 * pi.min_turn_radius; l += pi.min_turn_radius / 2.0) {
//                        ll.add(new PlannerPoint((double) (pi.start.x + pi.segStartLength * pi.goal.getAngle().cos() + l * cos_angle),
//                                (double) (pi.goal.y + pi.segStartLength * pi.goal.getAngle().cos() + l * sin_angle)));
//                    }
//                }
            }
            if (null != pi.goal) {
                if (!pi.crab) {
                    pi.goal.setReverse(pi.reverse);
                }
                Point2Dd pt = new Point2Dd(pi.goal.x - pi.segStartLength * pi.goal.getAngle().cos(),
                        pi.goal.y - pi.segStartLength * pi.goal.getAngle().sin());
                ll = surroundPoint(pi, pt, ll);
                adjustRectB(pi,pt);

//                for (double angle = 0; angle < 2 * Math.PI; angle += Math.PI / 12.0) {
//                    double cos_angle = Math.cos(angle);
//                    double sin_angle = Math.sin(angle);
//                    for (double l = pi.min_turn_radius / 2.0; l < 7.0 * pi.min_turn_radius; l += pi.min_turn_radius / 2.0) {
//                        ll.add(new PlannerPoint((double) (pi.goal.x - pi.segStartLength * pi.goal.getAngle().cos() + l * cos_angle),
//                                (double) (pi.goal.y - pi.segStartLength * pi.goal.getAngle().sin() + l * sin_angle)));
//                    }
//                }
            }
            if (null != startList) {
                for (CarrierState cs : startList) {
                    Point2Dd pt = new Point2Dd(cs.x + pi.segStartLength * cs.getAngle().cos(),
                            cs.y + pi.segStartLength * cs.getAngle().sin());
                    ll = surroundPoint(pi, pt, ll);
                    adjustRectB(pi,pt);
                
//                    for (double angle = 0; angle < 2 * Math.PI; angle += Math.PI / 12.0) {
//                        double cos_angle = Math.cos(angle);
//                        double sin_angle = Math.sin(angle);
//                        for (double l = pi.min_turn_radius / 2.0; l < 7.0 * pi.min_turn_radius; l += pi.min_turn_radius / 2.0) {
//                            ll.add(new PlannerPoint((double) (pt.x + l * cos_angle),
//                                    (double) (pt.y + l * sin_angle)));
//                        }
//                    }
                }
            }
            if (null != goalList) {
                for (CarrierState cs : goalList) {
                    Point2Dd pt = new Point2Dd(cs.x - pi.segStartLength * cs.getAngle().cos(),
                            cs.y - pi.segStartLength * cs.getAngle().sin());
                    ll = surroundPoint(pi, pt, ll);
                    adjustRectB(pi,pt);
                
//                    for (double angle = 0; angle < 2 * Math.PI; angle += Math.PI / 12.0) {
//                        double cos_angle = Math.cos(angle);
//                        double sin_angle = Math.sin(angle);
//                        for (double l = pi.min_turn_radius / 2.0; l < 7.0 * pi.min_turn_radius; l += pi.min_turn_radius / 2.0) {
//                            ll.add(new PlannerPoint((double) (pt.x + l * cos_angle),
//                                    (double) (pt.y + l * sin_angle)));
//                        }
//                    }
                }
            }
            if (null != waypointsList) {
                for (CarrierState cs : waypointsList) {
                    Point2Dd pt = new Point2Dd(cs.x - pi.segStartLength * cs.getAngle().cos(),
                            cs.y - pi.segStartLength * cs.getAngle().sin());
                    adjustRectB(pi,pt);
                    ll = surroundPoint(pi, pt, ll);
                    Point2Dd pt2 = new Point2Dd(cs.x + pi.segStartLength * cs.getAngle().cos(),
                            cs.y + pi.segStartLength * cs.getAngle().sin());
                    ll = surroundPoint(pi, pt2, ll);
                    ll = surroundPoint(pi, cs, ll);
//                    for (double angle = 0; angle < 2 * Math.PI; angle += Math.PI / 12.0) {
//                        double cos_angle = Math.cos(angle);
//                        double sin_angle = Math.sin(angle);
//                        for (double l = pi.min_turn_radius / 2.0; l < 7.0 * pi.min_turn_radius; l += pi.min_turn_radius / 2.0) {
//                            ll.add(new PlannerPoint((double) (pt.x + l * cos_angle),
//                                    (double) (pt.y + l * sin_angle)));
//                        }
//                    }
                }
            }
        }
        ll = surroundPoint(pi, pi.start, ll);
        ll = surroundPoint(pi, pi.goal, ll);

//        System.out.println("ll.size() = "+ll.size());
        for (int i = 0; i < ll.size(); i++) {
            PlannerPoint ppi = ll.get(i);
            if (null != pi.rectB && !pi.rectB.contains(ppi)) {
                ll.remove(i);
                i--;
                continue;
            }
            boolean pp_removed = false;
            if (null != obstacles) {
                for (int j = 0; j < obstacles.size(); j++) {
                    Obstacle o = obstacles.get(j);
                    if (ppi.distance(o) < o.radius + veh_width / 2f) {
                        ll.remove(i);
                        i--;
                        pp_removed = true;
                        break;
                    }
                }
            }
            if (pp_removed) {
                continue;
            }
            if (null != boundaries) {
                for (int k = 0; k < boundaries.size(); k++) {
                    Boundary b = boundaries.get(k);
                    if (b.ptSegDist(ppi) < veh_width / 2f) {
                        ll.remove(i);
                        i--;
                        pp_removed = true;
                        break;
                    }
                }
            }
            if (pp_removed) {
                continue;
            }
        }
        for (int i = 0; i < ll.size(); i++) {
            PlannerPoint ppi = ll.get(i);
            for (int j = i + 1; j < ll.size(); j++) {
                PlannerPoint ppj = ll.get(j);
                if (ppj.distance(ppi) < pi.plannerResolution) {
                    ll.remove(j);
                    j--;
                }
            }
        }

        pi.create_planner_list_end_cpu_ns = Planner.getCpuTimeNs();
        pi.create_planner_list_end_world_ms = System.currentTimeMillis();

//        System.out.println("ll.size() = "+ll.size());
        return ll;
    }

//    public static List<PlannerPoint> plan(PlannerInput pi) throws Exception {
//        points_checked = 0;
//        List<PlannerPoint> ll = Planner.createPlannerList(pi, null, null,null);
//        return Planner.planWithPlannerList(pi, ll);
//    }
    public static int countSelected(List<PlannerPoint> pplist) {
        int c = 0;
        if (pplist != null) {
            for (PlannerPoint pp : pplist) {
                if (pp.selected) {
                    c++;
                }
            }
        }
        return c;
    }

    public static double computePathDist(final PlannerPoint pp) {
        double dist = 0;
        PlannerPoint ppi = pp;
        while (ppi.prev_pt != null) {
            dist += ppi.distance(ppi.prev_pt);
            ppi = ppi.prev_pt;
        }
        return dist;
    }

    public static double chord(Point2Dd uvec1, Point2Dd uvec2, double turn_radius) {
        double dot = uvec1.unit().dot(uvec2.unit());
        return turn_radius * Math.sqrt(2 * (1 - dot));
    }

    static public boolean checkTurn(Point2Dd uvec1, Point2Dd uvec2, PlannerInput pi, double distance) {
        double dot = uvec1.unit().dot(uvec2.unit());
        double chord = pi.min_turn_radius * Math.sqrt(2 * (1 - dot));
        if (dot < pi.min_turn_radius_dot_limit) {
            return false;
        }
        return distance > chord;
    }
    static public PlannerStatusSetter statusSetter = null;

    static public PlannerInput last_succeed_pi = null;
    static public List<PlannerPoint> last_succeed_pplist = null;
    static public PlannerInput last_fail_pi = null;
    static public List<PlannerPoint> last_fail_pplist = null;
    static public boolean record_stats = false;
    static public File recordStatsFile = null;
    static public boolean cpu_time_not_supported = false;
    static public boolean cpu_time_supported_checked = false;

    static public long getCpuTimeNs() {
        if (cpu_time_not_supported) {
            return -1;
        }
        ThreadMXBean tmxbean = null;
        try {
            tmxbean = ManagementFactory.getThreadMXBean();
            if (null != tmxbean) {
                if (!cpu_time_supported_checked) {
                    cpu_time_not_supported = !tmxbean.isCurrentThreadCpuTimeSupported();
                } else {
                    cpu_time_supported_checked = true;
                }
                if (cpu_time_not_supported) {
                    return -1;
                }
                return tmxbean.getCurrentThreadCpuTime();
            }
        } catch (Exception e) {
            // ignore any exceptions
        }
        return -1;
    }

    public static double listLength(List<PlannerPoint> pl) {
        if (null == pl || pl.size() < 2) {
            return 0;
        }
        double d = 0;
        PlannerPoint lp = pl.get(0);
        for (int i = 0; i < pl.size(); i++) {
            PlannerPoint pt = pl.get(i);
            d += pt.distance(lp);
            lp = pt;
        }
        return d;
    }

    public static void closeRecordStatsFile() {
        if (Planner.recordStatsFile != null) {
            Planner.recordStatsFile = null;
        }
    }

    public static void recordStats(PlannerInput pi,
            List<PlannerPoint> pplist,
            List<PlannerPoint> output_list,
            long wall_time_start_ms,
            long wall_time_end_ms,
            long cpu_time_start_ns,
            long cpu_time_end_ns
    ) {
        if (!record_stats) {
            return;
        }
        PrintStream ps = null;
        try {
            boolean new_file = false;
            if (null == recordStatsFile) {
                new_file = true;
                recordStatsFile = File.createTempFile("planner_stats_", ".csv");
                System.out.println("Recording planner stats to " + recordStatsFile.getCanonicalPath());
            }

            ps = new PrintStream(new FileOutputStream(recordStatsFile, true));
            if (new_file) {
                ps.println("wall_time_start_ms,create_planner_list_start_world_ms,wall_time_diff_s,cpu_time_diff_s,create_list_wall_time_diff_s,create_list_cpu_time_diff_s,total_wall_time_diff_s, total_cpu_time_diff_s,pplist_size,path_found,output_list_size,output_list_length,start_x,start_y,start_angle,goal_x,goal_y,goal_angle");
            }
            double wall_time_diff_s = (double) (wall_time_end_ms - wall_time_start_ms) * 1e-3;
            double cpu_time_diff_s = (double) (cpu_time_end_ns - cpu_time_start_ns) * 1e-9;
            double create_list_wall_time_diff_s = (double) (pi.create_planner_list_end_world_ms - pi.create_planner_list_start_world_ms) * 1e-3;
            double create_list_cpu_time_diff_s = (double) (double) (pi.create_planner_list_end_cpu_ns - pi.create_planner_list_start_cpu_ns) * 1e-9;
            double total_wall_time_diff_s = wall_time_diff_s + create_list_wall_time_diff_s;
            double total_cpu_time_diff_s = cpu_time_diff_s + create_list_cpu_time_diff_s;
            ps.println(wall_time_start_ms + "," + pi.create_planner_list_start_world_ms + ","
                    + wall_time_diff_s + ","
                    + cpu_time_diff_s + ","
                    + create_list_wall_time_diff_s + ","
                    + create_list_cpu_time_diff_s + ","
                    + total_wall_time_diff_s + ","
                    + total_cpu_time_diff_s + ","
                    + ((pplist != null) ? pplist.size() : 0) + ","
                    + ((output_list != null && output_list.size() > 2) ? 1 : 0) + ","
                    + ((output_list != null) ? output_list.size() : 0) + ","
                    + listLength(output_list) + ","
                    + pi.start.x + "," + pi.start.y + "," + pi.start.getAngle().toDegrees() + ","
                    + pi.goal.x + "," + pi.goal.y + "," + pi.goal.getAngle().toDegrees()
            );
        } catch (Exception exception) {
            exception.printStackTrace();
        }
        try {
            if (null != ps) {
                ps.close();
                ps = null;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public synchronized static List<PlannerPoint> planWithPlannerList(PlannerInput pi, List<PlannerPoint> pplist) {
        points_checked = 0;
        long wall_start_time_ms = System.currentTimeMillis();
        long cpu_start_time_ns = getCpuTimeNs();
        CarrierState start = pi.start;
        CarrierState goal = pi.goal;
        List<Obstacle> obstacles = pi.obstacles;
        double veh_width = pi.veh_width + pi.path_uncertainty;
        double front = pi.front + pi.path_uncertainty;
        boolean crab = pi.crab;
        boolean reverse = pi.reverse;
        pi.min_turn_radius_dot_limit = Math.cos(Math.toRadians(pi.max_turn_angle_degrees));
        AngleD orig_start_angle = start.getAngle();
        AngleD orig_goal_angle = goal.getAngle();
        if (reverse && !crab) {
            front = pi.back + pi.path_uncertainty;
        }
        double max_pt2pt_dist = pi.max_pt2pt_dist;
        if (!pi.crab) {
            goal.setReverse(pi.reverse);
            start.setReverse(pi.reverse);
        }
        PlannerPoint trueGoalPP = new PlannerPoint(goal);
        PlannerPoint trueStartPP = new PlannerPoint(start);
        PlannerPoint goalPP = trueGoalPP;
        PlannerPoint startPP = trueStartPP;
        if (pi.segStartLength > 0) {
            goalPP = new PlannerPoint(goal.x - pi.segStartLength * goal.getAngle().cos(),
                    goal.y - pi.segStartLength * goal.getAngle().sin());
            startPP = new PlannerPoint(start.x + pi.segStartLength * start.getAngle().cos(),
                    start.y + pi.segStartLength * start.getAngle().sin());
            startPP.prev_pt = trueStartPP;
            trueStartPP.next_pt = startPP;
            goalPP.next_pt = trueGoalPP;
            trueGoalPP.prev_pt = goalPP;
        }

        LinkedList<PlannerPoint> openlist = new LinkedList<PlannerPoint>();
        startPP.pathEstimate = startPP.distFromGoal = (double) startPP.distance(goal);
        startPP.pathFromStartDist = 0f;
        openlist.add(startPP);
        LinkedList<PlannerPoint> ll2 = new LinkedList<PlannerPoint>(pplist);
        ll2.add(new PlannerPoint((double) (start.x + veh_width / 4.0 * start.getAngle().cos()),
                (double) (start.y + veh_width / 4.0 * start.getAngle().sin())));
        ll2.add(new PlannerPoint((double) (start.x + front * start.getAngle().cos()),
                (double) (start.y + front * start.getAngle().sin())));
        ll2.add(goalPP);

//        ll2.add(new PlannerPoint(goal.x+veh_width/2f,goal.y));
//        ll2.add(new PlannerPoint(goal.x+veh_width/2f,goal.y+veh_width/2f));
//        ll2.add(new PlannerPoint(goal.x,goal.y+veh_width/2f));
//        ll2.add(new PlannerPoint(goal.x-veh_width/2f,goal.y));
//        ll2.add(new PlannerPoint(goal.x-veh_width/2f,goal.y-veh_width/2f));
//        ll2.add(new PlannerPoint(goal.x,goal.y-veh_width/2f));
//        ll2.add(new PlannerPoint(goal.x+veh_width/2f,goal.y-veh_width/2f));
//        ll2.add(new PlannerPoint(goal.x-veh_width/2f,goal.y+veh_width/2f));
//        ll2.add(new PlannerPoint(start.x+veh_width/2f,start.y));
//        ll2.add(new PlannerPoint(start.x+veh_width/2f,start.y+veh_width/2f));
//        ll2.add(new PlannerPoint(start.x,start.y+veh_width/2f));
//        ll2.add(new PlannerPoint(start.x-veh_width/2f,start.y));
//        ll2.add(new PlannerPoint(start.x-veh_width/2f,start.y-veh_width/2f));
//        ll2.add(new PlannerPoint(start.x,start.y-veh_width/2f));
//        ll2.add(new PlannerPoint(start.x+veh_width/2f,start.y-veh_width/2f));
//        ll2.add(new PlannerPoint(start.x-veh_width/2f,start.y+veh_width/2f));
//        ll2.add(new PlannerPoint(goal.x+frontback_max,goal.y));
//        ll2.add(new PlannerPoint(goal.x+frontback_max,goal.y+frontback_max));
//        ll2.add(new PlannerPoint(goal.x,goal.y+frontback_max));
//        ll2.add(new PlannerPoint(goal.x-frontback_max,goal.y));
//        ll2.add(new PlannerPoint(goal.x-frontback_max,goal.y-frontback_max));
//        ll2.add(new PlannerPoint(goal.x,goal.y-frontback_max));
//        ll2.add(new PlannerPoint(goal.x+frontback_max,goal.y-frontback_max));
//        ll2.add(new PlannerPoint(goal.x-frontback_max,goal.y+frontback_max));
//        ll2.add(new PlannerPoint(start.x+frontback_max,start.y));
//        ll2.add(new PlannerPoint(start.x+frontback_max,start.y+frontback_max));
//        ll2.add(new PlannerPoint(start.x,start.y+frontback_max));
//        ll2.add(new PlannerPoint(start.x-frontback_max,start.y));
//        ll2.add(new PlannerPoint(start.x-frontback_max,start.y-frontback_max));
//        ll2.add(new PlannerPoint(start.x,start.y-frontback_max));
//        ll2.add(new PlannerPoint(start.x+frontback_max,start.y-frontback_max));
//        ll2.add(new PlannerPoint(start.x-frontback_max,start.y+frontback_max));
        boolean goal_found = false;
        //System.out.println("countSelected(ll2) = " + countSelected(ll2));

        for (PlannerPoint pp : ll2) {
            if (Thread.currentThread().isInterrupted()) {
                if (null != statusSetter) {
                    statusSetter.setPlannerStatus("Planner Status: Interrupted");
                }
                return null;
            }
            pp.distFromGoal = (double) pp.distance(goalPP);
            pp.pathFromStartDist = Double.POSITIVE_INFINITY;
            pp.pathEstimate = Double.POSITIVE_INFINITY;
            if (!pp.equals(goalPP)) {
                pp.distFromGoal += 0.05f;
            }
            pp.failed_neighbors = null;
            pp.potential_neighbors = null;
            pp.neighbors = null;
            pp.opened = false;
            pp.testedInLinesMap = null;
            pp.testedOutLinesMap = null;
        }

        for (int i = 0; i < ll2.size(); i++) {
            PlannerPoint pp = ll2.get(i);
            if (pp != goalPP && pp.distFromGoal < 0.05) {
                ll2.remove(i);
                i--;
            }
        }
        if (null == goalPPList) {
            goalPPList = new LinkedList<>();
        }
        goalPPList.add(goalPP);
        if (null == startPPList) {
            startPPList = new LinkedList<>();
        }
        startPPList.add(startPP);
        startPP.selected = pi.start.selected;
        goalPP.selected = pi.goal.selected;
        Point2Dd next_goal_diffu = new Point2Dd(goal.getAngle().cos(), goal.getAngle().sin());
        while (openlist.size() > 0 && !goal_found) {
            if (Thread.currentThread().isInterrupted()) {
                if (null != statusSetter) {
                    statusSetter.setPlannerStatus("Planner Status: Interrupted");
                }
                return null;
            }
            final PlannerPoint pp = getBestFromOpenList(openlist);
            if (pp == goalPP) {
                goal_found = true;
                break;
            }
//            if (pp.selected) {
//                System.out.println("pp = " + pp);
//            }
            if (!pp.opened) {
                ll2.removeAll(openlist);
                if (null == pp.potential_neighbors) {
                    LinkedList<PlannerPoint> pppnl = new LinkedList<PlannerPoint>();
                    //HashMap<PlannerPoint, Double> d1chkl = new HashMap<PlannerPoint, Double>();
                    Point2Dd goal_pp_diff = null;
                    Point2Dd goal_pp_diffu = null;
                    double goal_pp_diff_mag = 0.0;
                    Point2Dd pp_prev_diff = null;
                    Point2Dd pp_prev_diffu = null;
                    double pp_prev_diff_mag = 0.0;
                    if (!pi.crab && pi.min_turn_radius > 0) {
                        goal_pp_diff = goalPP.diff(pp);
                        goal_pp_diffu = goal_pp_diff.unit();
                        goal_pp_diff_mag = goal_pp_diff.mag();
                        if (pp.prev_pt != null) {
                            pp_prev_diff = pp.diff(pp.prev_pt);
                            pp_prev_diffu = pp_prev_diff.unit();
                            pp_prev_diff_mag = pp_prev_diff.mag();
                        }
                    }
                    PlannerPoint closest_point = null;
                    double min_d0 = Double.POSITIVE_INFINITY;
                    for (PlannerPoint pppn : ll2) {
                        Point2Dd pppn_pp_diff = pppn.diff(pp);
                        double d0 = pppn_pp_diff.mag();
                        Point2Dd pppn_pp_diffu = pppn_pp_diff.unit();
                        Point2Dd goal_pppn_diff = goalPP.diff(pppn);
                        Point2Dd goal_pppn_diffu = goal_pppn_diff.unit();
                        double goal_pppn_diff_mag = goal_pppn_diff.mag();
                        boolean goal_directly_reachable = true;
                        if (d0 < min_d0) {
                            closest_point = pppn;
                            min_d0 = d0;
                        }
                        boolean is_goal = false;
                        if (pppn.equals(goalPP)) {
                            is_goal = true;
                        }
                        if (!pi.crab && pi.min_turn_radius > 0) {
                            if (null != pp_prev_diffu) {
                                if (!checkTurn(pp_prev_diffu,
                                        pppn_pp_diffu,
                                        pi,
                                        d0)) {
                                    continue;
                                }
                            }
                            if (goal_pppn_diff_mag > pi.plannerResolution) {
                                if (!checkTurn(goal_pppn_diffu,
                                        pppn_pp_diffu,
                                        pi,
                                        d0)) {
                                    goal_directly_reachable = false;
                                }
                                if (!checkTurn(goal_pppn_diffu,
                                        next_goal_diffu,
                                        pi,
                                        goal_pppn_diff_mag)) {
                                    goal_directly_reachable = false;
                                }
                            } else {
                                if (!checkTurn(pppn_pp_diffu,
                                        next_goal_diffu,
                                        pi,
                                        d0)) {
                                    goal_directly_reachable = false;
                                }
                            }
                        }
                        if (!goal_directly_reachable
                                && pppn.distFromGoal < 3 * pi.min_turn_radius) {
                            continue;
                        }
                        if (d0 <= pi.planningHorizon
                                || pi.planningHorizon <= 0
                                || pppn.equals(goalPP)) {
                            pppn.d1 = pp.distance(pppn) + pppn.distFromGoal;
                            if (!goal_directly_reachable) {
                                pppn.d1 += pi.min_turn_radius;
                            }
                            pppnl.add(pppn);
                        }
                    }
                    if (!pppnl.contains(closest_point)) {
                        pppnl.add(closest_point);
                    }

                    try {
                        Collections.sort(pppnl, new Comparator<PlannerPoint>() {
                            @Override
                            public int compare(PlannerPoint t, PlannerPoint t1) {
                                int result = Double.compare(t.d1, t1.d1);
                                return result;
                            }
                        });
                    } catch (Exception e) {
                        System.err.println();
                        System.err.flush();
//                        System.out.println("pppnl.size() = " + pppnl.size());
//                        for (int i = 0; i < pppnl.size(); i++) {
//                            PlannerPoint pppn = pppnl.get(i);
//                            Double d1chk = d1chkl.get(pppn);
//                            if (d1chk.doubleValue() != pppn.d1) {
//                                System.out.println("i = " + i);
//                                System.out.println("d1chk = " + d1chk);
//                                System.out.println("pppn.d1 = " + pppn.d1);
//                            }
//                        }
                        System.err.println();
                        System.err.flush();
                        System.out.println();
                        System.out.flush();
                        throw e;
                    }
                    pp.potential_neighbors = pppnl;
                }

                int added_neighbors = 0;
                int failed_neighbors = 0;
                pp.potential_neighbors.removeAll(openlist);
                if (pp.potential_neighbors.size() > 0) {
//                while (pp.potential_neighbors.size() > 0
//                        && (added_neighbors < 1 || failed_neighbors < 1)) {
                    PlannerPoint potential_neighbor = pp.potential_neighbors.removeFirst();
                    double dist = pp.distance(potential_neighbor);
                    if (pp == potential_neighbor) {
                        continue;
                    }
                    if (potential_neighbor.pathFromStartDist <= pp.pathFromStartDist + dist - 0.0001f) {
                        continue;
                    }
                    if (pp.neighbors != null && pp.neighbors.contains(potential_neighbor)) {
                        continue;
                    }
                    if (pp.failed_neighbors != null && pp.failed_neighbors.contains(potential_neighbor)) {
                        continue;
                    }
                    pp.pathEstimate = (double) (pp.pathFromStartDist + dist + potential_neighbor.distFromGoal + 0.075f);
                    if (pp.potential_neighbors.size() > 1) {
                        PlannerPoint next_neighbor = pp.potential_neighbors.getFirst();
                        pp.pathEstimate = (double) (pp.pathFromStartDist + pp.distance(next_neighbor) + next_neighbor.distFromGoal + 0.075f);
                    }
                    if (checkPoint(pp, potential_neighbor,
                            pi,
                            false)) {
//                        if (potential_neighbor.selected && pi) {
//                            System.out.println("potential_neighbor = " + potential_neighbor);
//                        }
                        if (pp.neighbors == null) {
                            pp.neighbors = new LinkedList<PlannerPoint>();
                        }
                        pp.neighbors.add(potential_neighbor);
                        potential_neighbor.pathFromStartDist = 0.05f
                                + (double) pp.pathFromStartDist + (double) pp.distance(potential_neighbor);
                        potential_neighbor.prev_pt = pp;
                        potential_neighbor.pathEstimate = potential_neighbor.pathFromStartDist + potential_neighbor.distFromGoal;
                        openlist.add(potential_neighbor);
                        added_neighbors++;
                        if (potential_neighbor == goalPP) {
                            goal_found = true;
                            //break;
                        }
                    } else {
                        failed_neighbors++;
                        if (pp.failed_neighbors == null) {
                            pp.failed_neighbors = new LinkedList<PlannerPoint>();
                        }
                        pp.failed_neighbors.add(potential_neighbor);
                        //break;
                    }
                }
            }
            if (pp.potential_neighbors == null || pp.potential_neighbors.size() < 1) {
                pp.opened = true;
                openlist.remove(pp);
            }
        }
        LinkedList<PlannerPoint> path = new LinkedList<>();
        PlannerPoint pp = goalPP;
//        System.out.println("goalPP.pathFromStartDist = " + goalPP.pathFromStartDist);
//        System.out.println("startPP = " + startPP);
        if (goalPP.prev_pt == null) {
//            if (reverse && !crab) {
//                start.setAngle(orig_start_angle);
//                goal.setAngle(orig_goal_angle);
//            }
            long wall_end_time_ms = System.currentTimeMillis();
            long cpu_end_time_ns = getCpuTimeNs();
            Planner.recordStats(pi, pplist, null, wall_start_time_ms, wall_end_time_ms, cpu_start_time_ns, cpu_end_time_ns);
            plan_time = wall_end_time_ms - wall_start_time_ms;
            if (plan_time > max_plan_time) {
                max_plan_time = plan_time;
            }
            last_points_checked = points_checked;
            if (points_checked > max_points_checked) {
                max_points_checked = points_checked;
            }
            if (null != statusSetter) {
                statusSetter.setPlannerStatus("Planner Status: No route to goal found. points_checked=" + points_checked + ", plan_time=" + plan_time + " ms");
            }
            last_fail_pi = pi.clone();
            last_fail_pplist = new LinkedList<PlannerPoint>();
            last_fail_pplist.addAll(pplist);
            return null;
        }
        double base_dist = computePathDist(pp);
        if (pi.max_cntrl_pts > 0) {
            double bdomn = base_dist / pi.max_cntrl_pts;
            if (bdomn > max_pt2pt_dist) {
                max_pt2pt_dist = bdomn;
            }
        }
        path.addFirst(pp);
        while (pp.prev_pt != null && pp.prev_pt.prev_pt != null && pp.diff(startPP).mag() > Double.MIN_NORMAL) {
//            if(!checkPoint(pp.prev_pt,pp,pp.prev_pt.prev_pt, obstacles,
//                            obs_width,
//                            veh_width,
//                            front,
//                            back, crab,
//                            true)) {
//                System.err.println("checkPoint failed on output path.");
//            }
            double dist = pp.distance(pp.prev_pt);
            if (null == pp.prev_pt.next_pt) {
                pp.prev_pt.next_pt = pp;
            }
            if (dist > max_pt2pt_dist) {
                int num_segments = ((int) (dist / max_pt2pt_dist)) + 1;
                double dist_seg = dist / num_segments;
                Point2Dd pp_prev_diff = pp.diff(pp.prev_pt);
                Point2Dd pp_prev_diffu = pp_prev_diff.unit();
                Point2Dd prev_prev_diff = null;
                Point2Dd prev_prev_diffu = null;
                Point2Dd next_pp_diff = null;
                Point2Dd next_pp_diffu = null;
                if (pp.prev_pt.prev_pt != null) {
                    prev_prev_diff = pp.prev_pt.diff(pp.prev_pt.prev_pt);
                    prev_prev_diffu = prev_prev_diff.unit();
                }
                if (pp.equals(startPP)) {
                }
                if (null != pp.next_pt) {
                    next_pp_diff = pp.next_pt.diff(pp);
                    next_pp_diffu = next_pp_diff.unit();
                }
                for (int i = 1; i < num_segments; i++) {
                    PlannerPoint new_pp
                            = new PlannerPoint(
                                    pp.x * (num_segments - i) / num_segments + pp.prev_pt.x * (i) / num_segments,
                                    pp.y * (num_segments - i) / num_segments + pp.prev_pt.y * (i) / num_segments);
                    if (null != next_pp_diffu
                            && !checkTurn(next_pp_diffu, pp_prev_diff, pi, new_pp.diff(pp).mag())) {
                        continue;
                    }
                    if (null != prev_prev_diffu
                            && !checkTurn(pp_prev_diff, prev_prev_diffu, pi, new_pp.diff(pp.prev_pt).mag())) {
                        continue;
                    }
                    path.addFirst(new_pp);
                }
            }
            pp = pp.prev_pt;
            if (path.contains(pp)) {
                throw new RuntimeException("Bad path already includes :" + pp);
            }

            path.addFirst(pp);
        }
        if (startPP != trueStartPP) {
            path.addFirst(trueStartPP);
        }
        if (goalPP != trueGoalPP) {
            path.add(trueGoalPP);
        }
        plan_time = System.currentTimeMillis() - wall_start_time_ms;
        if (plan_time > max_plan_time) {
            max_plan_time = plan_time;
        }
        long wall_end_time_ms = System.currentTimeMillis();
        long cpu_end_time_ns = getCpuTimeNs();
        Planner.recordStats(pi, pplist, path, wall_start_time_ms, wall_end_time_ms, cpu_start_time_ns, cpu_end_time_ns);
        last_points_checked = points_checked;
        if (points_checked > max_points_checked) {
            max_points_checked = points_checked;
        }
        if (null != statusSetter) {
            statusSetter.setPlannerStatus("Planner Status: path.size()=" + path.size() + ", points_checked=" + points_checked + ", plan_time = " + plan_time + " ms");
        }
        last_succeed_pi = pi.clone();
        last_succeed_pplist = new LinkedList<PlannerPoint>();
        last_succeed_pplist.addAll(pplist);
        return path;
    }

    static private PlannerPoint getBestFromOpenList(List<PlannerPoint> openList) {
        PlannerPoint best_pp = openList.get(0);
        for (PlannerPoint pp : openList) {
            if (pp.pathEstimate < best_pp.pathEstimate) {
                best_pp = pp;
            }
        }
        return best_pp;
    }
    static public int points_checked = 0;
    static public int last_points_checked = 0;
    static public int max_points_checked = 0;
    static public long plan_time = 0;
    static public long max_plan_time = 0;
    static double min_failed_angle = Double.POSITIVE_INFINITY;

    static public boolean checkPoint(
            PlannerPoint p1,
            PlannerPoint p2,
            PlannerInput pi,
            boolean testing) {
        points_checked++;
        PlannerPoint prev_pp = p1.prev_pt;
        PlannerPoint next_pp = p2.next_pt;
        List<Obstacle> obstacles = pi.obstacles;
        double veh_width = pi.veh_width + pi.path_uncertainty;
        double front = pi.front + pi.path_uncertainty;
        double back = pi.back + pi.path_uncertainty;
        boolean crab = pi.crab;
        boolean reverse = pi.reverse;
        if (reverse && !crab) {
            double tmp = front;
            front = back;
            back = tmp;
        }
        List<Boundary> boundaries = pi.boundaries;
        double left_dot_max = veh_width / 2f;
        double right_dot_max = veh_width / 2f;
        p1.checked = true;
        p2.checked = true;

        Point2Dd diff = p2.diff(p1);
        Point2Dd diffu = diff.unit();
        Point2Dd orig_diffu = diffu;
        if (pi.crab) {
            diffu = new Point2Dd(pi.start.getAngle().cos(), pi.start.getAngle().sin());
        }
        double diff_mag = diff.mag();
        Point2Dd p1back = new Point2Dd(p1.x - diffu.x * back, p1.y - diffu.y * back);
        Point2Dd p1front = new Point2Dd(p1.x + diffu.x * front,
                p1.y + diffu.y * front);
        Point2Dd p1front_left = new Point2Dd(p1front.x - diffu.y * veh_width / 2f, p1front.y + diffu.x * veh_width / 2f);
        Point2Dd p1front_right = new Point2Dd(p1front.x + diffu.y * veh_width / 2f, p1front.y - diffu.x * veh_width / 2f);
        Point2Dd p1back_left = new Point2Dd(p1back.x - diffu.y * veh_width / 2f, p1back.y + diffu.x * veh_width / 2f);
        Point2Dd p1back_right = new Point2Dd(p1back.x + diffu.y * veh_width / 2f, p1back.y - diffu.x * veh_width / 2f);
        Point2Dd p2front = new Point2Dd(p2.x + diffu.x * front, p2.y + diffu.y * front);
        Point2Dd p2front_left = new Point2Dd(p2front.x - diffu.y * veh_width / 2f, p2front.y + diffu.x * veh_width / 2f);
        Point2Dd p2front_right = new Point2Dd(p2front.x + diffu.y * veh_width / 2f, p2front.y - diffu.x * veh_width / 2f);
        Point2Dd p2back = new Point2Dd(p2.x - diffu.x * back,
                p2.y - diffu.y * back);
        Point2Dd p2back_left = new Point2Dd(p2back.x - diffu.y * veh_width / 2f, p2back.y + diffu.x * veh_width / 2f);
        Point2Dd p2back_right = new Point2Dd(p2back.x + diffu.y * veh_width / 2f, p2back.y - diffu.x * veh_width / 2f);
        List<List<Point2Dd>> polys;
        if (crab) {
            List<Point2Dd> front_poly = Arrays.asList(p1front_left, p1front_right, p2front_right, p2front_left);
            List<Point2Dd> back_poly = Arrays.asList(p1back_left, p1back_right, p2back_right, p2back_left);
            List<Point2Dd> left_poly = Arrays.asList(p1front_left, p1back_left, p2back_left, p2front_left);
            List<Point2Dd> right_poly = Arrays.asList(p1front_right, p1back_right, p2back_right, p2front_right);
            polys = Arrays.asList(front_poly, back_poly, left_poly, right_poly);
        } else {
            List<Point2Dd> back_to_front_poly = Arrays.asList(p1back_left, p1back_right, p2front_right, p2front_left);
            polys = Arrays.asList(back_to_front_poly);
        }
        LinkedList<Line2Dd> outlines = new LinkedList<>();
        for (List<Point2Dd> poly : polys) {
            Point2Dd last_poly_pt = poly.get(poly.size() - 1);
            for (Point2Dd poly_pt : poly) {
                outlines.add(new Line2Dd(poly_pt, last_poly_pt));
                last_poly_pt = poly_pt;
            }
        }
        LinkedList<Line2Dd> prev_back_rot_lines = new LinkedList<>();
        LinkedList<Line2Dd> prev_front_rot_lines = new LinkedList<>();
        LinkedList<Line2Dd> next_back_rot_lines = new LinkedList<>();
        LinkedList<Line2Dd> next_front_rot_lines = new LinkedList<>();
        if (crab) {
            outlines.add(new Line2Dd(p1back_right, p2back_left));
            outlines.add(new Line2Dd(p1back_left, p2front_right));
            outlines.add(new Line2Dd(p1back_left, p2back_left));
            outlines.add(new Line2Dd(p1back_right, p2back_right));
            outlines.add(new Line2Dd(p1front_left, p2front_left));
            outlines.add(new Line2Dd(p1front_right, p2front_right));
        }
        double prev_back_dist_min = 0f;
        double prev_front_dist_min = 0f;
        double next_back_dist_min = 0f;
        double next_front_dist_min = 0f;
        double prev_angle = 0.0;
        double next_angle = 0.0;
        if (p1.selected && p2.selected) {
            System.out.println("checkPoint(p1 = " + p1 + ",p2=" + p2 + ")");
        }
        List<Line2Dd> testedOutLines = new LinkedList<>();
        testedOutLines.addAll(outlines);
        List<Line2Dd> testedInLines = new LinkedList<>();
        testedInLines.addAll(outlines);

        double back_corner_dist = (double) Math.sqrt(back * back + veh_width * veh_width / 4f);
        double front_corner_dist = (double) Math.sqrt(front * front + veh_width * veh_width / 4f);
        if (!crab) {
            if (null != prev_pp) {
                Point2Dd prev_diff = p1.diff(prev_pp);
                Point2Dd prev_diffu = prev_diff.unit();
                Point2Dd prev_back = new Point2Dd(p1.x - prev_diffu.x * back,
                        p1.y - prev_diffu.y * back);
                Point2Dd prev_back_left = new Point2Dd(
                        prev_back.x - prev_diffu.y * veh_width / 2f,
                        prev_back.y + prev_diffu.x * veh_width / 2f);
                Point2Dd prev_back_right = new Point2Dd(
                        prev_back.x + prev_diffu.y * veh_width / 2f,
                        prev_back.y - prev_diffu.x * veh_width / 2f);

                Point2Dd prev_front = new Point2Dd(p1.x + prev_diffu.x * front,
                        p1.y + prev_diffu.y * front);
                Point2Dd prev_front_left = new Point2Dd(
                        prev_front.x - prev_diffu.y * veh_width / 2f,
                        prev_front.y + prev_diffu.x * veh_width / 2f);
                Point2Dd prev_front_right = new Point2Dd(
                        prev_front.x + prev_diffu.y * veh_width / 2f,
                        prev_front.y - prev_diffu.x * veh_width / 2f);
                prev_back_rot_lines.add(new Line2Dd(p1back_left, prev_back_left));
                prev_back_rot_lines.add(new Line2Dd(p1back_right, prev_back_right));
                prev_front_rot_lines.add(new Line2Dd(p1front_left, prev_front_left));
                prev_front_rot_lines.add(new Line2Dd(p1front_right, prev_front_right));
                double dot_l1_l2 = prev_diffu.dot(diffu);
                prev_angle = Math.acos(dot_l1_l2);
                prev_back_dist_min = back_corner_dist;
                prev_back_dist_min *= (1 - Math.abs(Math.cos(prev_angle / 2.0)));
                prev_front_dist_min = front_corner_dist;
                prev_front_dist_min *= (1 - Math.abs(Math.cos(prev_angle / 2.0)));
                if (pi.min_turn_radius > 0) {
//                    double angle_cos = prev_diffu.dot(diffu);
//                    if (angle_cos < 0) {
//                        return false;
//                    }
//                    if (angle_cos < 0.99) {
//                        if (diff_mag / Math.sqrt(1 - angle_cos * angle_cos) < pi.min_turn_radius) {
//                            return false;
//                        }
//                    }
                    if (!checkTurn(prev_diffu, diffu, pi, diff_mag)) {
                        return false;
                    }
                }
            }
            if (null != next_pp) {
                Point2Dd next_diff = next_pp.diff(p2);
                Point2Dd next_diffu = next_diff.unit();
                if (pi.crab) {
                    next_diffu = new Point2Dd(pi.start.getAngle().cos(), pi.start.getAngle().sin());
                }
                Point2Dd next_back = new Point2Dd(
                        p2.x - next_diffu.x * back,
                        p2.y - next_diffu.y * back);
                Point2Dd next_back_left = new Point2Dd(
                        next_back.x - next_diffu.y * veh_width / 2f,
                        next_back.y + next_diffu.x * veh_width / 2f);
                Point2Dd next_back_right = new Point2Dd(
                        next_back.x + next_diffu.y * veh_width / 2f,
                        next_back.y - next_diffu.x * veh_width / 2f);
                Point2Dd next_front = new Point2Dd(
                        p2.x + next_diffu.x * front,
                        p2.y + next_diffu.y * front);
                Point2Dd next_front_left = new Point2Dd(
                        next_front.x - next_diffu.y * veh_width / 2f,
                        next_front.y + next_diffu.x * veh_width / 2f);
                Point2Dd next_front_right = new Point2Dd(
                        next_front.x + next_diffu.y * veh_width / 2f,
                        next_front.y - next_diffu.x * veh_width / 2f);
                double next_dot = next_diffu.dot(diffu);
                next_angle = Math.acos(next_dot);
                next_back_dist_min = back_corner_dist;
                next_back_dist_min *= (1 - Math.abs(Math.cos(next_angle / 2.0)));
                next_front_dist_min = front_corner_dist;
                next_front_dist_min *= (1 - Math.abs(Math.cos(next_angle / 2.0)));
                next_back_rot_lines.add(new Line2Dd(p2back_left, next_back_left));
                next_back_rot_lines.add(new Line2Dd(p2back_right, next_back_right));
                next_front_rot_lines.add(new Line2Dd(p2front_left, next_front_left));
                next_front_rot_lines.add(new Line2Dd(p2front_right, next_front_right));
                if (pi.min_turn_radius > 0) {
//                    double angle_cos = diffu.dot(next_diffu);
//                    if (angle_cos < 0) {
//                        return false;
//                    }
//                    if (angle_cos < 0.99) {
//                        if (diff_mag / Math.sqrt(1 - angle_cos * angle_cos) < pi.min_turn_radius) {
//                            return false;
//                        }
//                    }
                    if (!checkTurn(next_diffu, diffu, pi, diff_mag)) {
                        return false;
                    }
                }
            }
            testedOutLines.addAll(prev_back_rot_lines);
            testedInLines.addAll(prev_back_rot_lines);
            testedOutLines.addAll(prev_front_rot_lines);
            testedInLines.addAll(prev_front_rot_lines);
            testedOutLines.addAll(next_back_rot_lines);
            testedInLines.addAll(next_back_rot_lines);
            testedOutLines.addAll(next_front_rot_lines);
            testedInLines.addAll(next_front_rot_lines);
        }

        if (p1.testedOutLinesMap == null) {
            p1.testedOutLinesMap = new HashMap<>();
        }
        p1.testedOutLinesMap.put(p2, testedOutLines);
        if (p2.testedInLinesMap == null) {
            p2.testedInLinesMap = new HashMap<>();
        }
        p2.testedInLinesMap.put(p1, testedInLines);
        double max_frontback = Math.max(front, back);
        if (null != obstacles) {
            for (Obstacle obs : obstacles) {
                for (List<Point2Dd> poly : polys) {
                    if (!checkPoly(poly, obs)) {
                        return false;
                    }
                }
                double min_dist = (obs.radius + veh_width / 2f);
                if (!checkPointSingleObs(p1, p2, orig_diffu, obs,
                        min_dist,
                        -back - obs.radius,
                        front + diff_mag + obs.radius,
                        crab,
                        testing)) {
                    return false;
                }
                if (null != outlines) {
                    for (Line2Dd l : outlines) {
                        if (l.ptSegDist(obs) < obs.radius) {
                            return false;
                        }
                    }
                }
                if (null != prev_back_rot_lines) {
                    for (Line2Dd l : prev_back_rot_lines) {
                        if (null != l && !checkLineWithObs(p1, l, obs, back_corner_dist + obs.radius)) {
                            return false;
                        }
                    }
                }
                if (null != prev_front_rot_lines) {
                    for (Line2Dd l : prev_front_rot_lines) {
                        if (null != l && !checkLineWithObs(p1, l, obs, front_corner_dist + obs.radius)) {
                            return false;
                        }
                    }
                }
                if (null != next_back_rot_lines) {
                    for (Line2Dd l : next_back_rot_lines) {
                        if (null != l && !checkLineWithObs(p2, l, obs, back_corner_dist + obs.radius)) {
                            return false;
                        }
                    }
                }
                if (null != next_front_rot_lines) {
                    for (Line2Dd l : next_front_rot_lines) {
                        if (null != l && !checkLineWithObs(p2, l, obs, front_corner_dist + obs.radius)) {
                            return false;
                        }
                    }
                }
            }
        }
        if (null != boundaries) {
            for (Boundary b : boundaries) {
                if (Double.isNaN(b.x2) || Double.isNaN(b.y2)) {
                    continue;
                }
                if (p1.selected && p2.selected && (b.P1Selected || b.P2Selected)) {
                    System.out.println("b = " + b);
                }
                for (List<Point2Dd> poly : polys) {
                    if (!checkPoly(poly, b)) {
                        return false;
                    }
                }
                for (Line2Dd outline_l : outlines) {
                    if (b.intersectsLine(outline_l)) {
                        return false;
                    }
                }
                if (null != prev_back_rot_lines) {
                    for (Line2Dd l : prev_back_rot_lines) {
                        if (null != l && !checkLineWithBoundary(p1, l, b, prev_back_dist_min)) {
                            return false;
                        }
                    }
                }
                if (null != prev_front_rot_lines) {
                    for (Line2Dd l : prev_front_rot_lines) {
                        if (null != l && !checkLineWithBoundary(p1, l, b, prev_front_dist_min)) {
                            return false;
                        }
                    }
                }
                if (null != next_back_rot_lines) {
                    for (Line2Dd l : next_back_rot_lines) {
                        if (null != l && !checkLineWithBoundary(p2, l, b, next_back_dist_min)) {
                            return false;
                        }
                    }
                }
                if (null != next_front_rot_lines) {
                    for (Line2Dd l : next_front_rot_lines) {
                        if (null != l && !checkLineWithBoundary(p2, l, b, next_front_dist_min)) {
                            return false;
                        }
                    }
                }
            }
        }
        return true;
    }

    static public boolean checkPoly(List<Point2Dd> poly, Obstacle o) {
        Point2Dd last_pt = null;
        boolean last_sign = false;
        boolean last_sign_set = false;
        int sign_change_count = 0;
        for (int i = 0; i < poly.size(); i++) {
            if (last_pt == null) {
                last_pt = poly.get(poly.size() - 1);
            }
            Point2Dd pt = poly.get(i);
            Line2Dd line = new Line2Dd(pt, last_pt);
            if (line.ptSegDist(o) < o.radius) {
                return false;
            }
            Point2Dd diff = pt.diff(last_pt);
            if (diff.mag() < 1e-4) {
                continue;
            }
            Point2Dd diffo = o.diff(last_pt);
            Point2Dd diffu = diff.unit();
            Point2Dd diffou = diffo.unit();
            boolean sign = (diffu.x * diffou.y - diffu.y * diffou.x) > 0;
            if (last_sign_set && sign != last_sign) {
                sign_change_count++;
            }
            last_sign = sign;
            last_sign_set = true;
            last_pt = pt;
        }
        return (sign_change_count > 0);
    }

    static public boolean checkPoly(List<Point2Dd> poly, Boundary b) {
        Point2Dd last_pt = null;
        boolean last_p1_sign = false;
        boolean last_p2_sign = false;
        boolean last_p1_sign_set = false;
        boolean last_p2_sign_set = false;
        int p1_sign_change_count = 0;
        int p2_sign_change_count = 0;
        Point2Dd p1 = b.getP1();
        Point2Dd p2 = b.getP2();
        if (Double.isInfinite(p1.x) || Double.isNaN(p1.x)) {
            return true;
        }
        if (Double.isInfinite(p1.y) || Double.isNaN(p1.y)) {
            return true;
        }
        if (Double.isInfinite(p2.x) || Double.isNaN(p2.x)) {
            return true;
        }
        if (Double.isInfinite(p2.y) || Double.isNaN(p2.y)) {
            return true;
        }
        for (int i = 0; i < poly.size(); i++) {
            if (last_pt == null) {
                last_pt = poly.get(poly.size() - 1);
            }
            Point2Dd pt = poly.get(i);
            Line2Dd line = new Line2Dd(pt, last_pt);
            if (line.intersectsLine(b)) {
                return false;
            }
            Point2Dd diff = pt.diff(last_pt);
            if (diff.mag() < 1e-5) {
                continue;
            }
            Point2Dd diffp1 = p1.diff(last_pt);
            Point2Dd diffp2 = p2.diff(last_pt);
            Point2Dd diffu = diff.unit();
            Point2Dd diffp1u = diffp1.unit();
            Point2Dd diffp2u = diffp2.unit();
            boolean p1_sign = (diffu.x * diffp1u.y - diffu.y * diffp1u.x) > 0;
            if (last_p1_sign_set && p1_sign != last_p1_sign) {
                p1_sign_change_count++;
            }
            boolean p2_sign = (diffu.x * diffp2u.y - diffu.y * diffp2u.x) > 0;
            if (last_p2_sign_set && p2_sign != last_p2_sign) {
                p2_sign_change_count++;
            }
            last_p1_sign = p1_sign;
            last_p2_sign = p2_sign;
            last_p1_sign_set = true;
            last_p2_sign_set = true;
            last_pt = pt;
        }
        return (p1_sign_change_count > 0 && p2_sign_change_count > 0);
    }

    static private boolean checkLineWithObs(Point2Dd pt, Line2Dd l, Obstacle o, double dist) {
        if (pt.distance(o) > dist) {
            return true;
        }
        if (l.ptSegDist(o) < o.radius) {
            return false;
        }
        Line2Dd lo = new Line2Dd(o, pt);
        if (lo.intersectsLine(l)) {
            return false;
        }
        Point2Dd diffu = o.diff(pt).unit();
        Line2Dd lor = new Line2Dd(new Point2Dd(o.x + diffu.y * o.radius, o.y - diffu.x * o.radius), pt);
        if (lor.intersectsLine(l)) {
            return false;
        }
        Line2Dd lol = new Line2Dd(new Point2Dd(o.x - diffu.y * o.radius, o.y + diffu.x * o.radius), pt);
        if (lol.intersectsLine(l)) {
            return false;
        }
        return true;
    }

    static private boolean checkLineWithBoundary(Point2Dd pt, Line2Dd l, Boundary b, double dist) {
        if (l.intersectsLine(b)) {
            return false;
        }
        if (b.ptSegDist(pt) > dist) {
            return true;
        }
        Point2Dd p1 = b.getP1();
        if (p1.distance(pt) < dist && new Line2Dd(p1, pt).intersectsLine(l)) {
            return false;
        }
        Point2Dd p2 = b.getP2();
        if (p2.distance(pt) < dist && new Line2Dd(p2, pt).intersectsLine(l)) {
            return false;
        }
        return true;
    }

    static private boolean checkPointSingleObs(
            PlannerPoint p1,
            PlannerPoint p2,
            Point2Dd diffu,
            Obstacle obs,
            double min_dist,
            double dot_min,
            double dot_max,
            boolean crab,
            boolean testing) {

        if (p1.distance(obs) < min_dist) {
            return false;
        }
        if (p2.distance(obs) < min_dist) {
            return false;
        }
        final boolean print_info = testing || (p1.selected && p2.selected && obs.selected); // obs.selected;
//        if (print_info
//                && obs.distance(p2) > p1.distance(p2)) {
//            print_info = false;
//        }
//        if (print_info
//                && obs.distance(p1) > p1.distance(p2)) {
//            print_info = false;
//        }
        Point2Dd d1 = new Point2Dd();
        d1.x = obs.x - p1.x;
        d1.y = obs.y - p1.y;
//        if (testing || obs.selected) {
//            System.out.println("d1 = " + d1);
//        }
        double dot = d1.x * diffu.x + d1.y * diffu.y;
        if (dot < dot_min || dot > dot_max) {
            if (print_info) {
                System.out.println("dot = " + dot);
                System.out.println("dot_min = " + dot_min);
                System.out.println("dot_max = " + dot_max);
            }
            return true;
        }
        double left_dot = -d1.x * diffu.y + d1.y * diffu.x;
        double right_dot = d1.x * diffu.y - d1.y * diffu.x;
        if (print_info) {
            System.out.println("left_dot = " + left_dot);
            System.out.println("right_dot = " + right_dot);
        }
        System.out.flush();
        if (left_dot >= 0 && left_dot < min_dist) {
            if (testing) {
                System.out.println("test failed.");
            }
            return false;
        }
        if (right_dot >= 0 && right_dot < min_dist) {
            if (testing) {
                System.out.println("test failed.");
            }
            return false;
        }
        if (print_info) {
            System.out.println("min_dist = " + min_dist);
            System.out.println("left_dot = " + left_dot);
            System.out.println("right_dot = " + right_dot);
        }
        return true;
    }

}
