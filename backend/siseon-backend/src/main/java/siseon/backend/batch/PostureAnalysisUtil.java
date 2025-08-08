package siseon.backend.batch;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.Map;

public class PostureAnalysisUtil {

    private static final Logger log = LoggerFactory.getLogger(PostureAnalysisUtil.class);
    private static final double[] VERTICAL = new double[]{0, 1, 0};

    public static class PostureResult {
        public double forwardHeadAngle;
        public boolean forwardHeadPosture;
        public double roundedShoulderAngle;
        public boolean roundedShoulders;
        public double kyphosisAngle;
        public boolean slouching;
        public double sideLeanAngle;
        public String sideLeaning;

        public boolean isValidPosture() {
            return !forwardHeadPosture
                    && !roundedShoulders
                    && !slouching
                    && sideLeaning == null;
        }
    }

    private static double angleBetween(double[] v1, double[] v2) {
        double n1 = norm(v1), n2 = norm(v2);
        double dot = (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]) / (n1 * n2 + 1e-8);
        dot = Math.max(-1.0, Math.min(1.0, dot));
        return Math.toDegrees(Math.acos(dot));
    }

    private static double norm(double[] v) {
        return Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    }

    private static double[] sub(double[] a, double[] b) {
        return new double[]{ a[0]-b[0], a[1]-b[1], a[2]-b[2] };
    }

    private static double[] avg(double[]... vs) {
        double[] s = new double[3];
        for (var v : vs) {
            s[0] += v[0]; s[1] += v[1]; s[2] += v[2];
        }
        for (int i = 0; i < 3; i++) s[i] /= vs.length;
        return s;
    }

    @SuppressWarnings("unchecked")
    public static PostureResult analyze(Map<String,double[]> pts) {
        PostureResult r = new PostureResult();

        // 기준점 계산
        double[] headCenter     = avg(pts.get("le_ear"), pts.get("re_ear"), pts.get("nose"));
        double[] shoulderCenter = avg(pts.get("le_shoulder"), pts.get("re_shoulder"));
        double[] hipCenter      = avg(pts.get("le_hip"), pts.get("re_hip"));

        // 1. 거북목 FHA
        double[] vShToHead = sub(headCenter, shoulderCenter);
        r.forwardHeadAngle   = angleBetween(vShToHead, VERTICAL);
        r.forwardHeadPosture = r.forwardHeadAngle > 46.0;
        log.debug("FHA Angle = {}, forwardHeadPosture = {}", r.forwardHeadAngle, r.forwardHeadPosture);

        // 2. 굽은 어깨 (앞뒤 기울기)
        double[] earCenter = avg(pts.get("le_ear"), pts.get("re_ear"));
        double[] vEarToSh  = sub(shoulderCenter, earCenter);
        r.roundedShoulderAngle = Math.toDegrees(
                Math.atan2(Math.abs(vEarToSh[2]), Math.abs(vEarToSh[1]))
        );
        r.roundedShoulders = r.roundedShoulderAngle > 10.0;
        log.debug("RSA Angle = {}, roundedShoulders = {}", r.roundedShoulderAngle, r.roundedShoulders);

        // 3. 등 굽음/흉추 후만
        double[] vHipToSh = sub(shoulderCenter, hipCenter);
        r.kyphosisAngle = angleBetween(vHipToSh, VERTICAL);
        r.slouching     = r.kyphosisAngle > 46.83;
        log.debug("Kyphosis Angle = {}, slouching = {}", r.kyphosisAngle, r.slouching);

        // 4. 좌우 기울임
        double lateral = vHipToSh[0], vertical = vHipToSh[1];
        r.sideLeanAngle = Math.toDegrees(Math.atan2(Math.abs(lateral), Math.abs(vertical)));
        r.sideLeaning   = r.sideLeanAngle > 10.0
                ? (lateral > 0 ? "right" : "left")
                : null;
        log.debug("Side Lean Angle = {}, sideLeaning = {}", r.sideLeanAngle, r.sideLeaning);

        log.info("PostureResult => FHA: {}, RSA: {}, Kyphosis: {}, SideLean: {}, Valid: {}",
                r.forwardHeadAngle, r.roundedShoulderAngle, r.kyphosisAngle, r.sideLeanAngle, r.isValidPosture());

        return r;
    }
}