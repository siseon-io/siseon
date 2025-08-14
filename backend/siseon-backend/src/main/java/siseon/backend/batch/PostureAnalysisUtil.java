package siseon.backend.batch;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class PostureAnalysisUtil {

    private static final Logger log = LoggerFactory.getLogger(PostureAnalysisUtil.class);
    private static final double[] VERTICAL = new double[]{0, 1, 0};

    // ====== 임계치 (한 곳에서 관리) ======
    public static final double THRESH_FHP_DEG   = 46.0;   // 거북목
    public static final double THRESH_RSA_DEG   = 10.0;   // 굽은 어깨
    public static final double THRESH_KYPH_DEG  = 46.83;  // 등 굽음
    public static final double THRESH_SIDE_DEG  = 10.0;   // 좌우 기울임

    public static class PostureResult {
        public double forwardHeadAngle;
        public boolean forwardHeadPosture;
        public double roundedShoulderAngle;
        public boolean roundedShoulders;
        public double kyphosisAngle;
        public boolean slouching;
        public double sideLeanAngle;
        public String sideLeaning; // "left" | "right" | null

        // 수집된 bad 사유
        private final List<Map<String, Object>> reasons = new ArrayList<>();

        public boolean isValidPosture() { return reasons.isEmpty(); }
        public List<Map<String, Object>> getReasons() { return reasons; }

        private static String severity(double angle, double threshold) {
            double over = Math.max(0, angle - threshold);
            if (over >= 15) return "high";
            if (over >= 5)  return "moderate";
            return "low";
        }

        private static double round1(double d) { return Math.round(d * 10.0) / 10.0; }

        private void addReason(String code, String label, double angle, double threshold, Map<String, Object> extra) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("code", code);
            m.put("label", label);
            m.put("angle", round1(angle));
            m.put("threshold", threshold);
            m.put("severity", severity(angle, threshold)); // low / moderate / high
            if (extra != null && !extra.isEmpty()) m.putAll(extra);
            reasons.add(m);
        }
    }

    /** 프론트 저장/표시용 표준 구조 */
    public static Map<String, Object> buildBadReasonsMap(PostureResult r) {
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("valid", r.isValidPosture());
        out.put("reasons", r.getReasons());
        out.put("summary", buildSummaryKo(r));
        return out;
    }

    private static String buildSummaryKo(PostureResult r) {
        List<String> parts = new ArrayList<>();
        for (var each : r.getReasons()) {
            String label = String.valueOf(each.get("label"));
            double angle = (double) each.get("angle");
            if ("좌우 기울임".equals(label) && each.containsKey("direction")) {
                String dirKo = "left".equals(each.get("direction")) ? "좌측" : "우측";
                parts.add(dirKo + " 기울임(" + fmt(angle) + ")");
            } else {
                parts.add(label + "(" + fmt(angle) + ")");
            }
        }
        return parts.isEmpty() ? "양호" : String.join(", ", parts);
    }

    private static String fmt(double d) { return String.format("%.1f°", d); }

    // ====== 벡터 유틸 ======
    private static double angleBetween(double[] v1, double[] v2) {
        double n1 = norm(v1), n2 = norm(v2);
        double dot = (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]) / (n1 * n2 + 1e-8);
        dot = Math.max(-1.0, Math.min(1.0, dot));
        return Math.toDegrees(Math.acos(dot));
    }
    private static double norm(double[] v) { return Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }
    private static double[] sub(double[] a, double[] b) { return new double[]{ a[0]-b[0], a[1]-b[1], a[2]-b[2] }; }
    private static double[] avg(double[]... vs) {
        double[] s = new double[3];
        for (var v : vs) { s[0]+=v[0]; s[1]+=v[1]; s[2]+=v[2]; }
        s[0]/=vs.length; s[1]/=vs.length; s[2]/=vs.length;
        return s;
    }

    @SuppressWarnings("unchecked")
    public static PostureResult analyze(Map<String,double[]> pts) {
        PostureResult r = new PostureResult();

        // 기준점 계산
        double[] headCenter     = avg(pts.get("le_ear"), pts.get("re_ear"), pts.get("nose"));
        double[] shoulderCenter = avg(pts.get("le_shoulder"), pts.get("re_shoulder"));
        double[] hipCenter      = avg(pts.get("le_hip"), pts.get("re_hip"));

        // 1) 거북목 (FHP)
        double[] vShToHead = sub(headCenter, shoulderCenter);
        r.forwardHeadAngle   = angleBetween(vShToHead, VERTICAL);
        r.forwardHeadPosture = r.forwardHeadAngle > THRESH_FHP_DEG;
        if (r.forwardHeadPosture) {
            r.addReason("FHP", "거북목", r.forwardHeadAngle, THRESH_FHP_DEG, Map.of(
                    // 즉시 적용 가능한 교정 가이드(운동 지시 X)
                    "cue", "턱을 뒤로 살짝 당겨 목 뒤가 길어지게 하고, 귀-어깨가 수직선상에 오도록 맞춰주세요",
                    "ergonomics", "턱을 뒤로 당겨 목 뒤를 곧게 세우고, 귀와 어깨가 일직선이 되도록 하세요"
            ));
        }
        log.debug("FHA Angle = {}, forwardHeadPosture = {}", r.forwardHeadAngle, r.forwardHeadPosture);

        // 2) 굽은 어깨 (Rounded Shoulders)
        double[] earCenter = avg(pts.get("le_ear"), pts.get("re_ear"));
        double[] vEarToSh  = sub(shoulderCenter, earCenter);
        r.roundedShoulderAngle = Math.toDegrees(Math.atan2(Math.abs(vEarToSh[2]), Math.abs(vEarToSh[1])));
        r.roundedShoulders     = r.roundedShoulderAngle > THRESH_RSA_DEG;
        if (r.roundedShoulders) {
            r.addReason("RSA", "굽은 어깨", r.roundedShoulderAngle, THRESH_RSA_DEG, Map.of(
                    "cue", "어깨를 위로 올리지 말고 살짝 뒤로 당겨 가슴을 부드럽게 펼친 느낌을 유지하세요",
                    "ergonomics", "키보드와 마우스는 몸 가까이 두고, 팔꿈치는 몸통 옆에서 자연스럽게 구부린 상태를 유지하세요"
            ));
        }
        log.debug("RSA Angle = {}, roundedShoulders = {}", r.roundedShoulderAngle, r.roundedShoulders);

        // 3) 등 굽음 (Kyphosis/Slouching)
        double[] vHipToSh = sub(shoulderCenter, hipCenter);
        r.kyphosisAngle = angleBetween(vHipToSh, VERTICAL);
        r.slouching     = r.kyphosisAngle > THRESH_KYPH_DEG;
        if (r.slouching) {
            r.addReason("KYP", "등 굽음", r.kyphosisAngle, THRESH_KYPH_DEG, Map.of(
                    "cue", "배꼽 위쪽(명치)와 골반을 같은 기둥에 세운 느낌으로 앉고, 흉곽을 살짝 위로 들어 올리세요",
                    "ergonomics", "등받이에 허리(요추) 지지점을 대고 엉덩이를 등받이 깊숙이 붙여 척추가 세워지게 하세요"
            ));
        }
        log.debug("Kyphosis Angle = {}, slouching = {}", r.kyphosisAngle, r.slouching);

        // 4) 좌우 기울임 (Side Lean)
        double lateral = vHipToSh[0], vertical = vHipToSh[1];
        r.sideLeanAngle = Math.toDegrees(Math.atan2(Math.abs(lateral), Math.abs(vertical)));
        r.sideLeaning   = r.sideLeanAngle > THRESH_SIDE_DEG ? (lateral > 0 ? "right" : "left") : null;
        if (r.sideLeaning != null) {
            String dirKo = "left".equals(r.sideLeaning) ? "좌측" : "우측";
            // label 자체를 좌측/우측 기울임으로 변경
            r.addReason("SIDE", dirKo + " 기울임", r.sideLeanAngle, THRESH_SIDE_DEG, Map.of(
                    "direction", r.sideLeaning,
                    "cue", dirKo + "으로 몸이 기울었습니다. 양 엉덩이뼈에 체중을 균등히 싣고, 가슴뼈가 배꼽 위에 오도록 정렬하세요",
                    "ergonomics", "의자 중앙에 앉고 팔걸이 높이를 좌우 동일하게 맞춰 골반이 한쪽으로 밀리지 않게 하세요"
            ));
        }
        log.debug("Side Lean Angle = {}, sideLeaning = {}", r.sideLeanAngle, r.sideLeaning);

        log.info("PostureResult => FHA: {}, RSA: {}, Kyphosis: {}, SideLean: {}, Valid: {}",
                r.forwardHeadAngle, r.roundedShoulderAngle, r.kyphosisAngle, r.sideLeanAngle, r.isValidPosture());

        return r;
    }
}