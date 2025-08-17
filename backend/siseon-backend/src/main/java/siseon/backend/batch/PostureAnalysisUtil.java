package siseon.backend.batch;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class PostureAnalysisUtil {

    private static final Logger log = LoggerFactory.getLogger(PostureAnalysisUtil.class);

    // ====== 임계치 (한 곳에서 관리) ======
    // FHP: 각도 + 전방 갭 복합 판정 (WARN / CRIT / DZ-ONLY)
    public static final double THRESH_FHP_ANGLE_WARN   = 28.0;   // 경계 각도 (경미한 거북목 신호)
    public static final double THRESH_FHP_ANGLE_CRIT   = 40.0;   // 확실한 거북목 각도
    public static final double FHP_Z_GAP_MIN_LOW       = 20.0;   // 각도 판정이 의미 있으려면 최소 z-gap
    public static final double FHP_Z_GAP_MIN_HIGH      = 60.0;   // z-gap 만으로도 강제 판정

    // 기타 기존 임계치
    public static final double THRESH_RSA_DEG   = 45.0;   // 굽은 어깨
    public static final double THRESH_KYPH_DEG  = 32.0;   // 등 굽음(힙 있을 때만)
    public static final double THRESH_SIDE_DEG  = 12.0;   // 좌우 기울임(힙 있을 때만)

    private static final double MISSING_EPS     = 1e-6;   // 결측 판단

    public static class PostureResult {
        public double forwardHeadAngle;
        public boolean forwardHeadPosture;
        public double roundedShoulderAngle;
        public boolean roundedShoulders;
        public double kyphosisAngle;
        public boolean slouching;
        public double sideLeanAngle;
        public String sideLeaning; // "left" | "right" | null

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

    // ⚠️ SIDE 요약 방식 수정: code == "SIDE" 이고 direction 존재 시 방향 텍스트 노출
    private static String buildSummaryKo(PostureResult r) {
        List<String> parts = new ArrayList<>();
        for (var each : r.getReasons()) {
            String code = String.valueOf(each.get("code"));
            String label = String.valueOf(each.get("label"));
            double angle = (double) each.get("angle");
            if ("SIDE".equals(code) && each.containsKey("direction")) {
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
    private static double norm(double[] v) { return Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }
    private static double[] sub(double[] a, double[] b) { return new double[]{ a[0]-b[0], a[1]-b[1], a[2]-b[2] }; }
    private static double[] avg(double[]... vs) {
        double[] s = new double[3];
        for (var v : vs) { s[0]+=v[0]; s[1]+=v[1]; s[2]+=v[2]; }
        s[0]/=vs.length; s[1]/=vs.length; s[2]/=vs.length;
        return s;
    }
    private static boolean isZero(double[] p) {
        return Math.abs(p[0]) < MISSING_EPS && Math.abs(p[1]) < MISSING_EPS && Math.abs(p[2]) < MISSING_EPS;
    }
    private static double degAtan2(double a, double b) {
        return Math.toDegrees(Math.atan2(Math.abs(a), Math.abs(b)));
    }

    /**
     * 입력: 키포인트 맵(각 value는 {x,y,z} double[])
     * 좌표계 가정: y는 화면 위/아래 방향(부호는 데이터에 맞춰 절대값 기반)
     */
    @SuppressWarnings("unchecked")
    public static PostureResult analyze(Map<String,double[]> pts) {
        PostureResult r = new PostureResult();

        // 기준점 계산
        double[] headCenter     = avg(pts.get("le_ear"), pts.get("re_ear"), pts.get("nose"));
        double[] shoulderCenter = avg(pts.get("le_shoulder"), pts.get("re_shoulder"));

        // 힙 결측 여부
        double[] leHip = pts.getOrDefault("le_hip", new double[]{0,0,0});
        double[] reHip = pts.getOrDefault("re_hip", new double[]{0,0,0});
        double[] hipCenter      = avg(leHip, reHip);
        boolean hipMissing      = isZero(leHip) && isZero(reHip);

        // ─────────────────────────────────────
        // 1) 거북목 (FHP): 복합 조건(각도 + 전방 갭) 방식
        //    - WARN(약한 신호): angle >= WARN && dz >= LOW
        //    - CRIT(확실): angle >= CRIT && dz >= LOW
        //    - DZ-ONLY: dz >= HIGH (angle 무관 강제 판정)
        // ─────────────────────────────────────
        double dzHeadSh = headCenter[2] - shoulderCenter[2];   // +면 머리가 더 "멀리"(카메라에서 먼), -면 "가까이(전방)"
        double absDzForForward = shoulderCenter[2] - headCenter[2]; // 양수 -> 머리가 앞으로 나옴(카메라 쪽)
        double dyHeadSh = headCenter[1] - shoulderCenter[1];

        r.forwardHeadAngle = degAtan2(dzHeadSh, dyHeadSh); // YZ plane

        boolean condWarn = (r.forwardHeadAngle >= THRESH_FHP_ANGLE_WARN) && (absDzForForward >= FHP_Z_GAP_MIN_LOW);
        boolean condCrit = (r.forwardHeadAngle >= THRESH_FHP_ANGLE_CRIT) && (absDzForForward >= FHP_Z_GAP_MIN_LOW);
        boolean condDzOnly = (absDzForForward >= FHP_Z_GAP_MIN_HIGH);

        r.forwardHeadPosture = condWarn || condCrit || condDzOnly;

        if (r.forwardHeadPosture) {
            // severity override: dz-only 또는 crit인 경우 high
            String sev = (condDzOnly || condCrit) ? "high" : "moderate";
            Map<String, Object> extra = new LinkedHashMap<>();
            extra.put("cue", "턱을 뒤로 살짝 당겨 목 뒤가 길어지게 하고, 귀-어깨가 수직선상에 오도록 맞춰주세요");
            extra.put("ergonomics", "모니터 상단을 눈높이와 같거나 살짝 낮게 맞추고, 등받이에 등을 기대어 목이 앞으로 쏠리지 않게 하세요");
            extra.put("dz_gap", Math.round(absDzForForward)); // 디버그용 수치 포함
            extra.put("angle_value", Math.round(r.forwardHeadAngle * 10.0) / 10.0);
            extra.put("severity_override", sev);

            // threshold로 WARN값 전달(프론트/서버에서 WARN 기준으로 색/우선표시 처리 가능)
            r.addReason("FHP", "거북목", r.forwardHeadAngle, THRESH_FHP_ANGLE_WARN, extra);
        }
        log.debug("[FHP] angleYZ={}, absDzForForward={}, condWarn={}, condCrit={}, condDzOnly={}",
                r.forwardHeadAngle, absDzForForward, condWarn, condCrit, condDzOnly);

        // ─────────────────────────────────────
        // 2) 굽은 어깨 (Rounded Shoulders): 임계 상향
        // ─────────────────────────────────────
        double[] earCenter = avg(pts.get("le_ear"), pts.get("re_ear"));
        double[] vEarToSh  = sub(shoulderCenter, earCenter);
        r.roundedShoulderAngle = degAtan2(vEarToSh[2], vEarToSh[1]); // YZ 평면
        r.roundedShoulders     = r.roundedShoulderAngle > THRESH_RSA_DEG;

        if (r.roundedShoulders) {
            r.addReason("RSA", "굽은 어깨", r.roundedShoulderAngle, THRESH_RSA_DEG, Map.of(
                    "cue", "어깨를 위로 올리지 말고 살짝 뒤로 당겨 가슴을 부드럽게 펼친 느낌을 유지하세요",
                    "ergonomics", "키보드/마우스를 몸 가까이 두고 팔꿈치는 몸통에 가깝게 90~110° 정도로 유지하세요"
            ));
        }
        log.debug("[RSA] angleYZ={}", r.roundedShoulderAngle);

        // ─────────────────────────────────────
        // 3) 등 굽음 (Kyphosis): 힙 결측이면 스킵
        //    torso 전방 굴곡(YZ)으로 해석
        // ─────────────────────────────────────
        if (!hipMissing) {
            double[] vHipToSh = sub(shoulderCenter, hipCenter);
            r.kyphosisAngle = degAtan2(vHipToSh[2], vHipToSh[1]); // 전/후 대 세로
            r.slouching     = r.kyphosisAngle > THRESH_KYPH_DEG;

            if (r.slouching) {
                r.addReason("KYP", "등 굽음", r.kyphosisAngle, THRESH_KYPH_DEG, Map.of(
                        "cue", "배꼽 위쪽(명치)와 골반을 같은 기둥에 세운 느낌으로 앉고, 흉곽을 살짝 위로 들어 올리세요",
                        "ergonomics", "등받이에 허리를 확실히 지지하고 엉덩이를 등받이 깊숙이 붙여 척추가 세워지게 하세요"
                ));
            }
            log.debug("[KYP] torsoAngleYZ={}, hipMissing={}", r.kyphosisAngle, false);

            // ─────────────────────────────────
            // 4) 좌우 기울임 (Side Lean): 힙 결측이면 스킵
            //    좌우(X) 대 세로(Y)
            // ─────────────────────────────────
            double lateral = vHipToSh[0], vertical = vHipToSh[1];
            r.sideLeanAngle = degAtan2(lateral, vertical);
            r.sideLeaning   = r.sideLeanAngle > THRESH_SIDE_DEG ? (lateral > 0 ? "right" : "left") : null;

            if (r.sideLeaning != null) {
                String dirKo = "left".equals(r.sideLeaning) ? "좌측" : "우측";
                r.addReason("SIDE", dirKo + " 기울임", r.sideLeanAngle, THRESH_SIDE_DEG, Map.of(
                        "direction", r.sideLeaning,
                        "cue", dirKo + "으로 몸이 기울었습니다. 양 엉덩이뼈에 체중을 균등히 싣고, 가슴뼈가 배꼽 위에 오도록 정렬하세요",
                        "ergonomics", "의자 중앙에 앉고 팔걸이 높이를 좌우 동일하게 맞춰 골반이 한쪽으로 밀리지 않게 하세요"
                ));
            }
            log.debug("[SIDE] angleXY={}, leaning={}", r.sideLeanAngle, r.sideLeaning);
        } else {
            log.debug("[KYP/SIDE] skipped due to missing hips (le_hip & re_hip are zero)");
        }

        log.info("PostureResult => FHP:{}, RSA:{}, KYP:{}, SIDE:{}, Valid:{}",
                r.forwardHeadAngle, r.roundedShoulderAngle, r.kyphosisAngle, r.sideLeanAngle, r.isValidPosture());

        return r;
    }
}