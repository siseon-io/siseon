package siseon.backend.scheduler;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.data.domain.PageRequest;
import org.springframework.data.domain.Sort;
import org.springframework.scheduling.annotation.EnableScheduling;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;
import siseon.backend.domain.batch.NotificationLog;
import siseon.backend.domain.batch.PostureStats;
import siseon.backend.domain.main.Profile;
import siseon.backend.repository.batch.NotificationLogRepository;
import siseon.backend.repository.batch.PostureStatsRepository;
import siseon.backend.repository.main.ProfileRepository;
import siseon.backend.service.PushNotificationService;

import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.temporal.ChronoUnit;
import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.Map;

@Slf4j
@Component
@EnableScheduling
@RequiredArgsConstructor
public class PresetPushScheduler {

    private static final ZoneId KST = ZoneId.of("Asia/Seoul");

    private final PostureStatsRepository statsRepo;
    private final ProfileRepository profileRepo;
    private final PushNotificationService pushService;
    private final NotificationLogRepository notifRepo;

    // ====== 운영 파라미터 ======
    private static final int WINDOW_SIZE         = 30;  // 최근 30개(≈30분)
    private static final double RATIO_THRESHOLD  = 0.8; // 80% 기준
    private static final int BAD_COOLDOWN_MIN    = 30;  // BAD_POSTURE 쿨다운
    private static final int PRESET_COOLDOWN_MIN = 60;  // PRESET_SUGGEST 쿨다운
    private static final int FRESHNESS_MAX_MIN   = 35;  // 신선도 허용(오차 포함 35분)
    private static final double SIMILAR_DELTA    = 5.0; // avgx/avgy/avgz 변동폭 허용치

    /**
     * 매 분 실행. 분 경계 커밋 충돌 방지를 위해 5초 지연을 둔다.
     * fixedDelay 이므로 이전 실행 종료 후 60초 뒤 재실행.
     */
    @Scheduled(initialDelayString = "PT5S", fixedDelayString = "PT60S")
    public void checkRecentSamples() {
        LocalDateTime nowTrunc = LocalDateTime.now(KST).truncatedTo(ChronoUnit.MINUTES);

        for (Profile p : profileRepo.findAll()) {
            final Long profileId = p.getId();
            final String token   = p.getFcmToken();
            if (token == null || token.isBlank()) continue;

            try {
                handleBadWindow(profileId, token, nowTrunc);
                handleGoodPresetWindow(profileId, token, nowTrunc);
            } catch (Exception e) {
                log.error("PresetPushScheduler 처리 중 오류 - profileId={}", profileId, e);
            }
        }
    }

    /**
     * 최근 30개 중 80% 이상 BAD → BAD_POSTURE 경고
     * - 미처리(processedBad=false) 중 최근 30개 사용
     * - 전부 신선도 기준(<= 35분) 통과
     * - 쿨다운 30분
     * - 사용한 30개 모두 processedBad=true 마킹
     */
    private void handleBadWindow(Long profileId, String fcmToken, LocalDateTime nowTrunc) {
        List<PostureStats> lastN = statsRepo.findByProfileIdAndProcessedBadFalse(
                profileId,
                PageRequest.of(0, WINDOW_SIZE, Sort.by(Sort.Direction.DESC, "endAt"))
        );
        if (lastN.size() < WINDOW_SIZE) return;

        boolean fresh = lastN.stream().allMatch(ps ->
                ps.getEndAt() != null && !ps.getEndAt().isBefore(nowTrunc.minusMinutes(FRESHNESS_MAX_MIN))
        );
        if (!fresh) return;

        long badCount = lastN.stream()
                .filter(ps -> Boolean.FALSE.equals(ps.getValidPosture()))
                .count();
        if (badCount < Math.ceil(RATIO_THRESHOLD * WINDOW_SIZE)) return;

        if (!cooldownOver(profileId, "BAD_POSTURE", BAD_COOLDOWN_MIN)) return;

        pushService.sendPushAsync(
                fcmToken,
                "자세 교정이 필요해요",
                "최근 30분 동안 좋지 않은 자세가 많이 감지되었어요. 자세를 바꿔볼까요?",
                Map.of(
                        "subtype", "bad_posture",
                        "profileId", profileId.toString()
                )
        );
        logSent(profileId, "BAD_POSTURE");
        log.info("BAD_POSTURE sent pid={}, badCount={}/{}", profileId, badCount, WINDOW_SIZE);

        lastN.forEach(ps -> ps.setProcessedBad(true));
        statsRepo.saveAll(lastN);
    }

    /**
     * 최근 30개 중 80% 이상 GOOD + 유사자세 → PRESET_SUGGEST
     * - 미처리(processedGood=false) 중 최근 30개 사용
     * - 전부 신선도 기준(<= 35분) 통과
     * - 변동폭(avgx/avgy/avgz)의 max-min <= SIMILAR_DELTA
     * - 쿨다운 60분
     * - 사용한 30개 모두 processedGood=true 마킹
     */
    private void handleGoodPresetWindow(Long profileId, String fcmToken, LocalDateTime nowTrunc) {
        List<PostureStats> lastN = statsRepo.findByProfileIdAndProcessedGoodFalse(
                profileId,
                PageRequest.of(0, WINDOW_SIZE, Sort.by(Sort.Direction.DESC, "endAt"))
        );
        if (lastN.size() < WINDOW_SIZE) return;

        boolean fresh = lastN.stream().allMatch(ps ->
                ps.getEndAt() != null && !ps.getEndAt().isBefore(nowTrunc.minusMinutes(FRESHNESS_MAX_MIN))
        );
        if (!fresh) return;

        long goodCount = lastN.stream()
                .filter(ps -> Boolean.TRUE.equals(ps.getValidPosture()))
                .count();
        if (goodCount < Math.ceil(RATIO_THRESHOLD * WINDOW_SIZE)) return;

        // 유사자세(변동폭)
        DoubleSummaryStatistics xs = lastN.stream()
                .map(PostureStats::getMonitorCoord)
                .mapToDouble(m -> safeNum(m, "avgx"))
                .summaryStatistics();
        DoubleSummaryStatistics ys = lastN.stream()
                .map(PostureStats::getMonitorCoord)
                .mapToDouble(m -> safeNum(m, "avgy"))
                .summaryStatistics();
        DoubleSummaryStatistics zs = lastN.stream()
                .map(PostureStats::getMonitorCoord)
                .mapToDouble(m -> safeNum(m, "avgz"))
                .summaryStatistics();

        double dx = xs.getMax() - xs.getMin();
        double dy = ys.getMax() - ys.getMin();
        double dz = zs.getMax() - zs.getMin();
        boolean similar = (dx <= SIMILAR_DELTA && dy <= SIMILAR_DELTA && dz <= SIMILAR_DELTA);
        if (!similar) return;

        if (!cooldownOver(profileId, "PRESET_SUGGEST", PRESET_COOLDOWN_MIN)) return;

        pushService.sendPushAsync(
                fcmToken,
                "이 자세를 프리셋으로 저장할까요?",
                "최근 30분 동안 좋은 자세를 안정적으로 유지하고 있어요. 프리셋으로 저장해두면 편하게 불러올 수 있어요.",
                Map.of(
                        "subtype", "preset_suggest",
                        "profileId", profileId.toString(),
                        "actionConfirm", "confirm",
                        "actionCancel",  "cancel"
                )
        );
        logSent(profileId, "PRESET_SUGGEST");
        log.info("PRESET_SUGGEST sent pid={}, dx={}, dy={}, dz={}", profileId, dx, dy, dz);

        lastN.forEach(ps -> ps.setProcessedGood(true));
        statsRepo.saveAll(lastN);
    }

    // ==== 공통 유틸 ====
    private static double safeNum(Map<String, Object> m, String key) {
        if (m == null) return 0.0;
        Object v = m.get(key);
        return (v instanceof Number) ? ((Number) v).doubleValue() : 0.0;
    }

    private boolean cooldownOver(Long pid, String type, int minutes) {
        var recent = notifRepo.findRecent(pid, type, PageRequest.of(0, 1));
        if (recent == null || recent.isEmpty()) return true;
        return recent.get(0).getSentAt().isBefore(LocalDateTime.now(KST).minusMinutes(minutes));
    }

    private void logSent(Long pid, String type) {
        notifRepo.save(NotificationLog.builder()
                .profileId(pid)
                .type(type)
                .sentAt(LocalDateTime.now(KST))
                .build());
    }
}