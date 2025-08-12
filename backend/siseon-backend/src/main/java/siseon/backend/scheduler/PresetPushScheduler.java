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
import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.Map;

@Slf4j
@Component
@EnableScheduling
@RequiredArgsConstructor
public class PresetPushScheduler {

    private final PostureStatsRepository statsRepo;
    private final ProfileRepository profileRepo;
    private final PushNotificationService pushService;
    private final NotificationLogRepository notifRepo;

    // ====== 파라미터 ======
    private static final int BAD_COOLDOWN_MIN    = 10; // BAD_POSTURE: 30분 이내 재발송 금지, 테스트로 10분으로 변경
    private static final int PRESET_COOLDOWN_MIN = 30; // PRESET_SUGGEST: 60분 이내 재발송 금지, 테스트로 30분으로 변경
    private static final double SIMILAR_DELTA    = 5.0; // 유사자세 변동폭 허용치 (avgx/avgy/avgz)

    @Scheduled(fixedDelay = 60_000) // 1분마다 최근 샘플 기반 판정
    public void checkRecentSamples() {
        for (Profile p : profileRepo.findAll()) {
            final Long profileId = p.getId();
            final String token   = p.getFcmToken();
            if (token == null || token.isBlank()) continue;

            try {
                handleBad3(profileId, token);   // 최근 1개 bad → 경고(테스트)
                handleGood6(profileId, token);  // 최근 3개 good + 유사자세 → 프리셋 제안(테스트)
            } catch (Exception e) {
                log.error("PresetPushScheduler 처리 중 오류 - profileId={}", profileId, e);
            }
        }
    }

    // 최근 1개가 bad → BAD_POSTURE 경고(테스트)
    private void handleBad3(Long profileId, String fcmToken) {
        // ✅ 미처리(processedBad=false) + 최근 1개
        List<PostureStats> last = statsRepo.findByProfileIdAndProcessedBadFalse(
                profileId,
                PageRequest.of(0, 1, Sort.by(Sort.Direction.DESC, "endAt"))
        );
        if (last.isEmpty()) return;

        boolean allBad = last.stream()
                .allMatch(ps -> Boolean.FALSE.equals(ps.getValidPosture())); // null은 제외됨

        if (!allBad) return;
        if (!cooldownOver(profileId, "BAD_POSTURE", BAD_COOLDOWN_MIN)) return;

        pushService.sendPushAsync(
                fcmToken,
                "자세 교정이 필요해요",
                "최근 30분 동안 좋지 않은 자세가 연속으로 감지됐어요. 자세를 바꿔볼까요?",
                Map.of(
                        "subtype", "bad_posture",
                        "profileId", profileId.toString()
                )
        );
        logSent(profileId, "BAD_POSTURE");
        log.info("⚠️ BAD_POSTURE sent pid={}, lastEndAt={}", profileId, last.get(0).getEndAt());

        // ✅ 사용한 샘플 마킹
        last.forEach(ps -> ps.setProcessedBad(true));
        statsRepo.saveAll(last);
    }

    // 최근 3개 모두 good + 유사자세 → PRESET_SUGGEST 제안(테스트)
    private void handleGood6(Long profileId, String fcmToken) {
        // ✅ 미처리(processedGood=false) + 최근 3개
        List<PostureStats> last = statsRepo.findByProfileIdAndProcessedGoodFalse(
                profileId,
                PageRequest.of(0, 3, Sort.by(Sort.Direction.DESC, "endAt"))
        );
        if (last.size() < 3) return;

        boolean allGood = last.stream()
                .allMatch(ps -> Boolean.TRUE.equals(ps.getValidPosture())); // null은 제외됨
        if (!allGood) return;

        // 유사자세(변동폭) 체크: monitorCoord.avgx/avgy/avgz의 max-min <= SIMILAR_DELTA
        DoubleSummaryStatistics xs = last.stream()
                .map(PostureStats::getMonitorCoord)
                .mapToDouble(m -> ((Number) m.get("avgx")).doubleValue())
                .summaryStatistics();
        DoubleSummaryStatistics ys = last.stream()
                .map(PostureStats::getMonitorCoord)
                .mapToDouble(m -> ((Number) m.get("avgy")).doubleValue())
                .summaryStatistics();
        DoubleSummaryStatistics zs = last.stream()
                .map(PostureStats::getMonitorCoord)
                .mapToDouble(m -> ((Number) m.get("avgz")).doubleValue())
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
                "1시간 동안 좋은 자세를 유지하고 있어요. 프리셋으로 저장하면 다음에 더 편해져요!",
                Map.of(
                        "subtype", "preset_suggest",
                        "profileId", profileId.toString(),
                        "actionConfirm", "confirm",
                        "actionCancel",  "cancel"
                )
        );
        logSent(profileId, "PRESET_SUGGEST");
        log.info("✅ PRESET_SUGGEST sent pid={}, dx={}, dy={}, dz={}", profileId, dx, dy, dz);

        // ✅ 사용한 샘플 마킹
        last.forEach(ps -> ps.setProcessedGood(true));
        statsRepo.saveAll(last);
    }

    // ==== 쿨다운 체크/로그 ====
    private boolean cooldownOver(Long pid, String type, int minutes) {
        var recent = notifRepo.findRecent(pid, type, org.springframework.data.domain.PageRequest.of(0, 1));
        if (recent == null || recent.isEmpty()) return true;
        return recent.get(0).getSentAt().isBefore(LocalDateTime.now().minusMinutes(minutes));
    }

    private void logSent(Long pid, String type) {
        notifRepo.save(NotificationLog.builder()
                .profileId(pid)
                .type(type)
                .sentAt(LocalDateTime.now())
                .build());
    }
}