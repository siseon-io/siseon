package siseon.backend.scheduler;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.scheduling.annotation.EnableScheduling;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;
import siseon.backend.domain.batch.PostureStats;
import siseon.backend.domain.main.Profile;
import siseon.backend.repository.batch.PostureStatsRepository;
import siseon.backend.repository.main.ProfileRepository;
import siseon.backend.service.PushNotificationService;

import java.time.LocalDateTime;
import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@Slf4j
@Component
@RequiredArgsConstructor
@EnableScheduling
public class PresetPushScheduler {

    private final PostureStatsRepository statsRepo;
    private final ProfileRepository    profileRepo;
    private final PushNotificationService pushService;

    /**
     * 매분 실행: 최근 30분치 stats 가 “유사”하면(variation ≤ delta) 푸시
     */
    @Scheduled(fixedDelay = 60_000)
    public void pushPresetSuggestions() {
        LocalDateTime now   = LocalDateTime.now();
        LocalDateTime since = now.minusMinutes(30);
        double delta = 5.0;  // x,y,z 축별 variation 허용치

        // 1) 지난 30분간 생성된 stats 모두 조회, 프로필별 그룹화
        Map<Long, List<PostureStats>> grouped = statsRepo
                .findByStartAtBetween(since, now)
                .stream()
                .collect(Collectors.groupingBy(PostureStats::getProfileId));

        grouped.forEach((profileId, statsList) -> {
            // 충분한 데이터(최소 30개)가 쌓였는지 확인
            if (statsList.size() < 30) return;

            // 2) 각 축별 max-min 계산
            DoubleSummaryStatistics xs = statsList.stream()
                    .map(PostureStats::getMonitorCoord)
                    .mapToDouble(m -> ((Number)m.get("avgx")).doubleValue())
                    .summaryStatistics();
            DoubleSummaryStatistics ys = statsList.stream()
                    .map(PostureStats::getMonitorCoord)
                    .mapToDouble(m -> ((Number)m.get("avgy")).doubleValue())
                    .summaryStatistics();
            DoubleSummaryStatistics zs = statsList.stream()
                    .map(PostureStats::getMonitorCoord)
                    .mapToDouble(m -> ((Number)m.get("avgz")).doubleValue())
                    .summaryStatistics();

            double diffX = xs.getMax() - xs.getMin();
            double diffY = ys.getMax() - ys.getMin();
            double diffZ = zs.getMax() - zs.getMin();

            // 3) variation 모두 delta 이하일 때만 푸시
            if (diffX <= delta && diffY <= delta && diffZ <= delta) {
                Profile profile = profileRepo.findById(profileId)
                        .orElseThrow(() -> new IllegalStateException("프로필 없음: " + profileId));

                String title = "프리셋 저장 제안";
                String body  = "유사한 자세가 30분 이상 지속되었습니다. 프리셋으로 저장하시겠어요?";

                // 4) data 페이로드에 profileId + 버튼 액션 정보 담기
                Map<String, String> data = Map.of(
                        "profileId",    profileId.toString(),
                        "actionConfirm","confirm",
                        "actionCancel", "cancel"
                );

                pushService.sendPushAsync(
                        profile.getFcmToken(),
                        title,
                        body,
                        data
                );
                log.info("액션 버튼 포함 푸시 전송 – profileId={}, diffX={}, diffY={}, diffZ={}",
                        profileId, diffX, diffY, diffZ);
            }
        });
    }
}
