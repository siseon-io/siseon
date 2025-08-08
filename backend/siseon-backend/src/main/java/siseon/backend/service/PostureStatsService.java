// src/main/java/siseon/backend/service/PostureStatsService.java
package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import siseon.backend.domain.batch.PostureStats;
import siseon.backend.dto.PostureStatsResponse;
import siseon.backend.repository.batch.PostureStatsRepository;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.time.ZoneId;
import java.util.List;

import static java.time.temporal.TemporalAdjusters.firstDayOfMonth;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class PostureStatsService {

    private final PostureStatsRepository postureStatsRepository;

    /**
     * profileId, period, from/to 조건에 따라 posture stats 조회
     *
     * @param profileId 조회할 프로필 ID
     * @param period    "daily", "weekly", "monthly" 중 하나
     * @param from      시작 시각 (nullable)
     * @param to        종료 시각 (nullable)
     */
    public List<PostureStatsResponse> getStats(
            Long profileId,
            String period,
            LocalDateTime from,
            LocalDateTime to
    ) {
        // 1) 기준 시각 (Asia/Seoul)
        ZoneId zone = ZoneId.of("Asia/Seoul");
        LocalDateTime now = LocalDateTime.now(zone);

        // 2) from/to 모두 없으면 period 기준 기본 범위 계산
        if (from == null && to == null) {
            switch (period) {
                case "daily":
                    from = now.toLocalDate().atStartOfDay();
                    to   = now.toLocalDate().atTime(LocalTime.MAX);
                    break;
                case "weekly":
                    from = now.minusDays(6).toLocalDate().atStartOfDay();
                    to   = now.toLocalDate().atTime(LocalTime.MAX);
                    break;
                case "monthly":
                    // 최근 12개월 (현재 월 포함)
                    from = now.with(firstDayOfMonth())
                            .minusMonths(11)
                            .with(LocalTime.MIN);
                    to   = now.toLocalDate().atTime(LocalTime.MAX);
                    break;
                default:
                    throw new IllegalArgumentException("Invalid period: " + period);
            }
        }
        // 3) from만 있고 to가 없으면 from 당일 종일
        else if (from != null && to == null) {
            to = from.toLocalDate().atTime(LocalTime.MAX);
        }
        // 4) to만 있고 from이 없으면 to 당일 00:00~to
        else if (from == null && to != null) {
            from = to.toLocalDate().atStartOfDay();
            to   = to.toLocalDate().atTime(LocalTime.MAX);
        }
        // 5) 둘 다 있으면 to 당일 종일 포함
        else {
            to = to.toLocalDate().atTime(LocalTime.MAX);
        }

        // 6) 실제 조회 및 매핑
        return postureStatsRepository
                .findByProfileIdAndStartAtBetween(profileId, from, to)
                .stream()
                .map(PostureStatsResponse::from)
                .toList();
    }
}