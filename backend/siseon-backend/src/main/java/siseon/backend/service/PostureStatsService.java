package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import siseon.backend.domain.batch.PostureStats;
import siseon.backend.domain.batch.PostureStatsDay;
import siseon.backend.dto.PostureStatsDayResponse;
import siseon.backend.dto.PostureStatsResponse;
import siseon.backend.repository.batch.PostureStatsDayRepository;
import siseon.backend.repository.batch.PostureStatsRepository;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.time.ZoneId;
import java.util.List;

import static java.time.temporal.TemporalAdjusters.firstDayOfMonth;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class PostureStatsService {

    private final PostureStatsRepository postureStatsRepository;       // minute 데이터
    private final PostureStatsDayRepository postureStatsDayRepository; // day 집계 데이터

    /**
     * 분(minute) 단위 posture stats 조회
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
        ZoneId zone = ZoneId.of("Asia/Seoul");
        LocalDateTime now = LocalDateTime.now(zone);

        // 기본 범위 계산
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
                    from = now.with(firstDayOfMonth()).minusMonths(11).with(LocalTime.MIN);
                    to   = now.toLocalDate().atTime(LocalTime.MAX);
                    break;
                default:
                    throw new IllegalArgumentException("Invalid period: " + period);
            }
        } else if (from != null && to == null) {
            to = from.toLocalDate().atTime(LocalTime.MAX);
        } else if (from == null) { // to != null
            from = to.toLocalDate().atStartOfDay();
            to   = to.toLocalDate().atTime(LocalTime.MAX);
        } else {
            to = to.toLocalDate().atTime(LocalTime.MAX);
        }

        // 조회 및 매핑
        return postureStatsRepository
                .findByProfileIdAndStartAtBetween(profileId, from, to)
                .stream()
                .map(PostureStatsResponse::from) // 기존 DTO 정적 팩토리 사용
                .toList();
    }

    /**
     * 일(day) 단위 집계 조회: posture_stats_day
     * 컨트롤러에서 period=day 일 때 이 메서드를 호출.
     */
    public List<PostureStatsDayResponse> getDailyStats(
            Long profileId,
            LocalDate fromDate,
            LocalDate toDate
    ) {
        List<PostureStatsDay> rows;
        if (fromDate != null && toDate != null) {
            rows = postureStatsDayRepository.findByProfileIdAndStatDateBetween(profileId, fromDate, toDate);
        } else if (fromDate != null) {
            rows = postureStatsDayRepository.findByProfileIdAndStatDateGreaterThanEqual(profileId, fromDate);
        } else if (toDate != null) {
            rows = postureStatsDayRepository.findByProfileIdAndStatDateLessThanEqual(profileId, toDate);
        } else {
            rows = postureStatsDayRepository.findByProfileIdOrderByStatDateDesc(profileId);
        }

        return rows.stream()
                .map(this::toDayDto)
                .toList();
    }

    // --- private mapper ---
    private PostureStatsDayResponse toDayDto(PostureStatsDay d) {
        return PostureStatsDayResponse.builder()
                .profileId(d.getProfileId())
                .statDate(d.getStatDate())
                .goodCount(d.getGoodCount())
                .badCount(d.getBadCount())
                .totalCount(d.getGoodCount() + d.getBadCount()) // 엔티티에는 없음 → 계산
                .build();
    }
}
