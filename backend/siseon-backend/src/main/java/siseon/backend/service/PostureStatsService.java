// src/main/java/siseon/backend/service/PostureStatsService.java
package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import siseon.backend.domain.batch.PostureStats;
import siseon.backend.dto.PostureStatsResponse;
import siseon.backend.repository.batch.PostureStatsRepository;

import java.time.LocalDateTime;
import java.util.List;

@Service
@RequiredArgsConstructor
@Transactional(readOnly = true)
public class PostureStatsService {

    private final PostureStatsRepository postureStatsRepository;

    public List<PostureStatsResponse> getStatsByPeriod(Long profileId, String period) {
        LocalDateTime now   = LocalDateTime.now();
        LocalDateTime start = switch (period) {
            case "daily"   -> now.minusDays(1);
            case "weekly"  -> now.minusWeeks(1);
            case "monthly" -> now.minusMonths(1);
            default -> throw new IllegalArgumentException("Invalid period: " + period);
        };

        return postureStatsRepository
                .findByProfileIdAndStartAtBetween(profileId, start, now)
                .stream()
                .map(PostureStatsResponse::from)
                .toList();
    }

    public List<PostureStatsResponse> getStatsByRange(
            Long profileId,
            LocalDateTime from,
            LocalDateTime to
    ) {
        return postureStatsRepository
                .findByProfileIdAndStartAtBetween(profileId, from, to)
                .stream()
                .map(PostureStatsResponse::from)
                .toList();
    }
}
