package siseon.backend.repository.batch;

import org.springframework.data.domain.Pageable;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;
import siseon.backend.domain.batch.PostureStats;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

@Repository
public interface PostureStatsRepository extends JpaRepository<PostureStats, Long> {

    List<PostureStats> findByStartAtBetween(LocalDateTime from, LocalDateTime to);

    List<PostureStats> findByProfileIdAndStartAtBetween(
            Long profileId, LocalDateTime from, LocalDateTime to
    );

    Optional<PostureStats> findTopByProfileIdOrderByStartAtDesc(Long profileId);

    // 기존
    List<PostureStats> findByProfileId(Long profileId, Pageable pageable);

    // ✅ BAD: 미처리 + (옵션) 시간 이후 + 페이지/정렬은 Pageable에서
    List<PostureStats> findByProfileIdAndProcessedBadFalse(Long profileId, Pageable pageable);
    List<PostureStats> findByProfileIdAndEndAtAfterAndProcessedBadFalse(
            Long profileId, LocalDateTime endAt, Pageable pageable);

    // ✅ GOOD: 미처리 + (옵션) 시간 이후 + 페이지/정렬은 Pageable에서
    List<PostureStats> findByProfileIdAndProcessedGoodFalse(Long profileId, Pageable pageable);
    List<PostureStats> findByProfileIdAndEndAtAfterAndProcessedGoodFalse(
            Long profileId, LocalDateTime endAt, Pageable pageable);
}