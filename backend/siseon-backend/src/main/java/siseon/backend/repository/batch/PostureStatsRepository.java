package siseon.backend.repository.batch;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;
import siseon.backend.domain.batch.PostureStats;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

@Repository
public interface PostureStatsRepository extends JpaRepository<PostureStats, Long> {

    // 지정된 기간(startAt between) 내의 모든 통계 조회
    List<PostureStats> findByStartAtBetween(LocalDateTime from, LocalDateTime to);


    // profileId + 지정된 기간(startAt between) 내의 통계 조회
    List<PostureStats> findByProfileIdAndStartAtBetween(
            Long profileId,
            LocalDateTime from,
            LocalDateTime to
    );

    // profileId 에 대해 startAt 기준 가장 최신 한 건 조회
    Optional<PostureStats> findTopByProfileIdOrderByStartAtDesc(Long profileId);
}
