package siseon.backend.repository.batch;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Modifying;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;
import org.springframework.transaction.annotation.Transactional;
import siseon.backend.domain.batch.PostureStats;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

@Repository
public interface PostureStatsRepository extends JpaRepository<PostureStats, Long> {

    // 기본 조회
    List<PostureStats> findByStartAtBetween(LocalDateTime from, LocalDateTime to);
    List<PostureStats> findByProfileIdAndStartAtBetween(Long profileId, LocalDateTime from, LocalDateTime to);
    Optional<PostureStats> findTopByProfileIdOrderByStartAtDesc(Long profileId);
    List<PostureStats> findByProfileId(Long profileId, org.springframework.data.domain.Pageable pageable);
    List<PostureStats> findByProfileIdAndProcessedBadFalse(Long profileId, org.springframework.data.domain.Pageable pageable);
    List<PostureStats> findByProfileIdAndEndAtAfterAndProcessedBadFalse(Long profileId, LocalDateTime endAt, org.springframework.data.domain.Pageable pageable);
    List<PostureStats> findByProfileIdAndProcessedGoodFalse(Long profileId, org.springframework.data.domain.Pageable pageable);
    List<PostureStats> findByProfileIdAndEndAtAfterAndProcessedGoodFalse(Long profileId, LocalDateTime endAt, org.springframework.data.domain.Pageable pageable);

    // 하루 경계 조회: start <= startAt < end
    @Query("SELECT s FROM PostureStats s WHERE s.startAt >= :start AND s.startAt < :end")
    List<PostureStats> findAllByStartAtInDay(@Param("start") LocalDateTime start,
                                             @Param("end")   LocalDateTime end);

    // 하루 경계 벌크 삭제
    @Modifying(clearAutomatically = true, flushAutomatically = true)
    @Transactional
    @Query("DELETE FROM PostureStats s WHERE s.startAt >= :start AND s.startAt < :end")
    int deleteAllByStartAtInDay(@Param("start") LocalDateTime start,
                                @Param("end")   LocalDateTime end);
}