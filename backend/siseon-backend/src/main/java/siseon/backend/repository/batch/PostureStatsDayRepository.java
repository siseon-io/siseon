package siseon.backend.repository.batch;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import siseon.backend.domain.batch.PostureStatsDay;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;

public interface PostureStatsDayRepository extends JpaRepository<PostureStatsDay, Long> {

    @Query("""
      SELECT ps.profileId as profileId,
             SUM(CASE WHEN ps.validPosture = TRUE  THEN 1 ELSE 0 END) as goodCount,
             SUM(CASE WHEN ps.validPosture = FALSE THEN 1 ELSE 0 END) as badCount
      FROM PostureStats ps
      WHERE ps.startAt >= :start AND ps.endAt < :end
      GROUP BY ps.profileId
    """)
    List<Object[]> aggregateByDay(@Param("start") LocalDateTime start, @Param("end") LocalDateTime end);

    boolean existsByProfileIdAndStatDate(Long profileId, LocalDate statDate);

    // 조회용 메서드
    List<PostureStatsDay> findByProfileIdAndStatDateBetween(Long profileId, LocalDate from, LocalDate to);
    List<PostureStatsDay> findByProfileIdAndStatDateGreaterThanEqual(Long profileId, LocalDate from);
    List<PostureStatsDay> findByProfileIdAndStatDateLessThanEqual(Long profileId, LocalDate to);
    List<PostureStatsDay> findByProfileIdOrderByStatDateDesc(Long profileId);
}