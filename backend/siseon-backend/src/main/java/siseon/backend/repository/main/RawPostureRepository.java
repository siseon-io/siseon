package siseon.backend.repository.main;

import siseon.backend.domain.main.RawPosture;
import org.springframework.data.jpa.repository.JpaRepository;
import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

public interface RawPostureRepository extends JpaRepository<RawPosture,Long> {
    long countByCollectedAtBetween(LocalDateTime start, LocalDateTime end);
    List<RawPosture> findByCollectedAtBetween(LocalDateTime start, LocalDateTime end);
    List<RawPosture> findByCollectedAtLessThanEqual(LocalDateTime end);
    Optional<RawPosture> findTopByProfileIdOrderByCollectedAtDesc(Long profileId);
}