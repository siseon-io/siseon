package siseon.backend.repository.batch;

import siseon.backend.domain.batch.PostureStats;
import org.springframework.data.jpa.repository.JpaRepository;
import java.util.Optional;

public interface PostureStatsRepository extends JpaRepository<PostureStats,Long> {
    Optional<PostureStats> findByProfileIdAndSlotIndex(Long profileId, int slotIndex);
}