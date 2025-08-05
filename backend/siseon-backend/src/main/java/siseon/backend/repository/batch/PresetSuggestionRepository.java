package siseon.backend.repository.batch;

import siseon.backend.domain.batch.PresetSuggestion;
import org.springframework.data.jpa.repository.JpaRepository;
import java.util.List;

public interface PresetSuggestionRepository extends JpaRepository<PresetSuggestion,Long> {
    List<PresetSuggestion> findBySatisfiedFalse();
}