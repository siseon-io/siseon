package siseon.backend.repository.main;

import org.springframework.data.jpa.repository.JpaRepository;
import siseon.backend.domain.main.Preset;

import java.util.List;

public interface PresetRepository extends JpaRepository<Preset, Long> {
    List<Preset> findByProfile_Id(Long profileId);
}
