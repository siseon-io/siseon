package siseon.backend.repository;

import org.springframework.data.jpa.repository.JpaRepository;
import siseon.backend.domain.Preset;

import java.util.List;

public interface PresetRepository extends JpaRepository<Preset, Long> {
    List<Preset> findByProfile_Id(Long profileId);
}
