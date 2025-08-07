package siseon.backend.repository.main;

import org.springframework.data.jpa.repository.JpaRepository;
import siseon.backend.domain.main.Device;
import java.util.List;

public interface DeviceRepository extends JpaRepository<Device, Long> {
    List<Device> findByProfileId_Id(Long profileId);
    void deleteByProfileId_Id(Long profileId);
}
