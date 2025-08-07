package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import siseon.backend.domain.main.RawPosture;
import siseon.backend.dto.RawPostureRequest;
import siseon.backend.repository.main.RawPostureRepository;

@Service
@RequiredArgsConstructor
public class RawPostureService {

    private final RawPostureRepository rawPostureRepository;

    @Transactional
    public void saveRawPosture(RawPostureRequest req) {
        RawPosture entity = RawPosture.builder()
                .profileId(req.getProfileId())
                .monitorCoord(req.getMonitorCoord())
                .userCoord(req.getUserCoord())
                .build();
        rawPostureRepository.save(entity);
    }
}
