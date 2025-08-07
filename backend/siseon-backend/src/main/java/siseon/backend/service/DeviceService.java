package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import siseon.backend.domain.main.Device;
import siseon.backend.domain.main.Profile;
import siseon.backend.repository.main.DeviceRepository;
import siseon.backend.repository.main.ProfileRepository;

import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
@Transactional
public class DeviceService {
    private final DeviceRepository deviceRepo;
    private final ProfileRepository profileRepo;

    public Device register(Long profileId, String serial) {
        Profile profile = profileRepo.findById(profileId)
                .orElseThrow(() -> new IllegalArgumentException("프로필이 없습니다: " + profileId));
        Device device = Device.builder()
                .profileId(profile)
                .serialNumber(serial)
                .build();
        return deviceRepo.save(device);
    }

    public List<String> findSerialsByProfile(Long profileId) {
        return deviceRepo
                .findByProfileId_Id(profileId)           // Repository 메서드 그대로 사용
                .stream()
                .map(Device::getSerialNumber)
                .collect(Collectors.toList());
    }
    public void deleteByProfileId(Long profileId) {
        deviceRepo.deleteByProfileId_Id(profileId);
    }
}
