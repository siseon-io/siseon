package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import siseon.backend.domain.Preset;
import siseon.backend.domain.Profile;
import siseon.backend.dto.PresetRequest;
import siseon.backend.dto.PresetResponse;
import siseon.backend.repository.PresetRepository;
import siseon.backend.repository.ProfileRepository;

import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
public class PresetService {

    private final PresetRepository presetRepository;
    private final ProfileRepository profileRepository;

    public PresetResponse createPreset(PresetRequest request) {
        Profile profile = profileRepository.findById(request.getProfileId())
                .orElseThrow(() -> new IllegalArgumentException("해당 프로필이 존재하지 않습니다."));

        Preset preset = Preset.builder()
                .deviceId(request.getDeviceId())
                .profile(profile)
                .name(request.getName())
                .position(request.getPosition())
                .build();

        return PresetResponse.fromEntity(presetRepository.save(preset));
    }

    public List<PresetResponse> getPresetsByProfile(Long profileId) {
        return presetRepository.findByProfile_Id(profileId).stream()
                .map(PresetResponse::fromEntity)
                .collect(Collectors.toList());
    }

    public PresetResponse updatePreset(Long presetId, PresetRequest request) {
        Preset preset = presetRepository.findById(presetId)
                .orElseThrow(() -> new IllegalArgumentException("해당 프리셋이 존재하지 않습니다."));

        Profile profile = profileRepository.findById(request.getProfileId())
                .orElseThrow(() -> new IllegalArgumentException("해당 프로필이 존재하지 않습니다."));

        preset.setDeviceId(request.getDeviceId());
        preset.setProfile(profile);
        preset.setName(request.getName());
        preset.setPosition(request.getPosition());

        return PresetResponse.fromEntity(presetRepository.save(preset));
    }

    public void deletePreset(Long presetId) {
        Preset preset = presetRepository.findById(presetId)
                .orElseThrow(() -> new IllegalArgumentException("해당 프리셋이 존재하지 않습니다."));
        presetRepository.delete(preset);
    }
}