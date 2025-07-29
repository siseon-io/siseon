package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
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
@Transactional
public class PresetService {

    private final ProfileRepository profileRepository;
    private final PresetRepository presetRepository;

    /** 프리셋 생성 */
    public PresetResponse createPreset(PresetRequest request) {
        Profile profile = profileRepository.findById(request.getProfileId())
                .orElseThrow(() -> new IllegalArgumentException("해당 프로필이 존재하지 않습니다."));

        Preset preset = Preset.builder()
                .deviceId(request.getDeviceId())
                .name(request.getName())
                .position(request.getPosition())
                .build();

        // 편의 메서드로 양방향 연관관계 설정
        profile.addPreset(preset);
        profileRepository.save(profile);  // cascade로 preset 저장

        return PresetResponse.fromEntity(preset);
    }

    /** 프로필별 프리셋 조회 */
    public List<PresetResponse> getPresetsByProfile(Long profileId) {
        return presetRepository.findByProfile_Id(profileId).stream()
                .map(PresetResponse::fromEntity)
                .collect(Collectors.toList());
    }

    /** 프리셋 수정 */
    public PresetResponse updatePreset(Long presetId, PresetRequest request) {
        Preset preset = presetRepository.findById(presetId)
                .orElseThrow(() -> new IllegalArgumentException("해당 프리셋이 존재하지 않습니다."));

        preset.setDeviceId(request.getDeviceId());
        preset.setName(request.getName());
        preset.setPosition(request.getPosition());
        // 프로필 변경이 필요할 경우:
        if (!preset.getProfile().getId().equals(request.getProfileId())) {
            // 이전 프로필에서 제거
            preset.getProfile().removePreset(preset);
            // 새 프로필에 추가
            Profile newProfile = profileRepository.findById(request.getProfileId())
                    .orElseThrow(() -> new IllegalArgumentException("해당 프로필이 존재하지 않습니다."));
            newProfile.addPreset(preset);
            profileRepository.save(newProfile);
            return PresetResponse.fromEntity(preset);
        }

        // 동일 프로필 내 수정
        return PresetResponse.fromEntity(presetRepository.save(preset));
    }

    /** 프리셋 삭제 */
    public void deletePreset(Long presetId) {
        Preset preset = presetRepository.findById(presetId)
                .orElseThrow(() -> new IllegalArgumentException("해당 프리셋이 존재하지 않습니다."));

        Profile profile = preset.getProfile();
        // 편의 메서드로 양방향 연관관계 해제
        profile.removePreset(preset);
        profileRepository.save(profile);  // cascade로 preset 삭제
    }
}