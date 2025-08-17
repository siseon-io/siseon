package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import siseon.backend.domain.batch.PostureStats;
import siseon.backend.domain.main.Device;
import siseon.backend.domain.main.Preset;
import siseon.backend.domain.main.Profile;
import siseon.backend.domain.main.RawPosture;
import siseon.backend.dto.PresetRequest;
import siseon.backend.dto.PresetResponse;
import siseon.backend.dto.PresetCoordinate;
import siseon.backend.repository.batch.PostureStatsRepository;
import siseon.backend.repository.main.PresetRepository;
import siseon.backend.repository.main.ProfileRepository;
import siseon.backend.repository.main.RawPostureRepository;
import siseon.backend.repository.main.DeviceRepository;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
@Transactional
public class PresetService {

    private final ProfileRepository profileRepo;
    private final RawPostureRepository rawRepo;
    private final PostureStatsRepository statsRepo;
    private final PresetRepository presetRepo;
    private final DeviceRepository deviceRepo;

    /**
     * 수동 프리셋 저장
     * - 최신 RawPosture의 pose_data에서 좌표 추출
     */
    public PresetResponse createFromRaw(PresetRequest req) {
        Profile profile = profileRepo.findById(req.getProfileId())
                .orElseThrow(() -> new IllegalArgumentException("프로필이 없습니다: " + req.getProfileId()));

        RawPosture latestRaw = rawRepo.findTopByProfileIdOrderByCollectedAtDesc(req.getProfileId())
                .orElseThrow(() -> new IllegalStateException("raw_posture 데이터가 없습니다: " + req.getProfileId()));

        @SuppressWarnings("unchecked")
        Map<String, Map<String, Number>> poseData =
                (Map<String, Map<String, Number>>) latestRaw.getUserCoord().get("pose_data");

        Preset saved = presetRepo.save(
                Preset.builder()
                        .profile(profile)
                        .name(req.getName())
                        .lefteyeX(poseData.get("le_eye").get("x").doubleValue())
                        .lefteyeY(poseData.get("le_eye").get("y").doubleValue())
                        .lefteyeZ(poseData.get("le_eye").get("z").doubleValue())
                        .righteyeX(poseData.get("re_eye").get("x").doubleValue())
                        .righteyeY(poseData.get("re_eye").get("y").doubleValue())
                        .righteyeZ(poseData.get("re_eye").get("z").doubleValue())
                        .build()
        );
        return PresetResponse.from(saved);
    }

    /**
     * 자동 프리셋 저장
     * - 최신 PostureStats의 좌표 컬럼 값 사용
     */
    public PresetResponse createFromStats(PresetRequest req) {
        Profile profile = profileRepo.findById(req.getProfileId())
                .orElseThrow(() -> new IllegalArgumentException("프로필이 없습니다: " + req.getProfileId()));

        PostureStats stats = statsRepo.findTopByProfileIdOrderByStartAtDesc(req.getProfileId())
                .orElseThrow(() -> new IllegalStateException("통계 데이터가 없습니다: " + req.getProfileId()));

        Preset saved = presetRepo.save(
                Preset.builder()
                        .profile(profile)
                        .name(req.getName())
                        .lefteyeX(stats.getLefteyeX())
                        .lefteyeY(stats.getLefteyeY())
                        .lefteyeZ(stats.getLefteyeZ())
                        .righteyeX(stats.getRighteyeX())
                        .righteyeY(stats.getRighteyeY())
                        .righteyeZ(stats.getRighteyeZ())
                        .build()
        );
        return PresetResponse.from(saved);
    }

    /**
     * 프로필별 프리셋 목록 조회
     */
    @Transactional(readOnly = true)
    public List<PresetResponse> getPresetsByProfile(Long profileId) {
        return presetRepo.findByProfile_Id(profileId).stream()
                .map(PresetResponse::from)
                .collect(Collectors.toList());
    }

    /**
     * 프리셋 수정
     */
    public PresetResponse updatePreset(Long presetId, PresetRequest req) {
        Preset preset = presetRepo.findById(presetId)
                .orElseThrow(() -> new IllegalArgumentException("프리셋이 없습니다: " + presetId));

        if (!preset.getProfile().getId().equals(req.getProfileId())) {
            throw new IllegalArgumentException("프리셋이 해당 프로필에 속하지 않습니다");
        }
        preset.setName(req.getName());
        return PresetResponse.from(presetRepo.save(preset));
    }

    /**
     * 프리셋 삭제
     */
    public void deletePreset(Long presetId) {
        presetRepo.deleteById(presetId);
    }

    /**
     * 프리셋 단건 조회
     */
    @Transactional(readOnly = true)
    public PresetResponse getPresetResponse(Long presetId) {
        return presetRepo.findById(presetId)
                .map(PresetResponse::from)
                .orElseThrow(() -> new IllegalArgumentException("프리셋이 없습니다: " + presetId));
    }

    /**
     * profileId에 매칭된 device의 serialNumber 조회
     */
    @Transactional(readOnly = true)
    public String getSerialNumberByProfileId(Long profileId) {
        return deviceRepo.findByProfileId_Id(profileId).stream()
                .findFirst()
                .map(Device::getSerialNumber)
                .orElseThrow(() -> new IllegalArgumentException(
                        "해당 프로필에 연결된 디바이스가 없습니다. profileId=" + profileId
                ));
    }

    /**
     * 프리셋 좌표 조회
     */
    @Transactional(readOnly = true)
    public PresetCoordinate getPresetCoordinate(Long profileId, Long presetId) {
        Preset preset = presetRepo.findById(presetId)
                .orElseThrow(() -> new IllegalArgumentException("프리셋이 없습니다: " + presetId));

        if (!preset.getProfile().getId().equals(profileId)) {
            throw new IllegalArgumentException("프리셋이 해당 프로필에 속하지 않습니다");
        }

        return new PresetCoordinate(
                profileId,
                preset.getLefteyeX(),
                preset.getLefteyeY(),
                preset.getLefteyeZ(),
                preset.getRighteyeX(),
                preset.getRighteyeY(),
                preset.getRighteyeZ()
        );
    }
}
