package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import siseon.backend.domain.batch.PostureStats;
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

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
@Transactional
public class PresetService {

    private final ProfileRepository      profileRepo;
    private final RawPostureRepository   rawRepo;
    private final PostureStatsRepository statsRepo;
    private final PresetRepository       presetRepo;

    // manual preset
    public PresetResponse createFromRaw(PresetRequest req) {
        Profile profile = profileRepo.findById(req.getProfileId())
                .orElseThrow(() -> new IllegalArgumentException("프로필이 없습니다: " + req.getProfileId()));

        RawPosture latestRaw = rawRepo
                .findTopByProfileIdOrderByCollectedAtDesc(req.getProfileId())
                .orElseThrow(() -> new IllegalStateException("raw_posture 데이터가 없습니다: " + req.getProfileId()));

        @SuppressWarnings("unchecked")
        Map<String,Number> rawMap = latestRaw.getMonitorCoord();
        // Number → Object 매핑
        Map<String,Object> coord = rawMap.entrySet().stream()
                .collect(Collectors.toMap(
                        Map.Entry::getKey,
                        Map.Entry::getValue
                ));

        Preset saved = presetRepo.save(
                Preset.builder()
                        .profile(profile)
                        .name(req.getName())
                        .monitorCoord(coord)
                        .build()
        );
        return PresetResponse.from(saved);
    }

    // push confirm preset
    public PresetResponse createFromStats(PresetRequest req) {
        Profile profile = profileRepo.findById(req.getProfileId())
                .orElseThrow(() -> new IllegalArgumentException("프로필이 없습니다: " + req.getProfileId()));

        PostureStats stats = statsRepo
                .findTopByProfileIdOrderByStartAtDesc(req.getProfileId())
                .orElseThrow(() -> new IllegalStateException("통계 데이터가 없습니다: " + req.getProfileId()));

        Map<String,Object> coord = stats.getMonitorCoord();

        Preset saved = presetRepo.save(
                Preset.builder()
                        .profile(profile)
                        .name(req.getName())
                        .monitorCoord(coord)
                        .build()
        );
        return PresetResponse.from(saved);
    }

    // profile <-> presets
    @Transactional(readOnly = true)
    public List<PresetResponse> getPresetsByProfile(Long profileId) {
        return presetRepo.findByProfile_Id(profileId).stream()
                .map(PresetResponse::from)
                .collect(Collectors.toList());
    }

    // update
    public PresetResponse updatePreset(Long presetId, PresetRequest req) {
        Preset preset = presetRepo.findById(presetId)
                .orElseThrow(() -> new IllegalArgumentException("프리셋이 없습니다: " + presetId));

        if (!preset.getProfile().getId().equals(req.getProfileId())) {
            throw new IllegalArgumentException("프리셋이 해당 프로필에 속하지 않습니다");
        }
        preset.setName(req.getName());
        return PresetResponse.from(presetRepo.save(preset));
    }

    // delete
    public void deletePreset(Long presetId) {
        presetRepo.deleteById(presetId);
    }

    // get
    @Transactional(readOnly = true)
    public PresetResponse getPresetResponse(Long presetId) {
        return presetRepo.findById(presetId)
                .map(PresetResponse::from)
                .orElseThrow(() -> new IllegalArgumentException("프리셋이 없습니다: " + presetId));
    }

    // get coordinate
    @Transactional(readOnly = true)
    public PresetCoordinate getPresetCoordinate(Long profileId, Long presetId) {
        Preset preset = presetRepo.findById(presetId)
                .orElseThrow(() -> new IllegalArgumentException("프리셋이 없습니다: " + presetId));

        if (!preset.getProfile().getId().equals(profileId)) {
            throw new IllegalArgumentException("프리셋이 해당 프로필에 속하지 않습니다");
        }

        Map<String,Object> coord = preset.getMonitorCoord();
        double x = ((Number)coord.get("x")).doubleValue();
        double y = ((Number)coord.get("y")).doubleValue();
        double z = ((Number)coord.get("z")).doubleValue();

        return new PresetCoordinate(x, y, z);
    }
}