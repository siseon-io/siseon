package siseon.backend.dto;

import lombok.*;
import siseon.backend.domain.main.Preset;

import java.time.LocalDateTime;
import java.util.Map;

@Getter @Setter
@NoArgsConstructor @AllArgsConstructor @Builder
public class PresetResponse {

    private Long    presetId;
    private Long    profileId;
    private String  name;
    private Map<String,Object> monitorCoord;
    private LocalDateTime createdAt;
    private LocalDateTime updatedAt;

    public static PresetResponse from(Preset e) {
        return PresetResponse.builder()
                .presetId(e.getPresetId())
                .profileId(e.getProfile().getId())
                .name(e.getName())
                .monitorCoord(e.getMonitorCoord())
                .createdAt(e.getCreatedAt())
                .updatedAt(e.getUpdatedAt())
                .build();
    }
}
