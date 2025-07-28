package siseon.backend.dto;

import lombok.*;

import java.time.LocalDateTime;
import java.util.Map;

@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class PresetResponse {

    private Long presetId;
    private Long deviceId;
    private String name;
    private Map<String, Object> position;
    private LocalDateTime createdAt;

    public static PresetResponse fromEntity(siseon.backend.domain.Preset preset) {
        return PresetResponse.builder()
                .presetId(preset.getPresetId())
                .deviceId(preset.getDeviceId())
                .name(preset.getName())
                .position(preset.getPosition())
                .createdAt(preset.getCreatedAt())
                .build();
    }
}