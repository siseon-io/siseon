package siseon.backend.dto;

import lombok.*;
import siseon.backend.domain.main.Preset;

import java.time.LocalDateTime;

@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class PresetResponse {

    private Long presetId;
    private Long profileId;
    private String name;

    private double lefteyeX;
    private double lefteyeY;
    private double lefteyeZ;
    private double righteyeX;
    private double righteyeY;
    private double righteyeZ;

    private LocalDateTime createdAt;
    private LocalDateTime updatedAt;

    public static PresetResponse from(Preset e) {
        return PresetResponse.builder()
                .presetId(e.getPresetId())
                .profileId(e.getProfile().getId())
                .name(e.getName())
                .lefteyeX(e.getLefteyeX())
                .lefteyeY(e.getLefteyeY())
                .lefteyeZ(e.getLefteyeZ())
                .righteyeX(e.getRighteyeX())
                .righteyeY(e.getRighteyeY())
                .righteyeZ(e.getRighteyeZ())
                .createdAt(e.getCreatedAt())
                .updatedAt(e.getUpdatedAt())
                .build();
    }
}
