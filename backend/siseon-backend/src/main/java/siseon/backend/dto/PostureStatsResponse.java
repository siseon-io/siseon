package siseon.backend.dto;

import lombok.*;
import siseon.backend.domain.batch.PostureStats;

import java.time.LocalDateTime;
import java.util.Map;

@Getter
@Builder
@AllArgsConstructor
@NoArgsConstructor
public class PostureStatsResponse {
    private Long id;
    private Long profileId;
    private Map<String, Object> monitorCoord;
    private Map<String, Object> userCoord;
    private LocalDateTime startAt;
    private LocalDateTime endAt;
    private int durationSeconds;
    private int slotIndex;

    public static PostureStatsResponse from(PostureStats entity) {
        return PostureStatsResponse.builder()
                .id(entity.getId())
                .profileId(entity.getProfileId())
                .monitorCoord(entity.getMonitorCoord())
                .userCoord(entity.getUserCoord())
                .startAt(entity.getStartAt())
                .endAt(entity.getEndAt())
                .durationSeconds(entity.getDurationSeconds())
                .slotIndex(entity.getSlotIndex())
                .build();
    }
}
