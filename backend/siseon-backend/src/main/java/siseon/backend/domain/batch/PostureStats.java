package siseon.backend.domain.batch;

import jakarta.persistence.*;
import lombok.*;
import siseon.backend.config.JsonConverter;

import java.time.LocalDateTime;
import java.util.Map;

@Entity
@Table(
        name = "posture_stats",
        indexes = {
                @Index(name = "idx_posture_stats_profile_endat", columnList = "profile_id, end_at"),
                // 조회 최적화용(선택): 프로필+끝시간+처리여부
                @Index(name = "idx_ps_bad_scan", columnList = "profile_id, end_at, processed_bad"),
                @Index(name = "idx_ps_good_scan", columnList = "profile_id, end_at, processed_good")
        }
)
@Getter @Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class PostureStats {

    @Id @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name = "profile_id", nullable = false)
    private Long profileId;

    @Column(name = "monitor_coord", columnDefinition = "JSON", nullable = false)
    @Convert(converter = JsonConverter.class)
    private Map<String, Object> monitorCoord;

    @Column(name = "user_coord", columnDefinition = "JSON")
    @Convert(converter = JsonConverter.class)
    private Map<String, Object> userCoord;

    @Column(name = "start_at", nullable = false)
    private LocalDateTime startAt;

    @Column(name = "end_at", nullable = false)
    private LocalDateTime endAt;

    private int durationSeconds;
    private int slotIndex;

    // AI가 나중에 채우므로 Nullable 허용
    @Column(name = "valid_posture")
    private Boolean validPosture;

    // 알림에 사용 여부 플래그(기본 false)
    @Builder.Default
    @Column(name = "processed_bad", nullable = false)
    private boolean processedBad = false;

    @Builder.Default
    @Column(name = "processed_good", nullable = false)
    private boolean processedGood = false;

}
